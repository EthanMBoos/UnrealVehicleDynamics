#include "UvdCameraPawn.h"

#include "Camera/CameraComponent.h"
#include "Components/InputComponent.h"
#include "Components/SceneComponent.h"
#include "Engine/Engine.h"
#include "Engine/World.h"
#include "EngineUtils.h"
#include "Framework/Application/NavigationConfig.h"
#include "Framework/Application/SlateApplication.h"
#include "GameFramework/PlayerController.h"
#include "InputCoreTypes.h"
#include "UvdVehicleComponent.h"

DEFINE_LOG_CATEGORY_STATIC(LogUvdCamera, Log, All);

namespace {

constexpr float LookSensitivity = 0.25F;

const TCHAR* ModeName(EUvdCameraMode Mode) {
  switch (Mode) {
    case EUvdCameraMode::Chase:
      return TEXT("Chase");
    case EUvdCameraMode::Orbit:
      return TEXT("Orbit");
    case EUvdCameraMode::Onboard:
      return TEXT("Onboard");
    case EUvdCameraMode::Side:
      return TEXT("Side");
  }
  return TEXT("Unknown");
}

TArray<AActor*> VehicleTargets(UWorld* World) {
  TArray<AActor*> Vehicles;
  for (TActorIterator<AActor> Actor(World); Actor; ++Actor) {
    if (Actor->FindComponentByClass<UUvdVehicleComponent>()) {
      Vehicles.Add(*Actor);
    }
  }
  Vehicles.Sort([](const AActor& Left, const AActor& Right) {
    return Left.GetName() < Right.GetName();
  });
  return Vehicles;
}

}  // namespace

AUvdCameraPawn::AUvdCameraPawn() {
  PrimaryActorTick.bCanEverTick = true;
  PrimaryActorTick.bTickEvenWhenPaused = true;

  SceneRoot = CreateDefaultSubobject<USceneComponent>(TEXT("CameraRoot"));
  SetRootComponent(SceneRoot);
  Camera = CreateDefaultSubobject<UCameraComponent>(TEXT("Camera"));
  Camera->SetupAttachment(SceneRoot);
  Camera->SetFieldOfView(70.0F);
  Camera->PostProcessSettings.bOverride_MotionBlurAmount = true;
  Camera->PostProcessSettings.MotionBlurAmount = 0.0F;
  Camera->SetPostProcessBlendWeight(1.0F);
}

void AUvdCameraPawn::BeginPlay() {
  Super::BeginPlay();
  if (FSlateApplication::IsInitialized()) {
    // This game reserves Tab for vehicle selection. Without this, Slate moves
    // native keyboard focus after the first press and later Tabs never reach
    // the pawn even in game-only input mode.
    TSharedRef<FNavigationConfig> Navigation =
        FSlateApplication::Get().GetNavigationConfig();
    PreviousTabNavigation = Navigation->bTabNavigation;
    Navigation->bTabNavigation = false;
    ChangedTabNavigation = true;
  }
  FindTarget();
  RefreshFraming(true);
  AttachToTarget();
}

void AUvdCameraPawn::EndPlay(const EEndPlayReason::Type EndPlayReason) {
  if (ChangedTabNavigation && FSlateApplication::IsInitialized()) {
    FSlateApplication::Get().GetNavigationConfig()->bTabNavigation =
        PreviousTabNavigation;
  }
  Super::EndPlay(EndPlayReason);
}

void AUvdCameraPawn::PossessedBy(AController* NewController) {
  Super::PossessedBy(NewController);
  if (APlayerController* Player = Cast<APlayerController>(NewController)) {
    // GameAndUI lets Slate claim Tab for widget navigation after its first
    // press. Keep keyboard focus on the viewport so vehicle cycling remains
    // repeatable; the cursor can still stay visible in game-only mode.
    Player->SetInputMode(FInputModeGameOnly{});
    Player->SetShowMouseCursor(true);
  }
}

void AUvdCameraPawn::Tick(float DeltaSeconds) {
  Super::Tick(DeltaSeconds);
  if (!Target.IsValid()) {
    FindTarget();
  }
  UpdateCamera();
}

void AUvdCameraPawn::SetupPlayerInputComponent(
    UInputComponent* PlayerInputComponent) {
  Super::SetupPlayerInputComponent(PlayerInputComponent);

  UE_LOG(LogUvdCamera, Display, TEXT("UVD camera controls ready"));
  ShowMessage(
      TEXT("Camera ready: 1 chase  2 orbit  3 onboard  4 side  |  "
           "RMB orbit  wheel zoom  |  "
           "C mode  Tab vehicle  F reacquire  Space pause"));

  PlayerInputComponent
      ->BindKey(EKeys::One, IE_Pressed, this, &AUvdCameraPawn::SelectChase)
      .bExecuteWhenPaused = true;
  PlayerInputComponent
      ->BindKey(EKeys::Two, IE_Pressed, this, &AUvdCameraPawn::SelectOrbit)
      .bExecuteWhenPaused = true;
  PlayerInputComponent
      ->BindKey(EKeys::Three, IE_Pressed, this, &AUvdCameraPawn::SelectOnboard)
      .bExecuteWhenPaused = true;
  PlayerInputComponent
      ->BindKey(EKeys::Four, IE_Pressed, this, &AUvdCameraPawn::SelectSide)
      .bExecuteWhenPaused = true;
  PlayerInputComponent
      ->BindKey(EKeys::C, IE_Pressed, this, &AUvdCameraPawn::CycleMode)
      .bExecuteWhenPaused = true;
  PlayerInputComponent
      ->BindKey(EKeys::Tab, IE_Pressed, this, &AUvdCameraPawn::CycleTarget)
      .bExecuteWhenPaused = true;
  PlayerInputComponent
      ->BindKey(EKeys::F, IE_Pressed, this, &AUvdCameraPawn::Reacquire)
      .bExecuteWhenPaused = true;
  PlayerInputComponent
      ->BindKey(EKeys::SpaceBar, IE_Pressed, this, &AUvdCameraPawn::TogglePause)
      .bExecuteWhenPaused = true;
  PlayerInputComponent
      ->BindKey(EKeys::RightMouseButton, IE_Pressed, this,
                &AUvdCameraPawn::BeginLook)
      .bExecuteWhenPaused = true;
  PlayerInputComponent
      ->BindKey(EKeys::RightMouseButton, IE_Released, this,
                &AUvdCameraPawn::EndLook)
      .bExecuteWhenPaused = true;
  PlayerInputComponent
      ->BindKey(EKeys::MouseScrollUp, IE_Pressed, this, &AUvdCameraPawn::ZoomIn)
      .bExecuteWhenPaused = true;
  PlayerInputComponent
      ->BindKey(EKeys::MouseScrollDown, IE_Pressed, this,
                &AUvdCameraPawn::ZoomOut)
      .bExecuteWhenPaused = true;

  PlayerInputComponent
      ->BindAxisKey(EKeys::MouseX, this, &AUvdCameraPawn::LookHorizontal)
      .bExecuteWhenPaused = true;
  PlayerInputComponent
      ->BindAxisKey(EKeys::MouseY, this, &AUvdCameraPawn::LookVertical)
      .bExecuteWhenPaused = true;
}

void AUvdCameraPawn::SetTarget(AActor* InTarget) {
  if (!InTarget) {
    return;
  }
  DetachFromActor(FDetachmentTransformRules::KeepWorldTransform);
  Target = InTarget;
  RefreshFraming(true);
  AttachToTarget();
  const FString Message =
      FString::Printf(TEXT("Camera target: %s"), *InTarget->GetName());
  UE_LOG(LogUvdCamera, Display, TEXT("%s"), *Message);
  ShowMessage(Message);
}

void AUvdCameraPawn::SelectChase() { SetMode(EUvdCameraMode::Chase); }
void AUvdCameraPawn::SelectOrbit() {
  SetMode(EUvdCameraMode::Orbit);
  OrbitYawDeg = 135.0F;
  OrbitPitchDeg = 20.0F;
  OrbitDistanceCm = ChaseDistanceCm;
}
void AUvdCameraPawn::SelectOnboard() { SetMode(EUvdCameraMode::Onboard); }
void AUvdCameraPawn::SelectSide() { SetMode(EUvdCameraMode::Side); }

void AUvdCameraPawn::CycleMode() {
  SetMode(static_cast<EUvdCameraMode>((static_cast<uint8>(Mode) + 1) % 4));
}

void AUvdCameraPawn::CycleTarget() {
  const TArray<AActor*> Vehicles = VehicleTargets(GetWorld());
  if (Vehicles.IsEmpty()) {
    ShowMessage(TEXT("Could not find a vehicle camera target"));
    return;
  }
  const int32 Current = Vehicles.IndexOfByKey(Target.Get());
  SetTarget(Vehicles[(Current + 1) % Vehicles.Num()]);
}

void AUvdCameraPawn::Reacquire() { SetMode(EUvdCameraMode::Chase); }

void AUvdCameraPawn::TogglePause() {
  UUvdVehicleComponent* Simulation =
      Target.IsValid() ? Target->FindComponentByClass<UUvdVehicleComponent>()
                       : nullptr;
  if (!Simulation) {
    ShowMessage(TEXT("Could not pause: vehicle simulation unavailable"));
    return;
  }
  const bool Paused = Simulation->IsSimulationPaused();
  Simulation->SetSimulationPaused(!Paused);
  ShowMessage(Paused
                  ? TEXT("Simulation resumed")
                  : TEXT("Simulation paused - camera controls remain active"));
}

void AUvdCameraPawn::BeginLook() {
  if (Mode == EUvdCameraMode::Chase || Mode == EUvdCameraMode::Side) {
    SetMode(EUvdCameraMode::Orbit);
  }
  if (Mode != EUvdCameraMode::Orbit) {
    return;
  }
  Looking = true;
  if (APlayerController* Player = Cast<APlayerController>(GetController())) {
    float MouseX = 0.0F;
    float MouseY = 0.0F;
    if (Player->GetMousePosition(MouseX, MouseY)) {
      SavedMouseX = FMath::RoundToInt(MouseX);
      SavedMouseY = FMath::RoundToInt(MouseY);
    }
    Player->SetShowMouseCursor(false);
    Player->SetInputMode(FInputModeGameOnly{});
  }
}

void AUvdCameraPawn::EndLook() {
  Looking = false;
  if (APlayerController* Player = Cast<APlayerController>(GetController())) {
    Player->SetInputMode(FInputModeGameOnly{});
    Player->SetShowMouseCursor(true);
    Player->SetMouseLocation(SavedMouseX, SavedMouseY);
  }
}

void AUvdCameraPawn::ZoomIn() {
  if (Mode == EUvdCameraMode::Orbit) {
    OrbitDistanceCm =
        FMath::Clamp(OrbitDistanceCm * 0.85F, 0.8F * TargetRadiusCm,
                     FMath::Max(5000.0F, 20.0F * TargetRadiusCm));
  }
}

void AUvdCameraPawn::ZoomOut() {
  if (Mode == EUvdCameraMode::Orbit) {
    OrbitDistanceCm =
        FMath::Clamp(OrbitDistanceCm / 0.85F, 0.8F * TargetRadiusCm,
                     FMath::Max(5000.0F, 20.0F * TargetRadiusCm));
  }
}

void AUvdCameraPawn::LookHorizontal(float Value) {
  if (!Looking) {
    return;
  }
  if (Mode == EUvdCameraMode::Orbit) {
    OrbitYawDeg += Value * LookSensitivity;
  }
}

void AUvdCameraPawn::LookVertical(float Value) {
  if (!Looking) {
    return;
  }
  if (Mode == EUvdCameraMode::Orbit) {
    OrbitPitchDeg =
        FMath::Clamp(OrbitPitchDeg + Value * LookSensitivity, -80.0F, 80.0F);
  }
}

void AUvdCameraPawn::FindTarget() {
  const TArray<AActor*> Vehicles = VehicleTargets(GetWorld());
  if (!Vehicles.IsEmpty()) {
    SetTarget(Vehicles[0]);
  }
}

void AUvdCameraPawn::AttachToTarget() {
  if (!Target.IsValid() || !Target->GetRootComponent() ||
      GetAttachParentActor() == Target.Get()) {
    return;
  }
  AttachToComponent(Target->GetRootComponent(),
                    FAttachmentTransformRules::KeepWorldTransform);
}

void AUvdCameraPawn::RefreshFraming(bool ResetOrbitDistance) {
  if (!Target.IsValid()) {
    return;
  }
  const FBox Bounds =
      Target->CalculateComponentsBoundingBoxInLocalSpace(true, false);
  if (Bounds.IsValid) {
    FocusLocalCm = Bounds.GetCenter();
    TargetRadiusCm = FMath::Max(50.0F, Bounds.GetExtent().Length());
  } else {
    FocusLocalCm = FVector::ZeroVector;
    TargetRadiusCm = 100.0F;
  }
  ChaseDistanceCm = FMath::Max(300.0F, 3.0F * TargetRadiusCm);
  ChaseHeightCm = 0.8F * TargetRadiusCm;
  if (ResetOrbitDistance) {
    OrbitDistanceCm = ChaseDistanceCm;
  }
}

void AUvdCameraPawn::SetMode(EUvdCameraMode NewMode) {
  AttachToTarget();
  if (NewMode == EUvdCameraMode::Orbit && Target.IsValid() &&
      GetAttachParentActor() == Target.Get()) {
    const FVector Offset = SceneRoot->GetRelativeLocation() - FocusLocalCm;
    if (!Offset.IsNearlyZero()) {
      OrbitDistanceCm =
          FMath::Clamp(Offset.Length(), 0.8F * TargetRadiusCm,
                       FMath::Max(5000.0F, 20.0F * TargetRadiusCm));
      const FRotator Direction = Offset.Rotation();
      OrbitYawDeg = Direction.Yaw;
      OrbitPitchDeg = FMath::Clamp(Direction.Pitch, -80.0F, 80.0F);
    }
  }
  Mode = NewMode;
  Camera->NotifyCameraCut();
  ShowMessage(FString::Printf(TEXT("Camera: %s"), ModeName(Mode)));
  UE_LOG(LogUvdCamera, Display, TEXT("Camera mode: %s"), ModeName(Mode));
}

void AUvdCameraPawn::UpdateCamera() {
  if (!Target.IsValid()) {
    return;
  }
  AttachToTarget();

  FVector DesiredLocationLocal;
  FRotator DesiredRotationLocal;
  if (Mode == EUvdCameraMode::Chase) {
    DesiredLocationLocal =
        FocusLocalCm + FVector{-ChaseDistanceCm, 0.0F, ChaseHeightCm};
    DesiredRotationLocal = (FocusLocalCm - DesiredLocationLocal).Rotation();
  } else if (Mode == EUvdCameraMode::Orbit) {
    const FVector Offset =
        FRotator{OrbitPitchDeg, OrbitYawDeg, 0.0F}.Vector() * OrbitDistanceCm;
    DesiredLocationLocal = FocusLocalCm + Offset;
    DesiredRotationLocal = (FocusLocalCm - DesiredLocationLocal).Rotation();
  } else if (Mode == EUvdCameraMode::Onboard) {
    const FBox Bounds =
        Target->CalculateComponentsBoundingBoxInLocalSpace(true, false);
    const FVector Extent =
        Bounds.IsValid
            ? Bounds.GetExtent()
            : FVector{TargetRadiusCm, TargetRadiusCm, TargetRadiusCm};
    DesiredLocationLocal =
        FocusLocalCm + FVector{0.25F * Extent.X, 0.0F, 0.65F * Extent.Z};
    DesiredRotationLocal = FRotator::ZeroRotator;
  } else {
    DesiredLocationLocal =
        FocusLocalCm + FVector{0.0F, ChaseDistanceCm, 0.15F * TargetRadiusCm};
    DesiredRotationLocal = (FocusLocalCm - DesiredLocationLocal).Rotation();
  }
  SceneRoot->SetRelativeLocationAndRotation(DesiredLocationLocal,
                                            DesiredRotationLocal);
}

void AUvdCameraPawn::ShowMessage(const FString& Message) const {
  if (GEngine) {
    GEngine->AddOnScreenDebugMessage(-1, 6.0F, FColor::White, Message);
  }
}
