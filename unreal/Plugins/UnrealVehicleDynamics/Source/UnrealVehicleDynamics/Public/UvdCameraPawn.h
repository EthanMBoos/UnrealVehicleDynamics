#pragma once

#include "GameFramework/Pawn.h"
#include "UvdCameraPawn.generated.h"

class AActor;
class AController;
class APlayerController;
class UCameraComponent;
class USceneComponent;

enum class EUvdCameraMode : uint8 {
  Chase,
  Orbit,
  Onboard,
  Side,
};

UCLASS()
class UNREALVEHICLEDYNAMICS_API AUvdCameraPawn final : public APawn {
  GENERATED_BODY()

 public:
  AUvdCameraPawn();

  virtual void BeginPlay() override;
  virtual void PossessedBy(AController* NewController) override;
  virtual void Tick(float DeltaSeconds) override;
  virtual void SetupPlayerInputComponent(
      UInputComponent* PlayerInputComponent) override;

  void SetTarget(AActor* InTarget);

 private:
  void SelectChase();
  void SelectOrbit();
  void SelectOnboard();
  void SelectSide();
  void CycleMode();
  void Reacquire();
  void TogglePause();
  void BeginLook();
  void EndLook();
  void ZoomIn();
  void ZoomOut();
  void LookHorizontal(float Value);
  void LookVertical(float Value);
  void FindTarget();
  void AttachToTarget();
  void RefreshFraming(bool ResetOrbitDistance);
  void SetMode(EUvdCameraMode NewMode);
  void UpdateCamera();
  void ShowMessage(const FString& Message) const;

  UPROPERTY(VisibleAnywhere)
  TObjectPtr<USceneComponent> SceneRoot;

  UPROPERTY(VisibleAnywhere)
  TObjectPtr<UCameraComponent> Camera;

  TWeakObjectPtr<AActor> Target;
  EUvdCameraMode Mode = EUvdCameraMode::Chase;
  FVector FocusLocalCm = FVector::ZeroVector;
  float TargetRadiusCm = 100.0F;
  float ChaseDistanceCm = 600.0F;
  float ChaseHeightCm = 220.0F;
  float OrbitYawDeg = 180.0F;
  float OrbitPitchDeg = 20.0F;
  float OrbitDistanceCm = 600.0F;
  int32 SavedMouseX = 0;
  int32 SavedMouseY = 0;
  bool Looking = false;
};
