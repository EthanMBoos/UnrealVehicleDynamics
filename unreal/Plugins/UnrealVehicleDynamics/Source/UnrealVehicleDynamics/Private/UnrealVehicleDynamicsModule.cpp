#include "Components/BoxComponent.h"
#include "Engine/World.h"
#include "GameFramework/Actor.h"
#include "HAL/IConsoleManager.h"
#include "Misc/CommandLine.h"
#include "Misc/Parse.h"
#include "Modules/ModuleManager.h"
#include "PhysicsEngine/PhysicsSettings.h"
#include "UvdVehicleComponent.h"

class FUnrealVehicleDynamicsModule final : public IModuleInterface {
 public:
  void StartupModule() override {
    double FixedDtS = 0.0;
    if (FParse::Value(FCommandLine::Get(), TEXT("UvdFixedDt="), FixedDtS) &&
        FMath::IsFinite(FixedDtS) && FixedDtS > 0.0) {
      UPhysicsSettings* Settings = UPhysicsSettings::Get();
      Settings->bTickPhysicsAsync = true;
      Settings->AsyncFixedTimeStepSize = static_cast<float>(FixedDtS);
      Settings->bSubstepping = false;
      Settings->bSubsteppingAsync = false;
    }
    WorldDelegate = FWorldDelegates::OnPostWorldInitialization.AddRaw(
        this, &FUnrealVehicleDynamicsModule::OnWorldInitialized);
  }

  void ShutdownModule() override {
    FWorldDelegates::OnPostWorldInitialization.Remove(WorldDelegate);
  }

 private:
  void OnWorldInitialized(UWorld* World,
                          const UWorld::InitializationValues) const {
    if (!World || !World->IsGameWorld()) {
      return;
    }
    FString RunPath;
    if (!FParse::Value(FCommandLine::Get(), TEXT("UvdRun="), RunPath)) {
      return;
    }
    FString TruthPath;
    FParse::Value(FCommandLine::Get(), TEXT("UvdTruth="), TruthPath);
    const bool OpenLoop =
        FParse::Param(FCommandLine::Get(), TEXT("UvdOpenLoop"));
    AActor* Vehicle = World->SpawnActor<AActor>();
    UBoxComponent* Body =
        NewObject<UBoxComponent>(Vehicle, TEXT("VehiclePhysicsBody"));
    Body->SetBoxExtent(FVector{40.0, 6.0, 6.0});
    Body->SetCollisionProfileName(TEXT("PhysicsActor"));
    Vehicle->SetRootComponent(Body);
    Body->RegisterComponent();
    Body->SetSimulatePhysics(true);

    UUvdVehicleComponent* Simulation =
        NewObject<UUvdVehicleComponent>(Vehicle, TEXT("VehicleDynamics"));
    Simulation->RunConfigPath = RunPath;
    Simulation->TruthOutputPath = TruthPath;
    Simulation->OpenLoop = OpenLoop;
    Simulation->RegisterComponent();
  }

  FDelegateHandle WorldDelegate;
};

IMPLEMENT_MODULE(FUnrealVehicleDynamicsModule, UnrealVehicleDynamics)
