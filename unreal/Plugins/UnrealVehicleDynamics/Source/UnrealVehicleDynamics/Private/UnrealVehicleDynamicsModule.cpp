#include "Components/BoxComponent.h"
#include "Engine/World.h"
#include "EngineUtils.h"
#include "GerstnerWaterWaves.h"
#include "GameFramework/Actor.h"
#include "HAL/IConsoleManager.h"
#include "Misc/CommandLine.h"
#include "Misc/Parse.h"
#include "Modules/ModuleManager.h"
#include "PhysicsEngine/PhysicsSettings.h"
#include "UvdVehicleComponent.h"
#include "WaterBodyOceanActor.h"

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
  static void SpawnVehicle(UWorld* World, const FName Name,
                           const FString& RunPath, const FString& TruthPath,
                           const bool ConfigureWorld,
                           const bool OpenLoop = false,
                           const int32 ReleaseParticipants = 1) {
    FActorSpawnParameters SpawnParameters;
    SpawnParameters.Name = Name;
    AActor* Vehicle = World->SpawnActor<AActor>(SpawnParameters);
    if (!Vehicle) {
      return;
    }
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
    Simulation->ConfigureWorld = ConfigureWorld;
    Simulation->SynchronizedReleaseParticipants = ReleaseParticipants;
    Simulation->RegisterComponent();
  }

  void OnWorldInitialized(UWorld* World,
                          const UWorld::InitializationValues) const {
    if (!World || !World->IsGameWorld()) {
      return;
    }
    if (FParse::Param(FCommandLine::Get(), TEXT("UvdVisualWaves"))) {
      for (TActorIterator<AWaterBodyOcean> Ocean(World); Ocean; ++Ocean) {
        // Decorative waves only. The C++ marine solver keeps its own datum.
        UGerstnerWaterWaves* Waves = NewObject<UGerstnerWaterWaves>(*Ocean);
        UGerstnerWaterWaveGeneratorSimple* Generator =
            NewObject<UGerstnerWaterWaveGeneratorSimple>(Waves);
        Generator->NumWaves = 8;
        Generator->Seed = 17;
        Generator->MinWavelength = 200.0F;
        Generator->MaxWavelength = 1200.0F;
        Generator->MinAmplitude = 5.0F;
        Generator->MaxAmplitude = 15.0F;
        Generator->WindAngleDeg = 30.0F;
        Generator->DirectionAngularSpreadDeg = 45.0F;
        Waves->GerstnerWaveGenerator = Generator;
        Waves->RecomputeWaves(false);
        Ocean->SetWaterWaves(Waves);
        UE_LOG(LogTemp, Display,
               TEXT("UVD visual waves enabled: %s, 8 waves, 2-12 m wavelengths, "
                    "5-15 cm component amplitudes; boat physics unchanged"),
               *Ocean->GetName());
      }
    }
    FString PlaneRunPath;
    FString OtterRunPath;
    if (FParse::Value(FCommandLine::Get(), TEXT("UvdPlaneRun="),
                      PlaneRunPath) &&
        FParse::Value(FCommandLine::Get(), TEXT("UvdOtterRun="),
                      OtterRunPath)) {
      FString PlaneTruthPath;
      FString OtterTruthPath;
      FParse::Value(FCommandLine::Get(), TEXT("UvdPlaneTruth="),
                    PlaneTruthPath);
      FParse::Value(FCommandLine::Get(), TEXT("UvdOtterTruth="),
                    OtterTruthPath);
      SpawnVehicle(World, TEXT("UvdOtter"), OtterRunPath, OtterTruthPath, true,
                   false, 2);
      SpawnVehicle(World, TEXT("UvdPlane"), PlaneRunPath, PlaneTruthPath, false,
                   false, 2);
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
    SpawnVehicle(World, TEXT("UvdVehicle"), RunPath, TruthPath, true, OpenLoop);
  }

  FDelegateHandle WorldDelegate;
};

IMPLEMENT_MODULE(FUnrealVehicleDynamicsModule, UnrealVehicleDynamics)
