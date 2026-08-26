#pragma once

#include "Components/ActorComponent.h"
#include "UvdVehicleComponent.generated.h"

class UPrimitiveComponent;
struct FUvdVehicleRuntime;

UCLASS()
class UNREALVEHICLEDYNAMICS_API UUvdVehicleComponent : public UActorComponent {
  GENERATED_BODY()

 public:
  UUvdVehicleComponent();
  virtual ~UUvdVehicleComponent() override;
  virtual void BeginPlay() override;
  virtual void EndPlay(const EEndPlayReason::Type EndPlayReason) override;
  virtual void AsyncPhysicsTickComponent(float DeltaTime,
                                         float SimTime) override;

  FString RunConfigPath;
  FString TruthOutputPath;
  bool OpenLoop = false;
  bool ConfigureWorld = true;
  int32 SynchronizedReleaseParticipants = 1;

  void SetSimulationPaused(bool Paused);
  bool IsSimulationPaused() const { return SimulationPaused; }

 private:
  bool LoadConfiguration();
  bool OpenTruthOutput();
  bool ConfigureBody();
  bool ConfigureVisual();
  bool ConfigureCesium();
  void SetInitialState();

  UPROPERTY(Transient)
  TObjectPtr<UPrimitiveComponent> Body;

  FUvdVehicleRuntime* Runtime = nullptr;
  FVector PausedLinearVelocity = FVector::ZeroVector;
  FVector PausedAngularVelocity = FVector::ZeroVector;
  bool SimulationPaused = false;
};
