#pragma once

#include "Components/ActorComponent.h"
#include "UvdAircraftComponent.generated.h"

class UPrimitiveComponent;
struct FUvdAircraftRuntime;

UCLASS()
class UNREALVEHICLEDYNAMICS_API UUvdAircraftComponent : public UActorComponent {
  GENERATED_BODY()

 public:
  UUvdAircraftComponent();
  virtual ~UUvdAircraftComponent() override;
  virtual void BeginPlay() override;
  virtual void EndPlay(const EEndPlayReason::Type EndPlayReason) override;
  virtual void AsyncPhysicsTickComponent(float DeltaTime,
                                         float SimTime) override;

  FString RunConfigPath;
  FString TruthOutputPath;

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

  FUvdAircraftRuntime* Runtime = nullptr;
  FVector PausedLinearVelocity = FVector::ZeroVector;
  FVector PausedAngularVelocity = FVector::ZeroVector;
  bool SimulationPaused = false;
};
