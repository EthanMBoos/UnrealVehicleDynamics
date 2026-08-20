#pragma once

#include "GameFramework/GameModeBase.h"
#include "UvdGameMode.generated.h"

UCLASS()
class UVDHOST_API AUvdGameMode final : public AGameModeBase {
  GENERATED_BODY()

 public:
  AUvdGameMode();
  virtual void RestartPlayer(AController* NewPlayer) override;
};
