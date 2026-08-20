#include "UvdGameMode.h"

#include "UvdCameraPawn.h"

AUvdGameMode::AUvdGameMode() {
  // DEV NOTE: Keep the camera as the default pawn so PlayerController
  // possession creates its input component. Manual possession during world
  // initialization races that lifecycle.
  // https://dev.epicgames.com/documentation/en-us/unreal-engine/setting-up-user-inputs-in-unreal-engine
  DefaultPawnClass = AUvdCameraPawn::StaticClass();
}

void AUvdGameMode::RestartPlayer(AController* NewPlayer) {
  RestartPlayerAtTransform(NewPlayer, FTransform::Identity);
}
