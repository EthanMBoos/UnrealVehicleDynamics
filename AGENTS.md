# Repository instructions

## Unreal development loop

- Do not use Unreal Hot Reload or treat a numbered module such as
  `libUnrealEditor-UnrealVehicleDynamics-0001.dylib` as a verified runtime build. A running editor
  can leave the module manifest pointing at an older binary even when compilation succeeds.
- Build runtime changes with `-NoHotReloadFromIDE`. The normal `./run.py` launcher already passes
  this flag.
- Before testing an Unreal change, start a fresh game process. If behavior looks stale, check
  `unreal/Binaries/*/UnrealEditor.modules` and the plugin's corresponding `UnrealEditor.modules`;
  both should name the current unnumbered module libraries.
- A successful compile is not runtime verification. For input, camera, Cesium, or ArduPilot
  changes, inspect the actual game log and exercise the affected behavior in the game window.
