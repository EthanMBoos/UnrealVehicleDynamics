# Repository instructions

## Scope

Keep the repository bare bones. A controls engineer changes plant or controller
parameters, runs a representative simulation, and inspects the response.
Physical-fidelity claims require external data.

A signal, setting, or command belongs in the repository when a working example
uses it and the saved report shows the result. User feedback chooses the next
experiment. Correct physical and numerical errors when found.

The boat example is one ArduRover ACRO speed-and-turn run. Its report shows
speed and yaw-rate targets and feedback, left/right PWM, plant truth, saturation,
and exchange timing. Tracking error and saturation are measurements, not tune
acceptance limits.

## New vehicles and controllers

The Aerosonde uses Chaos-owned motion. The Otter uses core-owned motion and a
kinematic Unreal pose. A new vehicle follows the closer pattern and may require
ordinary C++ plus direct CLI, Unreal, launcher, and controller edits.

A model normally adds:

```text
core/include/uvd/<vehicle>.hpp
core/src/<vehicle>.cpp
core/src/<vehicle>_integrator.cpp
examples/<vehicles>/<model>.json
tests/<vehicle>_test.cpp
```

Wire the model into the CLI, Unreal runtime, truth output, launcher, and
controller session with direct branches. Repeat the small branch until two
working implementations need the same code. State ownership, controls, solver,
and physical assumptions must remain visible.

Refactor when one of these occurs:

- three plants repeat the same run parsing;
- lockstep exchange or truth writing is copied between controllers;
- one Unreal component becomes difficult to review;
- the same central branch causes recurring merge conflicts; or
- two reports need the same calculation.

Extract the repeated code only. Vehicle state advancement may remain separate.
A universal plant interface is unnecessary while the two working models have
different state owners.

For student work, finish one run, document its command and output, and let
someone use it before reorganizing the code.

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
