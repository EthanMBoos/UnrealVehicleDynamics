# Repository instructions

## Product and scope discipline

Keep this repository bare bones. Its purpose is to let a controls engineer do
three things quickly: change a small set of plant or controller parameters, run
a representative simulation, and inspect whether the response improved or
degraded. The simulator provides understandable and repeatable engineering
evidence; it does not need to claim that the plant is a trusted digital twin.

Prefer one demonstrated workflow over broad configurability. Add a signal,
setting, experiment, or abstraction only when the current example uses it and
the result appears in the saved engineering output. Do not build general
scenario systems, parameter-management infrastructure, dashboards, plugin
frameworks, or exhaustive model options in anticipation of possible users.
Correct physical or numerical errors when found, but let feedback from working
runs choose the next capability.

For the current boat work, the useful closed-loop result is intentionally
narrow: one ArduRover ACRO speed-and-turn maneuver showing desired and achieved
speed, desired and achieved yaw rate, left/right motor output, plant truth,
ArduRover estimate, and timing health. Report measurements and saturation
clearly without declaring that a controller or plant is universally good.

## Extension discipline

Keep the repository easy to understand, not universally configurable. A new
vehicle is allowed to require ordinary C++ and a few explicit integration edits.
Do not refactor in advance merely because students or future users may add
vehicles, controllers, experiments, or visual models. The Aerosonde and Otter
are the working examples: the aircraft demonstrates Chaos-owned motion, while
the Otter demonstrates plant-owned motion committed to Unreal for rendering.
New work should follow the closer of those two patterns until a real case proves
that neither fits.

A new physical model should normally add concrete files such as:

```text
core/include/uvd/<vehicle>.hpp
core/src/<vehicle>.cpp
core/src/<vehicle>_integrator.cpp
examples/<vehicles>/<model>.json
tests/<vehicle>_test.cpp
```

Wire it into the CLI, Unreal runtime, truth output, launcher, and controller
session with direct, reviewable branches. Some repetition is preferable to an
abstract plant hierarchy whose state, controls, solver ownership, or physical
assumptions are unclear. A different ArduPilot mode usually needs only a small
parameter/session change. Another ArduPilot vehicle may use another explicit
container and session. A controller with a genuinely different protocol may
add its own transport boundary without changing the physical plants.

Refactor only after implemented cases create demonstrated pain. Good triggers
include three plants repeating the same run-setting parsing, multiple copies of
the same lockstep exchange or truth writer, an Unreal component that can no
longer be reviewed safely, recurring merge conflicts in the same central
switch, or two controller paths needing identical session/report behavior.
Extract only the repeated part at that point. Likely small seams are common run
settings, shared ArduPilot state exchange, or separate per-vehicle Unreal
runtime files; even then, a universal `IPlant`, factory, or plugin system may
not be appropriate because vehicles can own and advance state differently.

For student or early-user work, finish one narrow demonstrated loop, document
the commands, and let users try it before reorganizing the repository. Observe
where they actually struggle. Fix correctness problems immediately, but let
repeated use—not hypothetical flexibility—choose the next abstraction.

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
