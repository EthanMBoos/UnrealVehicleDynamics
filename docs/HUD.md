# In-game HUD

An in-game HUD could make the demonstrations easier to follow without opening
Mission Planner. It is optional. Ship landing, scripted aerobatics, racing, and
PX4 work should not wait for it.

The normal workflow stays the same:

```text
./run.py dual
    -> start fresh Unreal and ArduPilot processes
    -> wait for both vehicles
    -> run the configured simulation
    -> close everything and write the report
```

The HUD appears inside that run. It does not turn Unreal into a permanently
open workbench, create scenarios, or replace the launcher. Existing examples
should still run from start to finish without user input.

## Useful information

The HUD should answer a few basic questions:

- Which vehicles are connected?
- Which vehicle is the camera following?
- Which vehicle would receive a command?
- What phase of the run is active?
- Is ArduPilot healthy and armed in the expected mode?
- Are the controller and plant doing what the scenario requested?
- Did lockstep remain healthy?

For each vehicle, show its name, type, system ID, mode, arm state, connection
state, and run phase. In a dual run, keep both vehicle summaries visible even
when the camera follows only one.

The selected vehicle can have a larger panel with:

- attitude and heading;
- airspeed or groundspeed;
- altitude and vertical speed when relevant;
- controller target and feedback;
- plant truth;
- PWM or actuator output;
- saturation; and
- simulation time, missed or duplicate frames, and clock skew.

Controller estimates and plant truth must use different labels. Their
difference is often the measurement the run is meant to show.

ArduPilot messages should remain visible in a small event list. Mode changes,
arming failures, script messages, timing faults, and parameter changes are more
useful than a transient popup that disappears before the user can read it.

## Current demonstrations

The first HUD should be read-only and use the existing Plane, Otter, and dual
runs.

The Plane view needs mode, arm state, attitude, airspeed, roll command, control
surface output, and lockstep status.

The Otter view needs mode, arm state, speed target and feedback, yaw-rate target
and feedback, left and right motor PWM, saturation, and lockstep status.

The dual view adds two persistent vehicle summaries, the selected camera
target, and the measured step and clock skew. Pressing Tab may change the camera
target, but it must not silently change the target of a parameter or vehicle
command.

This first version is useful if a person can run `./run.py dual`, understand the
30-second experiment from the game window, and receive the same report that is
produced today.

## Controls

The current demonstrations should continue automatically after their
controllers are ready. A later interactive run may wait for a Start button or
offer Pause and Abort, but only when its launcher profile requests that
behavior.

Vehicle commands such as mode, Hold, RTL, Land, or Disarm must show the target
vehicle and system ID. Commands that can end or substantially change a run need
confirmation. The display should show whether ArduPilot accepted or rejected
the command.

A restart means closing the current processes and starting a new run directory.
Resetting actors and controllers in place risks carrying state from one test to
the next.

## Tuning

Real-time tuning can help with a dedicated tuning run, but a full parameter
editor is unnecessary.

Each tuning run should expose only the few ArduPilot parameters it is designed
to study. A tuning control shows:

- parameter name and purpose;
- units and allowed range;
- starting, current, and proposed values;
- whether it can change while running; and
- the target, feedback, output, and saturation plot used to judge the change.

The Python session sends the parameter write and checks the value reported back
by ArduPilot. The HUD must not report success before that check. Every attempted
change is written to the run output with its time, previous value, requested
value, returned value, and result.

The Otter is the best first tuning example. Its session already records
ArduRover speed and steering targets and feedback. The
[Rover tuning process](https://ardupilot.org/rover/docs/rover-tuning-process.html)
uses those signals to tune the speed and steering-rate controllers.

For Plane and VTOL, a guided
[AUTOTUNE](https://ardupilot.org/plane/docs/automatic-tuning-with-autotune.html)
display is more useful than general gain sliders. It should show prerequisites,
progress, target and actual rates, the values learned, and a clear abort action.

Starting the checked-in run again restores its original settings. A useful
change can be reviewed and saved later as a new parameter overlay or scenario.
Do not change gains during a scored aerobatics run.

## Runtime connection

The Python sessions should remain the only code that speaks MAVLink to the
autopilots. They already know the vehicle system ID, mode, arm state, telemetry,
and run phase.

Each session can send a small status stream to Unreal and receive requests from
the HUD. Unreal already has the plant truth and fixed-step timing. It combines
those values for display but does not need another MAVLink implementation.

```text
ArduPilot <-> Python session <-> Unreal HUD
                                  ^
                                  |
                         plant truth and timing
```

`run.py` still creates the run directory, starts the processes, releases the
scenario, stops it, and renders the report. HUD messages and parameter changes
are simply added to that run's saved output.

The Plane and Rover keep separate message streams and parameter state. Never
send vehicle commands or parameter changes to a broadcast system ID.

## Ship landing

The ship-landing run will need a more specific display. Show:

- the QuadPlane and Rover system IDs;
- whether the aircraft is receiving the Rover beacon position;
- beacon age;
- beacon position and actual deck landing point;
- aircraft position and velocity relative to the deck;
- ship heading and speed;
- landing offset;
- hold-off orbit;
- approach and abort directions; and
- the current takeoff or landing phase.

ArduPilot's
[moving-platform documentation](https://ardupilot.org/plane/docs/common-ship-landing.html)
describes the beacon, landing offset, hold-off point, approach angle, and pilot
inputs used during the landing. The HUD should make those states visible. It
should not invent a different landing procedure.

The run ends on touchdown, abort, or a maximum run time. Its report should add
deck-relative path, approach error, touchdown velocity, beacon age, clearance,
saturation, and lockstep timing.

## Scripted aerobatics

The aerobatics display should show the selected maneuver or schedule, its
arguments, a preview of the requested path, the current maneuver, and progress
through the schedule.

During the run, plot the requested and actual 3D paths, path error, attitude and
rate commands, speed, and saturation. Keep an Abort action visible. ArduPilot's
[scripted aerobatics documentation](https://ardupilot.org/plane/docs/common-scripted-aerobatics-4.4.html)
and Lua scripts remain the source for maneuver names, arguments, and behavior.

The report should score completion, path error by maneuver, rate tracking,
saturation, minimum altitude and airspeed, and any abort reason.

## Drone racing

Start with one checked-in course rather than a general course editor. Show the
next gate, lap and sector time, missed gates, penalties, and run validity. A
reference trajectory or ghost is useful for comparing controller changes.

The report should include lap consistency, time lost by sector, gate clearance,
collisions, saturation, and a replay of the vehicle path and commands.

## PX4

PX4 runs should use the same layout and measurements where possible. Keep PX4's
native modes, messages, parameters, and tuning behavior visible instead of
renaming them to look like ArduPilot.

The comparison is useful when both runs use the same plant, environment,
initial state, requested maneuver, and report calculations.

## What not to build

The HUD does not need to provide:

- firmware flashing;
- hardware, sensor, radio, or actuator calibration;
- arbitrary airframe setup;
- a general mission editor;
- unrestricted parameter import, reset, or editing;
- script file management;
- a MAVLink console; or
- general DataFlash analysis.

Mission Planner and QGroundControl remain useful while developing and debugging
new scenarios. They are not required once a published run contains its mission,
scripts, parameters, commands, and report.

## Suggested order

1. Add a read-only display to the current Plane, Otter, and dual runs.
2. Add the saved event list and explicit command target.
3. Add a dedicated Otter tuning run with a few approved parameters.
4. Add special displays only after ship landing, aerobatics, and racing runs
   work from the command line.
5. Reuse the useful parts for PX4.

The ArduPilot demonstration roadmap does not depend on any of this work. A new
demo should first run from `run.py`, stop cleanly, and produce a useful report.
Add HUD features after the demonstration works and users can say what would
make it easier to operate or understand.

## References

- [Mission Planner Flight Data](https://ardupilot.org/planner/docs/mission-planner-flight-data.html)
- [Mission Planner Configuration and Tuning](https://ardupilot.org/planner/docs/mission-planner-configuration-and-tuning.html)
- [QGroundControl Fly View](https://docs.qgroundcontrol.com/master/en/qgc-user-guide/fly_view/fly_view.html)
- [QGroundControl Plan View](https://docs.qgroundcontrol.com/master/en/qgc-user-guide/plan_view/plan_view.html)
- [Auterion Mission Control](https://docs.auterion.com/vehicle-operation/auterion-mission-control)
- [MAVLink parameter protocol](https://mavlink.io/en/services/parameter.html)
- [MAVLink command protocol](https://mavlink.io/en/services/command.html)
- [MAVLink mission protocol](https://mavlink.io/en/services/mission.html)
