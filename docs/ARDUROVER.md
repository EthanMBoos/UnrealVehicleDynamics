# ArduRover and the Fossen Otter

The marine demonstration runs ArduRover against a C++ port of Thor Fossen's
Otter USV model:

```text
ArduRover PWM -> left/right shaft commands -> C++ Otter solver
                                                |
                                                +-> Unreal pose
                                                +-> controller state
```

Launch the closed-loop run with:

```sh
./run.py otter
```

For an open-loop Unreal run using the shaft commands in the run file:

```sh
./run.py otter-open-loop
```

## Model source

The numerical reference is
[`otter.m`](https://github.com/cybergalactic/MSS/blob/d7c1e969856f4e98bc2f6ffa466d0abd6ec5d950/CRAFT/USV/models/otter.m)
from the MIT-licensed
[Marine Systems Simulator](https://github.com/cybergalactic/MSS). The model
file records the source commit and SHA-256.

The older
[Python Otter](https://github.com/cybergalactic/PythonVehicleSimulator/blob/master/src/python_vehicle_simulator/vehicles/otter.py)
uses different added-mass, cross-flow, payload, and integration choices. It is
a readable secondary reference; the C++ results are matched to MSS.

The port includes:

- six-axis rigid-body and added-mass dynamics;
- Coriolis, damping, cross-flow, hydrostatic, and current loads;
- separate forward and reverse bollard-thrust curves; and
- left/right propeller saturation and first-order shaft response.

The reference describes a two-meter research vessel in calm water. It uses
small-motion hydrostatics, semi-empirical damping, and bollard thrust that does
not vary with advance speed. These limits are part of the model.

## Files and units

```text
examples/boats/fossen_otter.json  hull, payload, hydrodynamic, and propeller data
examples/otter-run.json           state, origin, current, shaft command, and ports
ardupilot/rover.parm              frame, motor mapping, and reference gains
```

The runtime uses SI units, NED position, and FRD body velocity. The body origin
is midships on the centerline. The state also carries actual left and right
shaft speed in radians per second.

The JSON file stores source scalars. The C++ model derives rigid-body mass,
added mass, hydrostatic stiffness, damping, equilibrium, and shaft limits. The
generalized axis order is surge, sway, heave, roll, pitch, yaw.

Headless inspection uses the same model:

```sh
build/uvd evaluate examples/otter-run.json
build/uvd simulate examples/otter-run.json --duration 10 --output runs/otter.csv
ctest --test-dir build --output-on-failure
```

## Unreal state ownership

The Otter's coupled 6x6 effective mass cannot be represented by one Chaos
rigid body. The core RK4 solver advances body and shaft state. Unreal publishes
that committed state through a kinematic actor; it does not add buoyancy,
gravity, collision impulses, or another pose update.

The current visual is a generated twin-hull shape over a flat water reference.
It shows position and attitude from the plant. Waves, contacts, and seakeeping
are separate work described in [WATER.md](WATER.md).

## ArduRover demonstration

`FRAME_CLASS=2` selects boat behavior. Servo functions 73 and 74 map the mixed
outputs to left and right motors. ACRO converts the steering stick to a yaw-rate
target and the throttle stick to a speed target; see the
[ArduRover ACRO documentation](https://ardupilot.org/rover/docs/acro-mode.html).

The session runs:

```text
settle -> speed step -> turn-rate step -> recover -> HOLD
```

The report plots desired and achieved speed, desired and achieved yaw rate,
plant truth, motor PWM, saturation time, and horizontal trajectory. Its run
checks cover finite plant state, all maneuver phases, controller telemetry,
and fixed-step exchange. Tracking error and saturation are reported as values;
the reference gains are not presented as a finished tune.

## Current limits

- calm water and horizontal current only;
- no waves, contacts, planing, wind, rudder, or arbitrary thruster layout;
- one payload and one reference gain set;
- one hard-coded ACRO maneuver; and
- no physical validation against a measured Otter trajectory.

Related public models include
[VRX/WAM-V](https://github.com/osrf/vrx) for marine robotics simulation and
[ShipMMG](https://github.com/ShipMMG/shipmmg) for standard hull-propeller-rudder
maneuvering models. Their vehicle and force assumptions differ from this
differential-thrust Otter.
