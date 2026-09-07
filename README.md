# UnrealVehicleDynamics

The central goal of UnrealVehicleDynamics is to reproduce demanding reference
demonstrations from open-source autopilot projects in one instrumented Unreal
simulation. Each demonstration should be runnable, inspectable, and published
with enough plant truth, controller data, and timing evidence to verify its
correctness. The work starts with ArduPilot, then repeats equivalent cases with
PX4 so their behavior can be compared on common scenarios.

## Demonstration roadmap

The next ArduPilot target is the
[RotorS copter landing](docs/COPTER_LANDING.md): port the ETH RotorS quadrotor
plant, prove it headlessly, fly one bounded standalone ArduCopter demonstration,
then land on and take off from the simulated ArduRover-controlled Otter. The
moving-deck run adds a landing target, explicit deck contact and a report of the
two vehicles' relative motion, target health, contact, saturation and timing.

Two other demanding targets broaden the controls evidence:

- follow ArduPilot's documented
  [scripted aerobatics](https://ardupilot.org/plane/docs/common-scripted-aerobatics-4.4.html),
  including its Lua-driven maneuvers and complete schedules, and report the
  commanded and flown trajectories; and
- build an Unreal drone-racing course and run a repeatable autonomous lap that
  exposes trajectory tracking, control effort, saturation, and timing.

After the ArduPilot demonstrations work and their reports are published, the
same classes of demonstration will be implemented with PX4. Comparisons should
use the same plant, environment, initial conditions, and measurements wherever
the controller architectures permit. These are reproducible implementation and
controls benchmarks; physical-fidelity claims still require external data.

## Current baseline

UnrealVehicleDynamics currently runs C++ vehicle models headlessly or inside
Unreal Engine with Cesium and ArduPilot SITL. The repository includes three
examples:

- a BYU/MAVSim Aerosonde controlled by ArduPlane; and
- a Fossen Otter surface vessel controlled by ArduRover; and
- both vehicles synchronized in one Unreal/Cesium world.

Both models use SI units, North-East-Down world axes, and Forward-Right-Down
body axes.

```text
ArduPilot PWM
      |
      v
shared C++ model -> committed vehicle state -> ArduPilot sensors
      |                       |
      +-> headless CSV        +-> Unreal and Cesium
```

The shared core contains the vehicle equations. It has no Unreal, networking,
JSON, or process code.

The aircraft model returns body force and moment; Chaos advances its rigid
body. The Otter model has a coupled six-axis mass matrix, so its core RK4
solver advances the state and Unreal renders the committed pose. In both cases
Unreal owns the fixed simulation clock and exchanges state and PWM with
ArduPilot once per accepted controller frame.

The `uvd` CLI compiles the same model sources. It evaluates and simulates both
plants without starting Unreal. The Aerosonde also provides trim and local
linearization.

## Headless use

Install the prerequisites in [INSTALL.md](docs/INSTALL.md), then build:

```sh
cmake -S . -B build
cmake --build build
```

Evaluate or simulate either model:

```sh
build/uvd evaluate examples/run.json
build/uvd simulate examples/run.json --duration 10 --output runs/aerosonde.csv

build/uvd evaluate examples/otter-run.json
build/uvd simulate examples/otter-run.json --duration 10 --output runs/otter.csv
```

Fixed-wing operating-point analysis is available separately:

```sh
build/uvd trim examples/run.json
build/uvd linearize examples/run.json --output runs/linear-model.json
```

## Unreal demonstrations

Store the Cesium ion token in the ignored local environment file:

```sh
cp .env.example .env
# Edit .env and set CESIUM_ION_TOKEN.
```

Run a closed-loop demonstration:

```sh
./run.py          # Aerosonde + ArduPlane FBWA
./run.py otter    # Otter + ArduRover ACRO
./run.py dual     # Both vehicles, released on one shared clock
```

`run.py` builds the core and Unreal plugin with hot reload disabled, builds the
selected ArduPilot Docker image, opens the game window, and stops both
processes when the run ends. Results are written under `runs/`:

```text
truth.csv       simulator state
exchange.csv    fixed-step controller exchange
telemetry.csv   ArduPilot telemetry
ardupilot/      DataFlash and SITL files
report.html     plots and timing checks
```

The dual run uses Plane ports 9002/9003 with system ID 1 and Rover ports
9012/9013 with system ID 17. It waits until both controllers are armed, then
releases both plants at the same 120 Hz physics step. It writes the same files
under `runs/dual-*/plane/` and
`runs/dual-*/otter/`; `scenario.json` records the run contract, and the
top-level `report.html` checks coverage, missed or duplicate frames, and maximum
step and clock skew. This is a transport and synchronization baseline, not yet
a moving-deck landing scenario.

Use `1`–`4` for chase, orbit, onboard, and side cameras. Right-drag orbits,
the scroll wheel zooms, Tab selects the other vehicle, `F` reacquires the
vehicle, and Space pauses the selected model while leaving the camera active.

## Model checks

```sh
python3 validation/timestep_convergence.py
python3 validation/jsbsim_compare.py --samples 50
```

The JSBSim check is optional. It compares the fixed-wing aerodynamic equations
with the repository fixture; it does not validate a physical Aerosonde.

## Repository layout

```text
core/       fixed-wing and marine equations and integrators
cli/        headless evaluation and simulation
unreal/     Unreal project and vehicle plugin
ardupilot/  ArduPlane and ArduRover containers, sessions, and reports
validation/ numerical and reference-model checks
examples/   model parameters and run files
docs/       model, runtime, and research notes
```

The Aerosonde is an educational coefficient model used for an air-started
FBWA response run. The Otter is an educational port of the MSS calm-water
model used for an ACRO speed-and-turn run. Their equations and source limits
are documented in [AERO.md](docs/AERO.md) and
[ARDUROVER.md](docs/ARDUROVER.md).

## License

Project code and documentation use the [MIT License](LICENSE). Third-party
licenses and packaging boundaries are recorded in
[THIRD_PARTY_NOTICES.md](THIRD_PARTY_NOTICES.md).
