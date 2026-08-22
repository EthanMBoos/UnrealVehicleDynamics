# UnrealVehicleDynamics

UnrealVehicleDynamics runs C++ vehicle models headlessly or inside Unreal
Engine with Cesium and ArduPilot SITL. The repository includes two examples:

- a BYU/MAVSim Aerosonde controlled by ArduPlane; and
- a Fossen Otter surface vessel controlled by ArduRover.

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

Run either closed-loop demonstration:

```sh
./run.py          # Aerosonde + ArduPlane FBWA
./run.py otter    # Otter + ArduRover ACRO
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

Use `1`–`4` for chase, orbit, onboard, and side cameras. Right-drag orbits,
the scroll wheel zooms, `F` reacquires the vehicle, and Space pauses the model
while leaving the camera active.

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
