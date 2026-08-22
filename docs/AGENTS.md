# Repository instructions

## Model conventions

- Read `README.md`, `docs/AERO.md`, and `docs/CPP_STYLE.md` before changing the vehicle model.
- Keep the shared core in SI units with NED world axes and FRD body axes.
- Preserve conventional controls notation, explicit frame/unit suffixes, and the staged equation flow
  documented in `docs/CPP_STYLE.md`.
- Keep I/O, JSON, Unreal types, sockets, and process control out of `core/`.
- Use fixed-size Eigen objects in per-tick model code.

## Directories

- `core/`: vehicle equations and headless integrators.
- `cli/`: the `uvd` command-line interface and JSON loading.
- `unreal/`: Cesium, Chaos, ArduPilot transport, and rendering.
- `validation/`: reference-model and numerical checks.
- `ardupilot/`: ArduPlane and ArduRover containers, sessions, and reports.

Use direct parsing, ordinary JSON, and focused scripts. Extract shared machinery after two working
paths need the same code.

## Checks after C++ changes

Format the touched project files, configure CMake, build, and exercise the affected command. The
shared headless operations are:

```sh
cmake -S . -B build
cmake --build build
build/uvd evaluate examples/run.json
build/uvd simulate examples/run.json --duration 1
build/uvd evaluate examples/otter-run.json
build/uvd simulate examples/otter-run.json --duration 1
```

For changes to the fixed-wing operating-point analysis, also run:

```sh
build/uvd trim examples/run.json
build/uvd linearize examples/run.json
```

For changes to integration or the aircraft equations, also run:

```sh
python3 validation/timestep_convergence.py
python3 validation/jsbsim_compare.py --samples 50
```

The JSBSim check is optional when its Python package is unavailable. For Unreal runtime changes,
build and launch the affected `run.py` profile and inspect the game log and window.
