# ArduPlane workflow

The fixed-wing demonstration runs the repository Aerosonde model against
ArduPlane SITL:

```text
Cesium LLA -> NED/FRD state -> C++ Aerosonde model -> Chaos
                    ^                                |
                    |                                v
               ArduPlane JSON <--------------- PWM outputs
```

Launch it with:

```sh
./run.py
```

The launcher builds the core and Unreal plugin, builds the pinned ArduPlane
Docker image, opens Unreal, and starts SITL. The session waits for GPS and EKF
readiness, selects Fly By Wire A (FBWA), arms, then releases the aircraft.

FBWA stabilizes roll and pitch while throttle remains a pilot command. This
exercises the basic fixed-wing controllers without adding a mission, takeoff,
or landing sequence. See the
[ArduPlane FBWA documentation](https://ardupilot.org/plane/docs/fbwa-mode.html).

## Aircraft model

The model is ordinary C++ compiled into both `uvd` and the Unreal plugin.
[AERO.md](AERO.md) records its equations, conventions, source parameters, and
JSBSim comparison.

```text
examples/aircraft/aerosonde.json  mass, inertia, geometry, coefficients,
                                  propulsion, effectors, PWM, and visual
examples/run.json                 initial state, origin, wind, command,
                                  controller ports, and Cesium asset
```

The core uses SI units, NED world axes, and FRD body axes. Aircraft data must
state its reference dimensions, center of mass, inertia convention, thrust
line, control signs, and visual transform.

The headless checks use the same equations:

```sh
build/uvd evaluate examples/run.json
build/uvd simulate examples/run.json --duration 10
build/uvd trim examples/run.json
build/uvd linearize examples/run.json
python3 validation/timestep_convergence.py
python3 validation/jsbsim_compare.py --samples 50
```

`evaluate` and `simulate` are the normal model checks. Trim and linearization
are fixed-wing operating-point tools.

## ArduPlane boundary

ArduPlane runs in Docker; the host does not need an ArduPlane executable. The
image build and firmware revision are in `ardupilot/Dockerfile`. Output mapping
and the small parameter overlay are in `ardupilot/uvd.parm`.

The Unreal plugin sends the JSON state expected by ArduPilot SITL and receives
PWM output. The aircraft file maps PWM channels to aileron, elevator, rudder,
and throttle. Surface direction, trim, travel, and physical limits remain part
of the aircraft definition.

The controller exchange runs at the configured fixed step. Each accepted
servo frame advances one simulation interval. Repeated frames reuse the cached
state response; missing or duplicate frames are counted in `exchange.csv`.
The implementation follows ArduPilot's
[JSON simulator protocol](https://github.com/ArduPilot/ardupilot/tree/master/libraries/SITL/examples/JSON).

## Demonstration and report

The session applies a short FBWA roll sequence:

```text
neutral -> right roll command -> left roll command -> neutral
```

The run directory contains simulator truth, controller exchange timing,
ArduPlane telemetry, SIMSTATE samples, DataFlash, and a self-contained Plotly
report. The report checks:

- one simulation step per accepted controller frame;
- zero missed or duplicate frames;
- the configured exchange rate and timestep; and
- exact transfer of the committed attitude through ArduPilot's JSON parser.

Plant truth and the ArduPlane attitude estimate are plotted separately. Their
difference is an estimator measurement, not a state-transfer error.

## Using another aircraft

The current example is a reference implementation, not a general aircraft
loader. A conventional derivative model can reuse the Aerosonde equation form
with new parameters. A different set of equations belongs in new C++ model and
integrator files, with direct CLI and Unreal branches like the existing
Aerosonde and Otter paths.

Start from the engineer's available data: coefficient tables, geometry tools,
CFD, wind-tunnel results, propulsion tests, an existing model, or flight logs.
Check frames, units, mass, inertia, control signs, propulsion, limits, and
timestep behavior before connecting ArduPlane.

The bundled launcher currently uses the repository Docker image, parameter
file, FBWA sequence, and report. Support for arbitrary images, a host GCS,
parameter overlays, scenario files, and automated run comparison can be added
when a user needs one of those workflows.

## Current limits

- air start only;
- no terrain contact, takeoff, or landing;
- ideal state and IMU inputs without sensor errors or latency;
- instantaneous control surfaces;
- one bundled ArduPlane build and parameter set; and
- no physical validation against a real Aerosonde.

## References

- [ArduPilot simulation overview](https://ardupilot.org/dev/docs/simulation-2.html)
- [ArduPilot JSON simulator protocol](https://github.com/ArduPilot/ardupilot/tree/master/libraries/SITL/examples/JSON)
- [Using SITL and connecting a GCS](https://ardupilot.org/dev/docs/using-sitl-for-ardupilot-testing.html)
- [ArduPilot servo functions](https://ardupilot.org/plane/docs/servo-functions.html)
- [ArduPilot DataFlash logs](https://ardupilot.org/plane/docs/common-logs.html)
- [ArduPlane release notes](https://raw.githubusercontent.com/ArduPilot/ardupilot/master/ArduPlane/ReleaseNotes.txt)
