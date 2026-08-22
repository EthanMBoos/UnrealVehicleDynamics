# Fossen Otter implementation plan

The first marine plant is the public Fossen Otter model: a two-meter,
twin-propeller surface vessel that maps naturally to ArduRover's left and
right throttle outputs. Implement and validate it headlessly before adding
Unreal or ArduRover integration.

```text
MSS Otter equations -> C++ plant -> headless evaluation and simulation
                                      |
                                      +-> Unreal committed pose
                                      +-> ArduRover JSON loop
```

## First: isolate the working aircraft path

Before porting the Otter, make one behavior-preserving refactor:

- move the shared rigid-body state, wrench, derivative and quaternion API into
  `rigid_body.hpp`;
- move geodesy and fixed-wing declarations into `geodesy.hpp` and
  `fixed_wing.hpp`, while keeping `core.hpp` as a compatibility umbrella;
- rename the current integrator, CLI run type, simulation and analysis files
  so they are explicitly Aerosonde code; and
- leave the commands, run-file fields, numerical output, Unreal aircraft
  component and ArduPlane lockstep path unchanged.

Do not extract a generic integrator or plant interface. The existing rigid-body
derivative assumes a scalar mass, a three-by-three inertia and explicit
gravity; it is shared infrastructure for the aircraft, not the Otter's coupled
six-axis solver. Likewise, do not add `plant.type` until the CLI has an actual
Otter branch. The current run file is parsed separately by the CLI and Unreal,
so changing it early would disturb the proven path without testing a real
model swap.

Verify this refactor with unchanged headless evaluation, simulation, trim and
linearization results, the timestep-convergence check, and a fresh
`-NoHotReloadFromIDE` Unreal/ArduPlane lockstep run. Only then begin the Otter
port below.

## Reference authority

Use the current
[`otter.m`](https://github.com/cybergalactic/MSS/blob/master/CRAFT/USV/models/otter.m)
in the MIT-licensed
[Marine Systems Simulator](https://github.com/cybergalactic/MSS) as the
numerical reference.

The
[Python Otter](https://github.com/cybergalactic/PythonVehicleSimulator/blob/master/src/python_vehicle_simulator/vehicles/otter.py)
remains useful for reading and independent comparison, but it has fallen
behind MSS:

- MSS includes the 2026 payload-inertia correction.
- MSS uses the Soeding surge added-mass estimate; Python uses `0.1` times the
  hull mass.
- MSS cross-flow drag uses `1025 kg/m^3` and includes heave and pitch; Python
  uses `1026 kg/m^3` and only sway and yaw.
- MSS shifts the hydrostatic equilibrium for payload; Python adds the payload
  wrench directly.
- Current MSS simulations use RK4; Python uses forward Euler.

Do not expect both programs to produce the same derivative or trajectory.
Match current MSS and use Python as a secondary implementation reference.
Record the upstream source revision as provenance, not as a runtime dependency
or a version gate.

## State and frames

The reference state is:

```text
eta = [north, east, down, roll, pitch, yaw]
nu  = [u, v, w, p, q, r]                  body FRD
n   = [left shaft speed, right shaft speed]
```

This maps onto the existing core state without a frame conversion:

```text
position_ned_m       <- north, east, down
q_body_to_ned        <- roll, pitch, yaw as a quaternion
velocity_body_mps    <- u, v, w
omega_body_radps     <- p, q, r
```

The Otter adds two actuator states:

```cpp
struct OtterState {
  RigidBodyState body;
  Vector2 propeller_speed_radps;
};
```

The body origin is the Otter coordinate origin: midships on the centerline.
It is not the combined center of gravity. The visual mesh, hydrostatic
reference and reported state must all use this origin consistently.

Use the core's SI, NED and FRD conventions. Keep attitude internally as a
Hamilton body-to-NED quaternion. Extract ZYX roll, pitch and yaw only where the
reference hydrostatic and current equations require them. Quaternion
kinematics replace the reference Euler-angle integration without changing the
physical state.

## Equations to carry over

The continuous model is:

```text
nu_c = horizontal current expressed in body axes
nu_r = nu - nu_c

M = M_RB + M_A
C = C_RB(nu) + C_A(nu_r)

nu_dot = nu_c_dot
        + solve(M,
              tau_propeller
            + tau_damping
            + tau_crossflow
            - C * nu_r
            - G * (eta - eta_0))
```

### Rigid body and payload

Carry over:

- the `55 kg` hull mass;
- the configurable payload mass and position;
- the hull center of gravity and radii of gyration;
- the corrected parallel-axis construction of the combined inertia;
- the combined center of gravity; and
- the `Hmtrx` transformation from the combined CG to the midships origin.

Compute the combined inertia using the current MSS construction:

```text
r_g       = (m_hull r_hull + m_payload r_payload) / m_total
r_hull/g  = r_hull - r_g
r_pay/g   = r_payload - r_g

I_g = I_hull
    - m_hull    S(r_hull/g)^2
    - m_payload S(r_pay/g)^2
```

Build the rigid-body mass matrix at the combined CG, then transform it to the
midships origin. Preserve the reference's angular-velocity-only rigid-body
Coriolis representation rather than replacing it with an algebraically
different form during the first port.

### Added mass

Use the current MSS values:

```text
-X_u_dot = addedMassSurge(hull mass, length, water density)
-Y_v_dot = 1.5 * hull mass
-Z_w_dot = 1.0 * hull mass
-K_p_dot = 0.2 * I_xx
-M_q_dot = 0.8 * I_yy
-N_r_dot = 1.7 * I_zz
```

The surge term uses the Soeding approximation:

```text
nabla_hull = hull mass / water density
-X_u_dot   = 2.7 * water density * nabla_hull^(5/3) / length^2
```

This deliberately uses hull mass, not total loaded mass, because that is what
the reference does. Form the added-mass Coriolis matrix from `M_A` and relative
velocity using the reference `m2c` construction.

### Current

The Otter reference supports a uniform horizontal current. With current speed
`V_c`, direction `beta_c` clockwise from north and vehicle yaw `psi`:

```text
u_c = V_c cos(beta_c - psi)
v_c = V_c sin(beta_c - psi)
nu_c = [u_c, v_c, 0, 0, 0, 0]
```

The current acceleration in the rotating body frame is:

```text
nu_c_dot = [-S(omega) * [u_c, v_c, 0], 0, 0, 0]
```

Preserve this reference behavior. Do not silently generalize it to a vertical
current or a full roll/pitch projection. Configuration may express the current
as a NED vector, but the loader must reject a nonzero down component and reduce
the horizontal vector to `V_c` and `beta_c`.

### Hydrostatics

Carry over the pontoon geometry, displaced volume, draft, transverse and
longitudinal metacentric heights, center of flotation, and transformed
hydrostatic stiffness matrix `G`.

This is a linear, small-motion hydrostatic model. It is not an instantaneous
hull-volume or free-surface calculation. The base hull weight and buoyancy are
already balanced by the model and must not be applied again.

The payload force is expressed in the body frame from NED gravity. MSS solves
an equilibrium offset in heave, roll and pitch:

```text
eta_0[heave, roll, pitch]
    = inverse(G[heave, roll, pitch])
      * g_payload[heave, roll, pitch]
```

Then it evaluates restoring force from `eta - eta_0`. Preserve this sequence
even though a later nonlinear hydrostatic model may treat payload weight and
buoyancy differently.

The flat-water environment supplies `water_surface_down_m`. Hydrostatic heave
uses the model origin's down displacement relative to that surface, not
altitude above the Cesium origin or an arbitrary global zero.

### Damping and cross-flow drag

Preserve the reference linear damping construction:

- surge damping derived from maximum forward speed;
- sway and yaw damping derived from one-second time constants;
- heave, roll and pitch damping derived from natural frequencies and damping
  ratios; and
- nonlinear yaw damping `N_r (1 + 10 |r|) r`.

Port the current MSS
[`crossFlowDrag`](https://github.com/cybergalactic/MSS/blob/master/LIBRARY/modeling/crossFlowDrag.m)
behavior exactly for the first implementation:

- Hoerner coefficient interpolation using the committed lookup table;
- pontoon beam and loaded draft as the section dimensions;
- `1025 kg/m^3` water density;
- sway/yaw and heave/pitch cross-flow loads; and
- the reference's 21 inclusive strip locations from `-L/2` to `L/2`.

Do not change the quadrature to midpoints or trapezoidal weighting during the
port. That may be a reasonable later improvement, but it would prevent exact
matched-state comparison.

### Propellers and actuator state

Each propeller produces forward body force:

```text
T = k_forward n |n|    when n > 0
T = k_reverse n |n|    when n <= 0
```

The left propeller is at negative body `y`; the right propeller is at positive
body `y`. Therefore greater left thrust produces positive body yaw moment,
which is a right turn in NED/FRD. This agrees with ArduRover skid-steering
output semantics and should not require a reversal workaround.

Actual shaft speed is a plant state. Commanded shaft speed is not passed
directly to the hull equations:

```text
n_dot = (n_command - n_actual) / 0.1 s
```

Clamp command and actual shaft speed to the asymmetric physical limits. Keep
shaft speed in `rad/s`; the source sometimes calls it propeller revolutions,
but it is not revolutions per second.

The plant accepts left and right commands directly. Do not port the reference
heading PID, path following, step-input generator, or control allocation.
ArduRover will own control and already produces left and right motor outputs.

## Reference values

For the reference `55 kg` hull and `25 kg` payload at
`[0.05, 0, -0.35] m`, construction must produce:

```text
total mass             80 kg
combined CG            [0.153125, 0, -0.246875] m
draft                   0.1951219512 m
surge added mass        5.2815210096 kg
propeller minimum      -101.7366655 rad/s  (-971.51 RPM)
propeller maximum       103.9308643 rad/s  ( 992.47 RPM)
equilibrium down         0.03754241 m
equilibrium pitch       -0.02511045 rad     (-1.439 deg)
```

Corrected inertia at the combined CG:

```text
[10.651039   0        -0.386719]
[ 0         14.523438  0       ]
[-0.386719   0         14.136719] kg m^2
```

Effective mass matrix at the midships origin:

```text
[ 85.281521    0          0          0        -19.75       0      ]
[  0         162.5        0         19.75       0         12.25   ]
[  0           0        135          0        -12.25       0      ]
[  0          19.75       0         17.657028   0          2.6375 ]
[-19.75        0        -12.25       0         32.89375    0      ]
[  0          12.25       0          2.6375     0         40.044922]
```

The off-diagonal terms are physical parts of this reference plant. A Chaos
rigid body cannot represent this matrix with one translational mass and one
3x3 rotational inertia tensor.

## Integration method

The numerical integrator is not a physical parameter of the Otter. The model
defines a continuous state derivative. The two public references already use
different integration policies:

- Python advances velocity and propeller state with forward Euler, then
  advances position and Euler attitude with the updated velocity.
- Current MSS simulations advance the twelve vehicle states with
  [RK4](https://github.com/cybergalactic/MSS/blob/master/LIBRARY/numericalMethods/rk4.m)
  while updating propeller speed separately with Euler.

Integrate the body and two propeller states together with the core's RK4
approach. Hold a controller command constant over one physics interval,
evaluate the complete derivative at each stage, normalize the quaternion at
intermediate stages and after the step, and clamp shaft speed after the step.

Use a fixed-size solve for:

```text
M * nu_dot_relative = generalized_load
```

Do not reproduce Python's explicit inverse or its finite-step Euler behavior.
A C++ RK4 trajectory will not match the Python samples exactly at a fixed
timestep. Validation instead requires:

1. matched-state derivatives against MSS;
2. a like-for-like RK4 trajectory with fixed propeller speed; and
3. independent timestep convergence of the augmented C++ state.

This separates equation errors from integration error. For a smooth case that
does not cross a saturation or absolute-value discontinuity, RK4 error should
decrease near fourth order. Around zero cross-flow or saturation boundaries,
require monotonic convergence rather than a misleading exact order.

## C++ API and files

Add concrete Otter code without introducing a general plant framework:

```text
core/include/uvd/marine.hpp
core/src/fossen_otter.cpp
core/src/marine_integrator.cpp

examples/boats/fossen_otter.json
examples/otter-run.json

tests/fossen_otter_test.cpp
```

The public core API should remain explicit:

```cpp
FossenOtterModel make_fossen_otter_model(
    const FossenOtterParameters& parameters);

OtterModelOutput evaluate_fossen_otter(
    const RigidBodyState& body,
    const OtterActuatorState& actuators,
    const OtterEnvironment& environment,
    const FossenOtterModel& model);

OtterState step_fossen_otter_rk4(
    const OtterState& state,
    const OtterCommand& command,
    const OtterEnvironment& environment,
    const FossenOtterModel& model,
    double dt_s);
```

`FossenOtterModel` contains derived, run-constant quantities such as the
combined CG and inertia, `M_RB`, `M_A`, `M`, hydrostatic stiffness, damping
coefficients, loaded draft and Hoerner coefficient. Validate those once at
configuration load. Runtime evaluation should use only fixed-size Eigen
objects and perform no I/O or allocation.

`OtterModelOutput` should expose enough named terms to audit the equations:

```text
current velocity in body axes
relative velocity
left and right thrust
propeller wrench
linear damping wrench
cross-flow wrench
Coriolis/centripetal wrench
hydrostatic restoring wrench
total generalized load
body linear and angular acceleration
propeller-speed derivative
validity
```

## Configuration

Use one explicit load-time plant selection rather than inferring a model from
its fields:

```json
"plant": {
  "type": "fossen_otter_educational",
  "parameters": "boats/fossen_otter.json"
}
```

Convert the current Aerosonde run to the same small seam when the CLI starts
supporting both plants. A tagged enum or `std::variant` in the CLI is enough;
do not add factories, dependency injection, a configuration framework, or a plugin framework.

The Otter file contains the source scalar assumptions:

- water density and reference gravity;
- hull mass, CG and radii of gyration;
- payload mass and position;
- hull, pontoon and waterline geometry;
- added-mass ratios and surge model;
- damping time constants, ratios and nonlinear yaw factor;
- center of flotation;
- propeller positions, forward/reverse thrust coefficients, shaft-speed
  limits and response time; and
- source name, URL, revision and license.

Derive matrices in C++ rather than storing anonymous arrays. Another marine
plant with measured matrices can later define its own C++ parameter type; it
should not accumulate optional fields in the Otter configuration.

The initial flat-water environment is deliberately narrow:

```json
"water": {
  "surface_down_m": 0.0,
  "current_ned_mps": [0.0, 0.0, 0.0]
}
```

Plant configuration, ArduRover parameters and experiment commands remain
separate files.

## Headless validation

The first implementation stops at a headless, open-loop Otter. Add ordinary
C++ tests registered with CTest; do not require Python, NumPy, MATLAB or Octave
to build or run the project.

### Construction checks

- Match the reference mass, CG, draft, inertia, added mass, mass matrix,
  equilibrium and propeller limits above.
- Require `M_RB`, `M_A`, `M` and `G` to be symmetric within floating-point
  tolerance.
- Require `M` to be positive definite.
- Reject invalid mass, geometry, draft, metacentric height, time constants and
  shaft-speed ranges at configuration load.

### Equation checks

- `S(a)b` equals `a cross b` and `S(a)` is skew symmetric.
- `H(r) inverse` agrees with `H(-r)`.
- `m2c(M, nu)` is skew symmetric after reference symmetrization.
- Zero cross-flow velocity produces zero cross-flow load.
- Cross-flow force and moment oppose sway, heave, yaw and pitch motion.
- Equal positive propeller speeds produce positive surge and zero yaw moment.
- Greater left thrust produces positive yaw moment.
- Reverse shaft speed produces negative surge using the weaker reverse
  coefficient.
- Shaft-speed saturation affects both thrust and the committed actuator state.

### Reference derivative checks

Generate and record high-precision MSS results for a small set of named states:

1. loaded equilibrium with zero current and zero propeller speed;
2. equal forward propeller speed;
3. equal reverse propeller speed;
4. asymmetric propulsion with nonzero six-axis state;
5. nonzero current with roll, pitch and yaw rates; and
6. commands beyond both shaft-speed limits.

Compare velocity, angular-rate, position and Euler-attitude derivatives. For
the quaternion C++ state, convert its kinematics to ZYX Euler rates only in the
test diagnostic. Expected data records the MSS source revision that generated
it. Agreement with these fixtures checks the port; it is not independent
physical validation.

### Integration checks

- A zero-command equilibrium remains stationary.
- A propeller step reaches approximately `63.2%` after one `0.1 s` time
  constant.
- A fixed-propeller trajectory agrees with MSS RK4.
- Halving the C++ timestep reduces smooth-trajectory error consistently.
- A sixty-second equal-thrust run remains straight, finite and bounded.
- A sixty-second differential-thrust run turns in the expected direction and
  remains finite and bounded.

The first CLI demonstration needs only `evaluate` and `simulate`. Boat trim,
linearization, command schedules and a general experiment format wait until an
actual controls study uses them.

## Unreal ownership

After headless validation, the Otter plant owns motion inside the Unreal
plugin. Unreal renders the committed pose; it does not solve the boat dynamics.

For the boat branch:

- keep the hidden body kinematic;
- disable Chaos gravity, damping, buoyancy and force application;
- advance the Otter state exactly once on the existing fixed physics clock;
- publish position, rotation, linear velocity and angular velocity through the
  physics-thread body handle;
- attach the non-colliding visual mesh to that body; and
- retain one pose writer so camera tracking and render interpolation cannot
  compete with game-thread transform updates.

Do not pass the Otter generalized load to the existing aircraft
`AddForce`/`AddTorque` path. Chaos cannot reproduce the effective mass matrix,
and enabling Unreal buoyancy would duplicate the Otter hydrostatic restoring
model.

The first water surface is a visual flat-water reference at
`water_surface_down_m`. Waves, queried Unreal Water, nonlinear hull buoyancy,
seakeeping and collision impulses remain separate work in
[WATER.md](WATER.md). A later model may replace the Otter hydrostatic term; it
must not stack a second buoyancy system on top of it.

The existing `UUvdAircraftComponent` and its runtime are hard-coded to
Aerosonde parameters, atmosphere, four aircraft controls, air-data output and
Chaos-owned motion. Rename or generalize that host only after the headless
Otter passes. Use one explicit plant switch rather than a broad runtime plugin
system.

## ArduRover mapping

ArduPilot documents the differential-thrust configuration as:

```text
FRAME_CLASS      2    Boat
SERVO1_FUNCTION 73    Throttle Left
SERVO3_FUNCTION 74    Throttle Right
```

See the ArduPilot
[boat configuration](https://ardupilot.org/rover/docs/boat-configuration.html)
and
[motor and servo configuration](https://ardupilot.org/rover/docs/rover-motor-and-servo-configuration.html).

Both channels are centered, bidirectional motor commands:

```text
1000 PWM -> full reverse
1500 PWM -> stopped
2000 PWM -> full forward
```

Map positive and negative normalized commands to their respective asymmetric
shaft-speed limits. The existing centered `map_pwm` behavior can support this;
left and right propellers must not use the aircraft's unidirectional throttle
mapping.

The existing
[ArduPilot JSON protocol](https://github.com/ArduPilot/ardupilot/blob/master/libraries/SITL/examples/JSON/readme.md)
already carries the required vehicle-independent state:

- NED position and velocity;
- body-to-NED attitude;
- FRD angular rate; and
- FRD IMU specific force.

Reuse the proven frame-counter, duplicate-frame and cached-reply lockstep
behavior. For the boat:

- omit the optional aircraft `airspeed` field instead of labeling
  water-relative speed as airspeed;
- derive specific force from consecutive committed states;
- initially place the simulated IMU at the Otter coordinate origin; and
- do not send a water-current field to ArduPilot, which should observe its
  effect through motion and sensors.

Use a near-sea-level demo origin. ArduPilot explicitly warns that its boat
sea-level GPS filtering is not intended for elevated lakes, so the existing
Utah aircraft origin is a poor default boat case.

The current Dockerfile and session build and start ArduPlane only. After the
C++ plant and Unreal pose path work, add one separate ArduRover image/session
for the demonstrated rig. Do not build arbitrary image loading, controller
adapters or a general supervisor as part of the Otter port.

## Explicit exclusions

The first Otter implementation does not include:

- Unreal Water or Chaos buoyancy;
- waves, radiation memory, diffraction or seakeeping;
- planing or speed-dependent hull attitude;
- wind loads;
- propeller advance-ratio or torque effects;
- rudders, vectored thrust, sails or arbitrary thruster layouts;
- contacts or docks;
- an Otter heading controller;
- an external MSS/Python runtime dependency;
- a general marine plant interface;
- arbitrary ArduPilot container loading; or
- boat trim and linearization commands.

The reference itself uses a calm-water, small-motion hydrostatic model,
semi-empirical damping and bollard-thrust propellers independent of vessel
advance speed. The implementation should state those limits rather than imply
physical validation of a real Otter.

## Implementation sequence

1. Isolate the shared rigid-body declarations and explicitly name the
   Aerosonde integrator and CLI path as described above. Verify that the
   existing headless and Unreal behavior is unchanged.
2. Update [ARDUROVER.md](ARDUROVER.md) to make current MSS the primary
   equation source and add the MSS MIT notice to `THIRD_PARTY_NOTICES.md`.
3. Add fixed-size marine aliases, concrete Otter parameters, derived model,
   state, command, environment, output and derivative types.
4. Port model construction and verify the golden mass, hydrostatic and
   propeller values.
5. Port current, Coriolis, propeller, damping, cross-flow and restoring terms
   in the same staged order as the source equations.
6. Add augmented quaternion RK4 integration for the body and propeller states.
7. Add the explicit plant selector and Otter JSON loading to the CLI.
8. Add headless `evaluate` and constant-command `simulate` support.
9. Add construction, equation, matched-derivative and convergence tests.
10. Demonstrate equal-thrust and differential-thrust runs, then review the
   generated trajectories before expanding scope.
11. Only after those checks pass, add the kinematic Unreal pose path.
12. Only after the pose path works, add the ArduRover container and bounded
    closed-loop experiment.

This sequence proves the physical model and the repository's first real plant
swap without accumulating unused water, controller, container or framework
code.
