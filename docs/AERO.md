# Fixed-wing model

The fixed-wing example is the BYU/MAVSim educational Aerosonde. The same C++
model runs in the headless CLI and the Unreal/ArduPlane demonstration.

## Source and limits

Parameters come from BYU MAVSim's pinned
[`aerosonde_parameters.py`](https://github.com/byu-magicc/mavsim_public/blob/a8489bbc30ab9e746dccac8ac614ab2f8acac6bb/mavsim_python/parameters/aerosonde_parameters.py)
at commit `a8489bbc30ab9e746dccac8ac614ab2f8acac6bb`. They are stored in
`examples/aircraft/aerosonde.json` with their provenance.

The parameter set describes an 11 kg aircraft near 25 m/s. Published summaries
of the physical Aerosonde describe a heavier and faster vehicle. This
repository has no flight data for the educational parameter set; its physical
accuracy is unknown.

The JSBSim comparison uses this range:

| Quantity | Range |
|---|---|
| true airspeed | 18–40 m/s |
| angle of attack | -10–12° |
| sideslip | ±10° |
| normalized body rates | ±0.2 |
| control surfaces | ±15° |
| altitude | 0–3000 m |

Outside the table the model is checked only for finite output and aerodynamic
loads that fall to zero with airspeed. It has no measured stall, ground-effect,
landing-gear, takeoff, or landing model.

The FRD inertia matrix uses the source's positive `Jxz` with negative
off-diagonal entries:

```text
I_body = [ Jx    0  -Jxz ]
         [  0   Jy    0   ]
         [-Jxz   0   Jz   ]
```

## Air data

The model evaluates air data at every call, including each RK4 stage:

```text
v_air_body = velocity_body - R_body_to_ned^T wind_ned
V          = norm(v_air_body)
alpha      = atan2(w, u)
beta       = atan2(v, sqrt(u^2 + w^2))
qbar       = 0.5 rho V^2

p_hat = p b / (2 V)
q_hat = q c / (2 V)
r_hat = r b / (2 V)
```

At `V=0`, aerodynamic force and moment are zero. Below `1e-12 m/s`, normalized
rates are set to zero to avoid division by airspeed.

There are no `alpha_dot` terms because the source data does not contain them.
Using previous-step acceleration would add an undocumented delay and timestep
dependence.

The implementation follows the equation stages directly:

```text
state + atmosphere
  -> air data
  -> nondimensional coefficients
  -> dimensional body wrench
```

`fixed_wing.cpp` contains atmosphere, aerodynamics, propulsion, and the final
aircraft wrench. `rigid_body.cpp` contains conventional rigid-body mechanics.
`aerosonde_integrator.cpp` contains the aircraft RK4 step.

## Aerodynamics

Angular derivatives and control-surface derivatives are per radian.

The high-angle lift blend is:

```text
sigma = [1 + exp(-M(alpha-alpha0)) + exp(M(alpha+alpha0))]
        / [(1 + exp(-M(alpha-alpha0))) (1 + exp(M(alpha+alpha0)))]

CL_base = (1-sigma)(CL0 + CL_alpha alpha)
          + sigma [2 sign(alpha) sin(alpha)^2 cos(alpha)]
```

The code uses an equivalent stable logistic form so the exponentials cannot
overflow. Source values are `alpha0 = 0.47 rad` and `M = 50`. This only guards
lift at high angle; drag remains tied to the linear lift expression and there
is no hysteresis or separated-flow state.

```text
CL = CL_base(alpha) + CL_q q_hat + CL_de de
CD = CD_p + (CL0 + CL_alpha alpha)^2 / (pi e AR)
     + CD_q q_hat + CD_de de

CY = CY0 + CY_beta beta + CY_p p_hat + CY_r r_hat
     + CY_da da + CY_dr dr

Cl = Cl0 + Cl_beta beta + Cl_p p_hat + Cl_r r_hat
     + Cl_da da + Cl_dr dr
Cm = Cm0 + Cm_alpha alpha + Cm_q q_hat + Cm_de de
Cn = Cn0 + Cn_beta beta + Cn_p p_hat + Cn_r r_hat
     + Cn_da da + Cn_dr dr
```

`CD_p` is the runtime drag baseline. The source's `CD0` and `CDalpha` belong to
its linear design model and are not added again. The small cross terms include
`Cl_dr = 0.0024` and `Cn_da = -0.011`.

The source uses the signed term `CD_de de`. Replacing it with
`CD_de abs(de)` defines a different model and requires a matching reference
fixture.

Lift and drag rotate into FRD body axes through angle of attack:

```text
Fx = qbar S (-CD cos(alpha) + CL sin(alpha))
Fy = qbar S CY
Fz = qbar S (-CD sin(alpha) - CL cos(alpha))

[L, M, N] = qbar S [b Cl, c Cm, b Cn]
```

This wrench is about the center of mass and excludes gravity and propulsion.

## Controls and propulsion

The PWM map produces normalized aircraft commands. The actuator map converts
them to surface radians and throttle:

```cpp
struct AircraftCommand {
    double aileron;   // -1..1
    double elevator;  // -1..1
    double rudder;    // -1..1
    double throttle;  //  0..1
};
```

Surface direction, neutral, and limits come from the aircraft JSON. Surfaces
move without lag. Positive aileron produces positive roll moment; positive
elevator produces negative pitch moment near trim; rudder follows the listed
side-force and yaw coefficients.

The electric motor uses the BYU propeller fit:

```text
C_T(J) = CT2 J^2 + CT1 J + CT0
C_Q(J) = CQ2 J^2 + CQ1 J + CQ0
J      = 2 pi V_a / (omega D)

a = rho D^5 CQ0 / (4 pi^2)
b = rho D^4 CQ1 V_a / (2 pi) + K_Q^2 / R_motor
c = rho D^3 CQ2 V_a^2 - K_Q V_in / R_motor + K_Q i_0

omega = (-b + sqrt(b^2 - 4ac)) / (2a)
T     = rho n^2 D^4 C_T(J)
Q     = rho n^2 D^5 C_Q(J),       n = omega / (2 pi)
```

Thrust acts along body `+X`; reaction torque acts about `X`. An offset
propeller also contributes `r_prop x T`. The model omits battery state, spool
time, propwash, gyroscopic effects, variable pitch, and multiple engines.

Atmosphere uses ISA temperature, pressure, and density with constant NED wind.
Gravity is applied by the rigid-body integrator, outside the aircraft wrench.

## Trim

`uvd trim` solves straight, level flight at the initial airspeed in still ISA
air. The repository run starts near 25 m/s. Propeller reaction torque makes it
a full six-degree-of-freedom solve:

```text
fixed:       yaw=roll=p=q=r=0
unknowns:    u, v, w, pitch, aileron, elevator, rudder, throttle
residuals:   velocity_dot_body(3), omega_dot_body(3), TAS target, down velocity
bounds:      alpha [-10,12]°, beta ±10°, surfaces ±15°, throttle [0,1]
```

The bounded damped Gauss-Newton solve reports state, effectors, normalized
command, iterations, and all eight residuals. Convergence requires the largest
residual to be no greater than `1e-5` in its listed SI unit.

## Linearization

`uvd linearize` applies central differences to the deterministic RK4 map at the
solved trim point. Its local state is:

```text
[position_ned(3), velocity_body(3), attitude_error_body(3), omega_body(3)]
```

Attitude perturbations use `q = qbar Exp(delta_theta)`. Inputs are normalized
aileron, elevator, rudder, and throttle. Output includes labeled discrete
`A_d/B_d` matrices, the operating point, the timestep, and `xbar_1`. The latter
records the nominal next state because a trimmed aircraft still moves through
NED.

## Reference checks

`validation/jsbsim_compare.py` loads a checked-in aero-only JSBSim fixture with
the same equations. It has no engine, gear, actuator dynamics, or output
system. The script compares air data, coefficients, and dimensional body
forces and moments using the
[`FGFDMExec`](https://jsbsim-team.github.io/jsbsim/python/FGFDMExec.html) API.

Coverage includes range boundaries, both signs of every rate and control basis,
then fixed-seed mixed samples. The default is 50 samples. Starting tolerances
are:

```text
coefficient: |dC| <= 1e-10 + 1e-9 |C_ref|
force:       |dF| <= 1e-5 N + 1e-8 qbar S
moment:      |dM| <= 1e-6 N-m + 1e-8 qbar S max(b,c)
```

The fixture uses FRD body axes and an aerodynamic reference point at the center
of mass. JSBSim construction coordinates are X aft, Y right, Z up, so fixture
point coordinates include that conversion.

`validation/timestep_convergence.py` runs the RK4 trajectory at `h`, `h/2`,
and `h/4` and checks that successive state differences shrink.

```sh
python3 validation/timestep_convergence.py
python3 validation/jsbsim_compare.py --samples 50
```

The JSBSim fixture checks the C++ implementation against a second evaluator of
the same equations. It does not establish physical Aerosonde accuracy.
