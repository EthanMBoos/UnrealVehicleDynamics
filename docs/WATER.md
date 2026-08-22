# Water and seakeeping reference

The current Otter demonstration uses a flat water reference and a calm-water
maneuvering model. It has no waves, buoyancy queries, or contact. Two different
extensions cover those effects:

1. Unreal Water, pontoon buoyancy, and Chaos for a visual/game-physics boat.
2. A project-owned 6x6 solver for engineering seakeeping calculations.

They answer different questions and use separate state owners.

## Unreal water and buoyancy

The stock Unreal path uses one displacement boat in a gentle regular deep-water
wave. Unreal Water supplies the rendered surface and CPU height query, pontoon
buoyancy supplies forces, and one Chaos body advances all six motion axes and
handles contact.

For wave height `H` and period `T`, the initial regular-wave setup is:

```text
amplitude  = H / 2
wavelength = 9.8 T^2 / (2 pi)
phase      = 0
```

```text
wave settings
  -> Unreal Water Waves Asset
       -> rendered water
       -> CPU height queries
            -> buoyancy pontoons
                 -> Chaos body
```

Use the boat's mass, center of mass, and inertia. Pontoon volume and placement
set flat-water draft and upright balance. Empirical damping requires a saved
decay test; foam, reflections, and normal-map ripples remain visual settings.

The first checks are mechanical:

- surface markers remain on the rendered wave through crest and trough;
- height, period, and direction changes affect render and query together;
- pause, reset, and replay reproduce the same sampled height;
- flat-water displacement and draft match the configured hull;
- heave and roll decay are repeatable; and
- a low-speed dock collision remains stable.

Unreal's asynchronous buoyancy can reuse one rendered wave time across several
physics steps. Compare at several render and physics rates. If the samples
separate from the render, evaluate the asset's wave list once per physics step
and feed those heights to the pontoons.

The stock components are documented in Unreal's
[Water Waves Asset](https://dev.epicgames.com/documentation/en-us/unreal-engine/simulating-waves-using-the-water-waves-asset-in-unreal-engine?application_version=5.6)
and
[Buoyancy Component](https://dev.epicgames.com/documentation/en-us/unreal-engine/water-buoyancy-component-in-unreal-engine?application_version=5.6)
guides. [asv_wave_sim](https://github.com/srmainwaring/asv_wave_sim/tree/ca8629df4e191235753dfae92ef725d30b923364)
shows wave queries and hull clipping.

Pontoon buoyancy does not calculate added mass, radiation, diffraction, or a
measured hull response. Those terms require a separate solver.

## Linear seakeeping model

The initial engineering case is one stationary displacement hull, zero current,
zero forward speed, small motion, and a first-order regular wave. Propulsion,
contact, planing, and nonlinear hydrostatics are excluded.

Let `xi = [x,y,z,roll,pitch,yaw]` use fixed equilibrium-frame axes:

```text
(M_RB + A_inf) xi_ddot
  + integral_0^t K_r(t-tau) xi_dot(tau) dtau
  + K_hs xi
  = tau_exc(t) + tau_external(t)
```

`A_inf` is infinite-frequency added mass, `K_r` is the radiation-memory
kernel, `K_hs` is hydrostatic stiffness, and `tau_exc` is wave excitation. The
base model has no nonlinear Coriolis or viscous term.

A first validation case uses one frequency and heading present in the
hydrodynamic data. The time-domain motion amplitude and phase are compared with
the corresponding frequency-domain solution:

```text
[-omega^2 (M_RB + A(omega)) + i omega B(omega) + K_hs] xi_hat
  = F_exc(omega,beta) a_hat
```

This case exposes frame, unit, sign, coupling, integration, and sample-time
errors before irregular waves are introduced.

## State ownership

Chaos supports one translational mass and one rotational inertia about principal
axes. It cannot apply a general marine 6x6 mass matrix with different surge,
sway, and heave mass or translation/rotation coupling.

The seakeeping solver therefore advances the double-precision 6x6 state. Unreal
renders its committed pose through a kinematic actor. Contact is disabled
because a Chaos impulse would use a different inverse mass. Dynamic contact
would require the marine inverse mass inside the contact solve.

## Hydrodynamic data

The runtime convention is SI with NED position and FRD body axes. Generalized
forces use `[X,Y,Z,K,M,N]` for surge, sway, heave, roll, pitch, and yaw.

Capytaine, WAMIT, and WEC-Sim commonly use X forward, Y port, Z up. Convert that
data once during import:

```text
Q = diag(+1,-1,-1, +1,-1,-1)

nu_core   = Q nu_bem
tau_core  = Q tau_bem
A_core    = Q A_bem Q^T
B_core    = Q B_bem Q^T
K_core    = Q K_bem Q^T
Fexc_core = Q Fexc_bem
```

The imported artifact records:

- source mesh, scale, origin, axes, and solver version;
- displaced volume, waterline, density, gravity, and depth;
- frequencies, headings, and phasor convention;
- `A(omega)`, `B(omega)`, `A_inf`, `K_hs`, and excitation force; and
- mesh and coefficient checks.

Check mesh refinement, displacement, hydrostatics, matrix symmetry,
nonnegative radiation damping, positive `M_RB + A_inf`, free modes, frequency
coverage, impulse-response decay, and irregular-frequency spikes. A change of
reference point requires the full matrix transform.

[Capytaine](https://capytaine.org/) can generate a fixed coefficient set.
[WEC-Sim](https://wec-sim.github.io/WEC-Sim/main/theory/theory.html) and
[HydroChrono](https://github.com/Project-SEA-Stack/HydroChrono/tree/0648a2f880ae4237760b745b2f9ff4696559b76b)
provide independent formulations and comparison cases.

## Radiation memory

Using the WEC-Sim Cummins convention:

```text
K_r(t) = 2/pi integral_0^inf B(omega) cos(omega t) domega
F_rad  = -A_inf xi_ddot - integral K_r(t-tau) xi_dot(tau) dtau
```

For `exp(i omega t)`:

```text
Khat_r(i omega) = B(omega) + i omega [A(omega) - A_inf]
```

A direct finite-history convolution is the reference implementation. A fitted
state-space form can replace it after comparison:

```text
z_dot      = A_r z + B_r xi_dot
tau_memory = -(C_r z + D_r xi_dot)
```

Record frequency range, windowing, extrapolation, fit error, and state order.
The fitted poles must be stable. Frequency-grid and time-history energy checks
support, but do not prove, passivity. Checkpoints include the radiation state;
zero state represents a vessel that was motionless before the run.

## Wave representation

Physics, logs, and rendering consume the same realized components:

```cpp
struct WaveComponent {
    double omega_radps;
    double wavenumber_radpm;
    double direction_ned_rad;
    double amplitude_m;
    double phase_rad;
};
```

```text
beta:       propagation direction clockwise from North
eta_up:     positive upward
psi_j:      omega_j (t-t_epoch) - k_j d_j dot (x_NE-origin_NE) + phase_j
eta_up:     sum_j amplitude_j cos(psi_j)
dispersion: omega_j^2 = g k_j tanh(k_j depth)
```

Amplitude is half regular-wave height. For a spectrum:

```text
a_j = sqrt(2 integral_bin S_omega d omega)
Hm0 = 4 sqrt(sum_j a_j^2/2)
```

Store the sorted component list, epoch, and origin. A random seed alone is not
enough to reproduce the surface. Capytaine uses `exp(-i omega t)`; conjugate its
complex excitation once when importing into this document's `exp(+i omega t)`
convention.

Stock Unreal Water does not expose arbitrary component phase through every
render and query path. A seakeeping renderer must consume the solver's saved
components and committed timestamp, then compare GPU height with the core wave
function through run, pause, seek, and origin rebasing.

## Numerical checks

Use small waves with speed, current, viscous drag, propulsion, contact, and
rendering disabled. Ramp the input, remove the startup transient, and fit the
complex response over several periods.

Initial tolerances are 2% complex response error for direct convolution against
the frequency-domain solution, 2% for the fitted model against convolution,
and 5% through the final core-to-Unreal pose and log samples. Use absolute
tolerances near a zero response. Cover head, beam, and oblique waves and add a
chirp or multisine so steady RAOs do not hide transient errors.

Physical validation requires measured or published decay and complex RAO data.
Roll commonly needs a separate viscous damping fit.

## Maneuvering boundary

The current Otter model covers calm-water maneuvering. Linear seakeeping uses a
different force set. Combining them requires one definition for shared added
mass, damping, encounter frequency, and mean drift; simply summing both models
would count some loads twice.

Planing, forward-speed seakeeping, nonlinear hydrostatics, mooring, towing, and
engineering contact remain outside these notes.
