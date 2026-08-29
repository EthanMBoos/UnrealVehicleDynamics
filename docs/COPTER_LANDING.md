# RotorS copter landing

The next vehicle is an **ETH Zurich RotorS quadrotor** controlled by
**ArduCopter**. Its job is to take off from and land on the existing small
Otter boat.

```text
BYU/MAVSim Aerosonde -> ArduPlane
MSS Fossen Otter     -> ArduRover
ETH RotorS quadrotor -> ArduCopter
```

Use ETH RotorS as the only source for the quadrotor plant and its parameters.
Do not blend it with a second dynamics model or a hand-built textbook plant.

Do not modify the Aerosonde, turn it into a QuadPlane, or add a fixed-wing
landing leg. The Aerosonde and Otter remain unchanged examples. The quadrotor
is a third vehicle.

## What we are using

[RotorS](https://github.com/ethz-asl/rotors_simulator) is the public
multirotor simulator developed by ETH Zurich's Autonomous Systems Lab. It is
Apache-2.0 C++, has a citable Springer reference chapter, and provides the
actual rotor model, motor response and vehicle definitions used by its Gazebo
examples.

Copy the small dynamics kernel. Do not bring in ROS or Gazebo. RotorS applies
forces and moments to a Gazebo rigid body; UVD will apply those forces and
moments to a Chaos rigid body. This fits the repository cleanly.

The useful pieces are:

- `rotors_gazebo_plugins/src/gazebo_motor_model.cpp`;
- `rotors_gazebo_plugins/include/rotors_gazebo_plugins/common.h`, specifically
  `FirstOrderFilter`; and
- the small four-rotor vehicle definition supplied by RotorS for mass, inertia,
  rotor geometry and motor coefficients.

Call the UVD vehicle and demo `rotors_quadrotor`. Keep the ETH and Apache-2.0
attribution with the port.

## Why RotorS

RotorS gives us the same important qualities that made the Aerosonde and Otter
good starting points:

- ETH Zurich's Autonomous Systems Lab published the implementation and a 2016
  reference chapter;
- the C++ equations and complete parameter set are public and inspectable;
- Apache-2.0 allows a close port with the license and notices preserved;
- motor-speed commands enter below the controller, leaving ArduCopter separate
  from the plant; and
- the output is a body force and moment, which maps cleanly to Chaos.

RotorS also gives us motor lag, rotor drag, rolling moment and reaction torque.
This is more useful than another ideal thrust-only classroom model.

What we can honestly say is:

> This is a port of ETH RotorS's published educational multirotor
> implementation, using one RotorS small-quad parameter preset.

What we cannot say is:

> This is a physically validated digital twin of a real aircraft.

RotorS includes a flight-data comparison for one framework vehicle, but not
this exact four-rotor setup. Any stronger physical-fidelity claim still needs
external data.

## What we compare against

For the Aerosonde, compare the UVD fixed-wing plant against JSBSim.

For the quadrotor, compare the UVD port directly against the ETH RotorS motor
and rotor kernel. Give both implementations the same parameters, motor
commands, vehicle state, wind and timestep. Compare the resulting motor speeds
and rotor forces and moments sample by sample.

Once that matches, send the UVD wrench into Chaos. Chaos only advances the
rigid body; it is not the RotorS reference. Check the Chaos side separately for
force and moment signs, timestep convergence and runtime behavior.

```text
Aerosonde:  UVD fixed-wing equations  <-> JSBSim reference evaluator
Quadrotor:  UVD multirotor port       <-> ETH RotorS motor/rotor kernel
Unreal:     verified body wrench      ->  Chaos rigid-body integration
```

## Starting configuration

Start with the complete small four-rotor, plus-layout configuration supplied by
RotorS. Its roughly 0.54 m rotor envelope fits on the Otter's roughly 1.70 m by
0.79 m landing area, leaving about 0.125 m on each side when centered. That is
enough room for the demo. It only establishes that the geometry fits.

## What to port

Keep RotorS's update order for each of the four rotors:

1. Clamp demanded rotor speed to the configured range.
2. Calculate thrust from the actual rotor speed present at the start of the
   update.
3. Calculate local hub velocity and its wind-relative component perpendicular
   to the rotor axis.
4. Apply thrust and translational rotor drag at the rotor location so both
   forces contribute the correct arm moment.
5. Apply yaw reaction torque using thrust, the moment constant and rotor
   direction.
6. Apply RotorS's wind-relative rolling moment.
7. Sum the rotor wrenches and let the external rigid-body engine advance the
   vehicle.
8. Advance actual rotor speed for the next update using the asymmetric
   first-order rise or fall time constant and RotorS's exact discrete
   zero-order-hold expression.

The RotorS parameter file supplies:

| Quantity | Value |
|---|---:|
| body mass | 0.680 kg |
| mass per rotor link | 0.009 kg |
| arm radius | 0.170 m |
| rotor radius | 0.100 m |
| body inertia | 0.007 / 0.007 / 0.012 kg m² |
| thrust coefficient | 8.54858e-6 N/(rad/s)² |
| reaction-moment ratio | 0.016 m |
| motor rise time constant | 0.0125 s |
| motor fall time constant | 0.025 s |
| maximum rotor speed | 838 rad/s |
| layout | plus |

With the four rotor-link masses included, equal rotor speeds balance nominal
weight at about 453 rad/s. Use that as a consistency check.

Do not add terms or average parameters from other public models. Once we do
that, we no longer have a clean reference implementation.

## How it fits this repository

Add it as a normal third vehicle:

```text
core/include/uvd/multirotor.hpp
core/src/rotors_quadrotor.cpp
core/src/rotors_quadrotor_integrator.cpp
examples/copters/rotors_quadrotor.json
examples/rotors-quadrotor-run.json
tests/rotors_quadrotor_test.cpp
```

Wire it directly into the CLI, JSON loader, evaluator, simulator, Unreal
runtime, truth output, launcher and controller session. There is no need for a
universal plant interface yet.

Keep ownership simple:

```text
ArduCopter motor PWM
        |
        v
explicit PWM-to-demanded-speed boundary
        |
        v
ETH RotorS-derived four-motor model
        |
        v
per-rotor force and moment -> Chaos rigid body
        |
        v
pose, velocity and sensors -> ArduCopter
```

The core owns the four actual motor-speed states and evaluates rotor wrenches.
Chaos owns pose, linear velocity, angular velocity and contact. A headless
integrator owns the rigid-body state only for CLI verification.

Keep PWM out of the RotorS equations. The runtime boundary maps the four PWM
channels to demanded rotor speeds and reports arming, idle and saturation.
ArduCopter remains responsible for mixing roll, pitch, yaw and collective for
the plus layout.

Preserve the ETH RotorS attribution, Apache-2.0 headers and required notices in
the repository's third-party record.

## Prove the plant first

Before connecting ArduCopter, the headless plant should pass:

- a harness that runs the same samples through the RotorS kernel and UVD port;
- exact asymmetric motor-step response;
- equal-command hover equilibrium;
- zero roll, pitch and yaw moment from equal rotor speeds;
- expected force and moment signs for each rotor;
- expected yaw sign for clockwise and counter-clockwise rotor pairs;
- exact sum of individual rotor wrenches;
- motor limit and saturation checks;
- drag opposing wind-relative transverse motion;
- finite state and quaternion normalization checks;
- invalid mass, inertia, geometry and coefficient rejection; and
- timestep convergence in the headless integrator.

Exercise equal and unequal motor inputs, collective and axis steps, nonzero
body rates, still air, wind and saturation.

## First flight

Leave the boat out until one bounded flight works:

```text
ground idle
-> vertical takeoff
-> hover
-> small roll, pitch, yaw and horizontal commands
-> recover
-> descend
-> land
-> disarm
```

Save altitude and vertical-speed targets and feedback; attitude and yaw-rate
targets and feedback; plant truth; all four PWM outputs; demanded and actual
rotor speeds; per-rotor thrust; total wrench; saturation; estimator output;
flight phases; and exchange timing. Tracking error and saturation are things to
inspect, not tune acceptance limits.

## How much work this is

The main work is a full third-vehicle addition: plant, CLI, Unreal, launcher
and controller integration. Once the standalone Copter is verified, the
moving-deck landing becomes a separate final milestone.

It is a real project, but it is contained. It does not need QuadPlane,
fixed-wing aerodynamics, transition control, a runway or changes to the
Aerosonde.

## Bottom line

The search covered educational equation libraries, named research airframes,
nanodrones, autopilot test plants, robotics simulators and larger textbook
vehicles. The survey of 44 aerial-robot simulators was a useful final check.
There is no single quadrotor model with quite the same canonical status as the
Aerosonde teaching model or Fossen's Otter.

RotorS is still the cleanest choice here. It has a published source, public
C++ equations, a complete four-rotor parameter set, direct motor commands and
a force-and-moment boundary that fits Chaos. Use ETH RotorS, port only the
plant, and do not mix in a second quadrotor model.

## Primary sources

- [ETH RotorS source](https://github.com/ethz-asl/rotors_simulator)
- [ETH RotorS publication metadata and DOI](https://www.research-collection.ethz.ch/items/45a5d038-c0a0-4d74-99ba-9b0892bf498d)
- [RotorS motor model](https://github.com/ethz-asl/rotors_simulator/blob/master/rotors_gazebo_plugins/src/gazebo_motor_model.cpp)
- [ArduCopter Precision Landing and Precision Loiter](https://ardupilot.org/copter/docs/precision-landing-and-loiter.html)
- [ArduPilot QuadPlane moving-platform landing reference](https://ardupilot.org/plane/docs/common-ship-landing.html)
- [Survey of 44 aerial-robot simulators](https://arxiv.org/abs/2311.02296)
- [Repository implementation policy](../AGENTS.md)
- [Existing third-party boundaries](../THIRD_PARTY_NOTICES.md)

## Implementation notes

Keep this as a third concrete plant and work through it in four pieces.

### Work packages

1. **Port the plant.** Bring over only the RotorS motor response, rotor loads
   and selected four-rotor parameters.
2. **Prove it headlessly.** Add a quadrotor run type and CLI `evaluate` and
   `simulate` paths. Leave Aerosonde trim and linearization alone.
3. **Put it in Unreal.** Load the same parameters, map controller outputs to
   rotor-speed demands, apply the wrench to Chaos and report all four motors.
4. **Fly with ArduCopter.** Finish and report the standalone flight before
   involving the boat.

Work out the rotor order, rotation directions, frame conversion, controller
mapping and signs from the RotorS and ArduCopter sources. The tests should
prove those choices rather than relying on this note for the mapping.

### Constraints

- Keep the core in SI units with NED world axes and FRD body axes.
- Keep RotorS as the sole quadrotor plant source and retain its ETH and
  Apache-2.0 attribution.
- Keep ArduCopter's mixer separate from the plant; the core accepts physical
  rotor-speed demands, not roll, pitch, yaw or collective commands.
- Keep the Aerosonde, Otter and existing demonstrations working unchanged.

### Required evidence

Connect ArduCopter after:

- the sample-by-sample RotorS kernel comparison passes for motor response and
  rotor forces and moments;
- unit tests establish rotor signs, equal-speed balance, limits, wind-relative
  drag, finite state and timestep convergence; and
- the CLI completes representative `evaluate` and `simulate` runs.

The standalone Copter report should show controller targets and feedback, all
four PWM outputs, demanded and actual rotor speeds, plant truth, saturation,
arming and landed state, and exchange timing.

That proves the source port and controls integration. It does not physically
validate the aircraft or boat.

## After that works: land on the moving Otter

Do not start the moving-deck work until the RotorS comparison, headless runs
and standalone ArduCopter flight all pass. Then run the existing Otter ACRO
demo and the Copter on the same clock.

Use perfect simulator truth for the first attempt. Send the deck-relative
target through ArduCopter Precision Landing and `LANDING_TARGET`, match the
boat's velocity, descend onto explicit deck collision, disarm on contact and
carry the aircraft with the boat. Then rearm and take off from the moving deck.

The landing addition needs only four things:

- deck and aircraft collision with a clear touchdown state;
- deck-relative target position, velocity, timestamp and validity;
- a target-loss abort instead of a blind descent; and
- a report showing both vehicles, relative motion, target health, contact,
  arming, rotor saturation and timing.

The Plane ship-landing page is a behavioral reference for target health,
velocity matching and aborts. This remains an ArduCopter quadrotor landing on a
small boat, not a QuadPlane or runway landing.

### Add water after the first landing

The first landing uses the current calm-water Otter. It checks the two
controller sessions, moving-target guidance, touchdown, disarm and re-takeoff.

Then follow [WATER.md](WATER.md). Use Unreal Water, pontoons and Chaos for waves
with physical deck contact. Use the project-owned 6x6 solver for engineering
seakeeping. That solver keeps the boat kinematic in Unreal, so normal Chaos
contact will not use the marine dynamics. It needs an explicit touchdown and
carried state or a marine contact solution.
