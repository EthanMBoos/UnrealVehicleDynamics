# ArduRover boat direction

The first marine push is a powered surface vessel that ArduRover can drive and
turn in calm water. Station keeping, waves, and engineering seakeeping are
later experiments; water work is documented in [WATER.md](WATER.md).

```text
public boat model -> C++ plant -> headless maneuvers -> ArduRover experiment
                                                       |
                            DataFlash + truth + report
```

## Reference boat

Use Thor Fossen's public Otter USV model as the first reference. The Otter is a
two-meter twin-hull research vessel with two fixed propellers, making it a good
match for ArduRover's left/right throttle outputs.

The primary numerical reference is current
[`otter.m`](https://github.com/cybergalactic/MSS/blob/master/CRAFT/USV/models/otter.m)
in the MIT-licensed
[Marine Systems Simulator](https://github.com/cybergalactic/MSS). The older
[Python Otter](https://github.com/cybergalactic/PythonVehicleSimulator/blob/master/src/python_vehicle_simulator/vehicles/otter.py)
is useful for reading and secondary comparison, but its equations now differ
from MSS.

The reference includes:

- six-axis rigid-body and added-mass dynamics;
- Coriolis, damping, hydrostatic restoring, and water-current terms;
- twin-propeller placement, saturation, and response lag;
- different forward and reverse bollard-thrust curves; and
- mass, payload, center of gravity, inertia, displacement, and draft.

This is the marine equivalent of the current educational Aerosonde: a public,
inspectable controls model with executable references. It is not a validated
digital twin of every physical Otter. Some hydrodynamic terms are engineering
approximations, while the propeller model includes experimental bollard data.
Call the model `fossen_otter_educational` and state that limitation plainly.

The source model is six-axis even though the first experiments are planar.
Begin with surge, sway, and yaw maneuvers in calm water, but do not replace the
reference equations with hand-tuned maximum-speed and turn-rate gains.

## Reference strategy

Implement the equations and parameters as ordinary C++ in this repository.
MSS and Python Vehicle Simulator remain optional validation tools, not runtime
dependencies.

The first checks should compare:

1. mass, added mass, hydrostatics, and equilibrium;
2. forward and reverse propeller thrust;
3. matched-state derivatives with and without current;
4. single- and dual-propeller step responses;
5. straight acceleration and coast-down; and
6. differential-thrust turning and zigzag maneuvers.

The C++ solver does not need to copy the reference program's integration
method. Compare the continuous model at matched states, then check trajectory
convergence independently.

## Configuration

Keep physical plant data, ArduRover parameters, and the experiment separate:

```text
fossen_otter.json  Otter source scalars for mass, geometry, hydrodynamics,
                   payload, and propellers; C++ derives its matrices
rover.parm      ArduRover frame, output mapping, and controller parameters
run.json        initial state, current, environment, and maneuver
```

The physical file uses SI units, North-East-Down world axes, and
Forward-Right-Down body axes. It declares the midships body origin and the
source scalar assumptions. Derived marine matrices stay in C++.

ArduPilot configuration does not belong in the hydrodynamic model. A
differential-thrust boat normally uses `Throttle Left` and `Throttle Right`;
`FRAME_CLASS=2` enables boat behavior. Rudder, vectored-thrust, sail, and
planing craft require different model types rather than optional fields added
to the Otter file.

## Other public references

[Virtual RobotX](https://github.com/osrf/vrx) provides the well-known WAM-V,
public visual assets, configurable thruster layouts, sensors, wind, waves, and
an Apache-licensed Gazebo implementation. It is a useful independent simulator
and visual reference. Its default hydrodynamics are deliberately simplified,
so it is not the primary numerical oracle for the Otter plant.

[ShipMMG](https://github.com/ShipMMG/shipmmg) implements the standard MMG
hull-propeller-rudder method and supports canonical ship maneuvering work such
as KVLCC2 cases. It is a useful later test of the plant boundary, but a tanker
with a rudder is not representative of the first small differential-thrust
ArduRover vehicle.

The public [OtterROS](https://github.com/offroad-robotics/otter_ros) project
documents a current research workflow around the physical Otter. It is useful
context for commands and field logs, not a dynamics reference.

## Boundary with water simulation

[WATER.md](WATER.md) currently covers two other problems:

1. a visual Unreal Water and Chaos buoyancy demonstration; and
2. a later engineering seakeeping solver using six-axis hydrodynamic
   coefficients, radiation memory, wave excitation, and matching rendered and
   physical wave phase.

Those remain useful, but they should not block the powered-USV loop. The Otter
work establishes calm-water propulsion and maneuvering first. Water rendering,
wave response, maneuvering in waves, planing, and hydrodynamic contact can be
added only when an experiment needs them.

## First useful result

The first result is one bounded ArduRover ACRO maneuver using the Otter model,
with desired and achieved speed and turn rate, left/right propeller commands,
simulator truth, ArduRover estimates, and timing health saved together. Open-
loop straight and turning runs work headlessly before Unreal renders the same
committed boat state.
