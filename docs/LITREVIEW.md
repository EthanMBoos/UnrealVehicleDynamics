# Literature review

**Review window:** January 2025 through 19 August 2026

This narrative review covers fixed-wing dynamics, simulation, model
verification, flight validation, ArduPilot, PX4, Unreal, and current research
software. It is not a systematic or bibliometric review. Software capabilities
come from official documentation and repositories; vendor performance claims
are identified as such.

## Findings

### Simulator integration is established work

Dimmig et al. survey 44 aerial-robot simulators and compare 14 in depth. Their
results show that dynamics, sensors, speed, maintenance, extensibility, and
environment quality remain use-case tradeoffs
([2025](https://doi.org/10.1109/MRA.2024.3433171)).

Recent platforms already combine many of the features relevant here:

- ProVANT provides Gazebo/ROS SIL and HIL control experiments
  ([Morais et al., 2025](https://doi.org/10.3390/aerospace12090762)).
- AeroQT combines JSBSim, MAVLink, PX4, HIL, and real-flight views
  ([Aziz and Loya, 2025](https://doi.org/10.3389/arc.2025.14524)).
- `aerial-autonomy-stack` supports PX4, ArduPilot, ROS 2, containers, and
  faster-than-real-time execution
  ([Panerati et al., 2026](https://arxiv.org/abs/2602.07264)).
- ROSplane 2.0 emphasizes readable fixed-wing research software and movement
  from simulation to flight
  ([Reid et al., 2026](https://arxiv.org/abs/2510.01041)).
- A fixed-wing swarm testbed reports JSBSim dynamics, deterministic seeds,
  synchronized replay, and mission logs
  ([Bolz et al., 2026](https://controls.papercept.net/conferences/scripts/rtf/ICUAS26_ContentListWeb_3.html)).

ArduPilot and PX4 both provide first-party simulator integrations. Project
AirSim provides deterministic stepping and PX4 lockstep. PteroSim markets UE5,
fixed-wing and VTOL dynamics, and both autopilots; those are product claims,
not peer-reviewed validation.

An Unreal scene plus an autopilot connection is therefore an implementation
choice, not a research result by itself.

### Unreal is usually the scene and sensor layer

AirSim established the combination of Unreal rendering, vehicle APIs,
synthetic sensors, and autopilot connections. Colosseum moved that design to
Unreal Engine 5; Cosys-AirSim continues the ecosystem. JSBSim ships an Unreal
reference application, while AeroSim separates dynamics and rendering through
co-simulation interfaces.

Recent studies use Unreal mainly for environment, perception, and mission
work: solar-plant inspection with ROS 2 and ArduPilot
([Andrade et al., 2025](https://doi.org/10.1016/j.ifacol.2025.09.198)), urban
manned-unmanned formation with Cesium and PX4
([Lee et al., 2025](https://doi.org/10.1109/DASC66011.2025.11257249)), and
low-altitude traffic simulation
([Li et al., 2025](https://arxiv.org/abs/2512.24112)).

Unreal Robotics Lab uses MuJoCo as the physics authority and Unreal for scene
construction and rendering
([Embley-Riches et al., 2026](https://arxiv.org/abs/2504.14135)). This supports
the same division used by the Otter path in this repository: the engineering
solver commits state; Unreal renders it.

### Fixed-wing validation increasingly uses measured data

Recent fixed-wing work derives models from aerodynamic tools, identifies them
from flight data, checks modal response, and validates on withheld maneuvers:

- AVL-to-Gazebo model generation with synchronized PX4 parameters
  ([Cavalheiro and Guerreiro, 2025](https://doi.org/10.1109/YEF-ECE66503.2025.11117433));
- open- and closed-loop identification of a small flying wing
  ([Matt et al., 2025](https://doi.org/10.2514/1.C038147));
- Skywalker X8 identification using estimated aerodynamic angles
  ([Løw-Hansen et al., 2025](https://doi.org/10.1007/s13272-025-00816-3));
- nonlinear aero-propulsive identification with out-of-sample checks
  ([Figueira et al., 2025](https://doi.org/10.2514/1.C037964));
- CFD-derived six-degree-of-freedom dynamics compared with three flight
  datasets ([Jayanti et al., 2026](https://doi.org/10.3311/PPtr.41190)); and
- overset CFD and flight telemetry for a longitudinal model
  ([Siswantara et al., 2026](https://doi.org/10.1016/j.taml.2026.100702)).

FalconWing combines an identified nonlinear plant, a photorealistic world, and
a physical vision-only landing result
([Miao et al., 2025](https://arxiv.org/abs/2505.01383)). García-Gascón et al.
publish 240 fixed-wing missions with synchronized telemetry, mission data, and
parameter snapshots
([2026](https://doi.org/10.1038/s41597-026-06716-3)).

The educational Aerosonde in this repository supports equation verification
and controls examples. Physical accuracy requires measured coefficients,
wind-tunnel data, or flight data from a specific aircraft.

### Digital twin has a physical-data requirement

Current reviews distinguish a simulation model from a twin that is calibrated,
synchronized, or updated from a physical asset
([Moon, 2025](https://journal.kci.go.kr/kiots/archive/articleView?artiId=ART003259737);
[Zhang et al., 2025](https://doi.org/10.3390/drones9060394)). Valencia et al.
compare simulation with fixed-wing and multirotor flight logs
([2025](https://doi.org/10.1007/s10846-025-02276-7)); Guevara et al. combine
model, hardware, and experiment validation
([2025](https://doi.org/10.2139/ssrn.5145160)).

NASA/FAA and ABS guidance describe credibility as application-specific code
verification, solution verification, validation, and uncertainty
quantification
([NASA/FAA, 2026](https://ntrs.nasa.gov/api/citations/20260001729/downloads/NASA-TM-20260001729.pdf);
[ABS, 2024](https://ww2.eagle.org/content/dam/eagle/rules-and-guides/current/design_and_analysis/348-guidance-notes-on-verification-and-validation-of-models%2C-simulations%2C-and-digital-twins-2024/348-vandv-gn-nov24.pdf)).

The repository is a simulator and model test bench. It is not a digital twin
of the Aerosonde or Otter examples.

### Learning does not remove the need for model checks

Current work includes differentiable flight training
([Zhang et al., 2025](https://doi.org/10.1038/s42256-025-01048-0)),
physics-informed parameter estimation
([Michek et al., 2025](https://doi.org/10.2514/1.J063991)), and Gaussian-process
aerodynamic models with uncertainty
([Harp et al., 2025](https://arxiv.org/abs/2501.01000)). Differentiable
aeroelastic tools are also available
([Cea and Palacios, 2025](https://doi.org/10.1016/j.cpc.2025.109547)).

These methods still need stated frames, limits, training data, held-out cases,
and uncertainty. A learned residual can complement a readable physical model;
it does not make an unchecked simulation credible.

## Position of this repository

The repository already provides:

- readable C++ fixed-wing and marine equations;
- headless evaluation and RK4 simulation;
- fixed-wing trim and discrete linearization;
- a mirrored-equation JSBSim check and timestep convergence;
- Unreal/Cesium runs with fixed-step ArduPlane and ArduRover exchange; and
- simulator truth, controller telemetry, DataFlash, and HTML reports.

The examples are educational plants. There are no measured Aerosonde or Otter
trajectories, wind-tunnel results, sensor-error models, or uncertainty bounds.

JSBSim remains the mature general-purpose fixed-wing FDM. The smaller core here
is retained because its equations, state update, and linearization fit in a
compact reviewable code path. If a study needs JSBSim's broader aircraft,
propulsion, ground, or systems support, using JSBSim is the simpler choice.

The practical contribution today is the working engineering loop:

```text
model parameters -> C++ plant -> headless check -> ArduPilot run -> report
```

Additional autopilots, model detail, sensors, and experiment infrastructure
need a specific user case before they add value.

## Verification labels

Use the following terms for results:

```text
equation and invariant tests                 code verification
mirrored-equation reference comparison       implementation comparison
timestep convergence                         solution verification
Unreal/controller state-transfer checks      runtime integration check
wind-tunnel or held-out flight data           physical validation
parameter propagation                         uncertainty quantification
```

The JSBSim fixture implements the same educational equations through another
evaluator. Agreement catches code, frame, sign, unit, normalized-rate, and
reference-point errors. It is not independent aerodynamic validation.

## Terminology

| Term | Required qualification |
|---|---|
| high fidelity | variable, envelope, reference, uncertainty, and measured error |
| validated against JSBSim | implementation agreement or physical validation must be distinguished |
| digital twin | identify the physical asset and its calibration or synchronization data |
| deterministic | name the deterministic boundary and separate SITL scheduling variance |
| same experiment | match commands, disturbances, sensors, timing, state, parameters, and scoring |
| better controller | document tuning, mode, estimator assumptions, failures, and uncertainty |
| first | support the exact statement with a current formal search |

## Selected references and software

### Peer-reviewed and proceedings sources

- Aziz, F., and Loya, A. “Multi-Mode MAVLink Test Bench AeroQT for Aircraft Simulation and Real-Flight
  Validation.” *Aerospace Research Communications* 3, 2025.
  [doi:10.3389/arc.2025.14524](https://doi.org/10.3389/arc.2025.14524).
- Cavalheiro, F. S., and Guerreiro, B. J. “Fixed-Wing UAV Simulation in PX4 and Gazebo: An AVL-Based
  Approach.” *YEF-ECE*, 2025.
  [doi:10.1109/YEF-ECE66503.2025.11117433](https://doi.org/10.1109/YEF-ECE66503.2025.11117433).
- Cea, A., and Palacios, R. “JAX-Based Aeroelastic Simulation Engine for Differentiable Aircraft
  Dynamics.” *Computer Physics Communications*, 2025.
  [doi:10.1016/j.cpc.2025.109547](https://doi.org/10.1016/j.cpc.2025.109547).
- Dimmig, C. A., Silano, G., McGuire, K., Gabellieri, C., Hönig, W., Moore, J. L., and Kobilarov, M.
  “Survey of Simulators for Aerial Robots: An Overview and In-Depth Systematic Comparisons.”
  *IEEE Robotics & Automation Magazine* 32(2), 153–166, 2025.
  [doi:10.1109/MRA.2024.3433171](https://doi.org/10.1109/MRA.2024.3433171).
- Figueira, J. C., et al. “Nonlinear Aero-Propulsive Modeling for Fixed-Wing eVTOL UAV from Flight Test
  Data.” *Journal of Aircraft* 62(2), 300–312, 2025.
  [doi:10.2514/1.C037964](https://doi.org/10.2514/1.C037964).
- Jayanti, E. B., et al. “Validation of a Small UAV Dynamic Model Using CFD and Flight Test Data.”
  *Periodica Polytechnica Transportation Engineering* 54(1), 88–104, 2026.
  [doi:10.3311/PPtr.41190](https://doi.org/10.3311/PPtr.41190).
- García-Gascón, C., Bas-Bolufer, J., Castelló-Pedrero, P., and García-Manrique, J. A. “An Open
  Benchmark Dataset for Machine Learning and Intelligent Trajectory Optimization in Fixed-Wing
  Unmanned Aerial Systems.” *Scientific Data* 13, 364, 2026.
  [doi:10.1038/s41597-026-06716-3](https://doi.org/10.1038/s41597-026-06716-3).
- Lee, U., Kim, T., and Lee, S. “An Aviation Manned-Unmanned Teaming Simulation in Urban Environments
  to Compare Autonomous Flight Formations.” *DASC*, 2025.
  [doi:10.1109/DASC66011.2025.11257249](https://doi.org/10.1109/DASC66011.2025.11257249).
- Løw-Hansen, B., et al. “Modeling and Identification of a Small Fixed-Wing UAV Using Estimated
  Aerodynamic Angles.” *CEAS Aeronautical Journal*, 2025.
  [doi:10.1007/s13272-025-00816-3](https://doi.org/10.1007/s13272-025-00816-3).
- Matt, J. J., Chao, H., Shawon, M. H., Svoboda, B. C., and Hagerott, S. G. “System Identification for
  Small Flying-Wing Unmanned Aircraft Using Open-Loop and Closed-Loop Flight Data.” *Journal of
  Aircraft* 62(4), 961–977, 2025.
  [doi:10.2514/1.C038147](https://doi.org/10.2514/1.C038147).
- Michek, N. E., Mehta, P. M., and Huebsch, W. W. “Flight Dynamics Modeling Using Physics-Informed
  Neural Networks.” *AIAA Journal*, 2025.
  [doi:10.2514/1.J063991](https://doi.org/10.2514/1.J063991).
- Morais, J. E., et al. “ProVANT Simulator: A Virtual Unmanned Aerial Vehicle Platform for Control
  System Development.” *Aerospace* 12(9), 762, 2025.
  [doi:10.3390/aerospace12090762](https://doi.org/10.3390/aerospace12090762).
- Andrade, F. A. A., Sivertsen, A., Moura, M., et al. “Unreal Engine 5 Simulations of Solar Plant
  Inspections by Unmanned Aerial Systems with Robot Operating System 2.” *IFAC-PapersOnLine* 59(10),
  1173–1178, 2025.
  [doi:10.1016/j.ifacol.2025.09.198](https://doi.org/10.1016/j.ifacol.2025.09.198).
- Siswantara, A. I., et al. “Flight Test Validation of High-Fidelity Overset Mesh Predictions for UAV
  Longitudinal Dynamics.” *Theoretical and Applied Mechanics Letters*, early access, 2026.
  [doi:10.1016/j.taml.2026.100702](https://doi.org/10.1016/j.taml.2026.100702).
- Valencia, E., et al. “An Open-Source UAV Digital Twin Framework: A Case Study on Remote Sensing in
  the Andean Mountains.” *Journal of Intelligent & Robotic Systems* 111, 71, 2025.
  [doi:10.1007/s10846-025-02276-7](https://doi.org/10.1007/s10846-025-02276-7).
- Wang, S., Dong, Z., Li, H., Shen, L., Peng, X., and She, D. “RouthSearch: Inferring PID Parameter
  Specification for Flight Control Program by Coordinate Search.” *Proceedings of the ACM on Software
  Engineering* 2(ISSTA), 640–662, 2025.
  [doi:10.1145/3728904](https://doi.org/10.1145/3728904).
- Zhang, T., Grzelak, D., Zhao, W., Islam, M. A., Fricke, H., and Aßmann, U. “A Review on the
  Construction, Modeling, and Consistency of Digital Twins for Advanced Air Mobility Applications.”
  *Drones* 9(6), 394, 2025.
  [doi:10.3390/drones9060394](https://doi.org/10.3390/drones9060394).
- Teetaert, S., Zhao, W., Loquercio, A., et al. “Advancing Reproducibility, Benchmarks, and Education
  With Remote Sim2real: Remote Simulation to Real Robot Hardware.” *IEEE Robotics & Automation
  Magazine* 32(1), 117–123, 2025.
  [doi:10.1109/MRA.2025.3527291](https://doi.org/10.1109/MRA.2025.3527291).
- Zhang, Y., Hu, Y., Song, Y., et al. “Learning Vision-Based Agile Flight via Differentiable Physics.”
  *Nature Machine Intelligence* 7, 954–966, 2025.
  [doi:10.1038/s42256-025-01048-0](https://doi.org/10.1038/s42256-025-01048-0).

### Preprints and emerging work

- Embley-Riches, J., Liu, J., Julier, S., and Kanoulas, D. “Unreal Robotics Lab: A High-Fidelity
  Robotics Simulator with Advanced Physics and Rendering.” ICRA 2026 / 2025 preprint.
  [arXiv:2504.14135](https://arxiv.org/abs/2504.14135).
- Guevara, B. S., Moya, V., Gandolfo, D., and Toibero, J. M. “Development and Experimental Validation
  of UAV Digital Twins.” 2025 preprint.
  [doi:10.2139/ssrn.5145160](https://doi.org/10.2139/ssrn.5145160).
- Harp, D. I., Ott, J., Asmar, D. M., Alora, J., and Kochenderfer, M. J. “Physics-Informed Gaussian
  Processes for Safe Envelope Expansion.” 2025 preprint; journal version published in 2026.
  [arXiv:2501.01000](https://arxiv.org/abs/2501.01000).
- Li, Z., Tao, T., Fu, R., Wang, L., Zhang, D., and Quan, Q. “RflyUT-Sim: A Simulation Platform for
  Development and Testing of Complex Low-Altitude Traffic Control.” 2025 preprint.
  [arXiv:2512.24112](https://arxiv.org/abs/2512.24112).
- Miao, Y., Shen, W., Cui, H., and Mitra, S. “FalconWing: An Open-Source Platform for Ultra-Light
  Fixed-Wing Aircraft Research.” 2025 preprint/workshop paper.
  [arXiv:2505.01383](https://arxiv.org/abs/2505.01383).
- Panerati, J., Sajjadi, S., Soleymanpour, S., Mehta, V., and Mantegh, I. “aerial-autonomy-stack—a
  Faster-than-Real-Time, Autopilot-Agnostic, ROS2 Framework to Simulate and Deploy Perception-Based
  Drones.” ICUAS 2026 / preprint.
  [arXiv:2602.07264](https://arxiv.org/abs/2602.07264).
- Reid, I., Ritchie, J., Moore, J., Sutherland, B., Snow, G., Tokumaru, P., and McLain, T. “ROSplane
  2.0: A Fixed-Wing Autopilot for Research.” ICUAS 2026 / 2025 preprint.
  [arXiv:2510.01041](https://arxiv.org/abs/2510.01041).
- Bolz, W., Faber, F., Lork, J., Cella, M., Zendel, O., and d'Apolito, F. “An Integrated Testbed for
  Mission-Level Autonomy Evaluation in Evolving Disaster Scenarios with Fixed-Wing Swarms.” ICUAS
  2026, 696–703. Evidence is limited to the
  [official program](https://controls.papercept.net/conferences/scripts/rtf/ICUAS26_ContentListWeb_3.html).
- “FALCON-S: Fixed-Wing Aerodynamics and Learning Control Suite.” Withdrawn ICLR 2026 submission.
  [OpenReview](https://openreview.net/forum?id=BIDlQ5ifz4).

### Official software and standards

- [ArduPilot SITL overview](https://ardupilot.org/dev/docs/sitl-simulator-software-in-the-loop.html),
  [external JSON interface](https://ardupilot.org/dev/docs/sitl-with-JSON.html),
  [JSBSim integration](https://ardupilot.org/dev/docs/sitl-with-jsbsim.html), and
  [Gazebo plugin](https://github.com/ArduPilot/ardupilot_gazebo).
- [PX4 simulation overview](https://docs.px4.io/main/en/simulation/),
  [Gazebo integration](https://docs.px4.io/v1.17/en/sim_gazebo_gz/), and
  [JSBSim integration](https://docs.px4.io/v1.12/en/simulation/jsbsim/).
- [JSBSim](https://github.com/JSBSim-Team/jsbsim) and its
  [Unreal reference application](https://github.com/JSBSim-Team/jsbsim/blob/master/UnrealEngine/README.md).
- [Microsoft AirSim](https://github.com/microsoft/AirSim),
  [Colosseum](https://github.com/CodexLabsLLC/Colosseum), and
  [Cosys-AirSim](https://github.com/Cosys-Lab/Cosys-AirSim).
- [AeroSim](https://github.com/aerosim-open/aerosim) and its
  [architecture documentation](https://aerosim.readthedocs.io/en/latest/).
- [Project AirSim](https://github.com/iamaisim/ProjectAirSim) and its
  [simulation clock documentation](https://iamaisim.github.io/ProjectAirSim/development/scene/sim_clock_internal.html).
- [PteroSim](https://github.com/PteroLabsAI/PteroSim-UAV-Simulator).
- [NASA/FAA VVUQ report, 2026](https://ntrs.nasa.gov/api/citations/20260001729/downloads/NASA-TM-20260001729.pdf)
  and [ABS V&V guidance, 2024](https://ww2.eagle.org/content/dam/eagle/rules-and-guides/current/design_and_analysis/348-guidance-notes-on-verification-and-validation-of-models%2C-simulations%2C-and-digital-twins-2024/348-vandv-gn-nov24.pdf).
