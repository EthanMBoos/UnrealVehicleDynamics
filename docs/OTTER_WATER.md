# Otter visual water experiment

Date: October 25, 2026. Local Windows UE 5.8, Development editor build.

## Run and inspect

```powershell
python run.py otter-open-loop
# Optional decorative Gerstner waves:
python run.py otter-open-loop --waves
# With Docker Desktop's Linux engine running:
python run.py otter
python run.py otter --waves
```

Both Otter profiles now load `/Game/Tests/OtterWater`. The open-loop profile
uses the existing `examples/otter-run.json`: 120 Hz solver, calm water,
75 rad/s on both propellers. It writes truth and exchange CSV under `runs/`.
This is a visual/movement check, not the ArduRover ACRO tracking demonstration.

The boat is spawned at runtime, so it is absent from the saved level's editor
Outliner until play. Its actor name is `UvdVehicle` for a single run and
`UvdOtter` for the dual run. The root is `VehiclePhysicsBody` (a box), with
`VehicleDynamics` (`UUvdVehicleComponent`) and generic twin-hull visual parts:
`LeftPontoon`, `RightPontoon`, `OtterDeck`, and `OtterMast`.

## Position ownership

`UnrealVehicleDynamicsModule.cpp::SpawnVehicle` creates the actor and components.
`UvdVehicleComponent.cpp::SetInitialState` maps NED metres through the Cesium
georeference and sets the initial Unreal location and rotation.
`AsyncPhysicsTickComponent` calls `uvd::step_fossen_otter_rk4` from
`core/src/fossen_otter.cpp` on each accepted fixed step. This shared C++ solver
owns the boat's position, orientation, velocities, and propeller states.
It runs inside Unreal's process; a separate external simulator is unnecessary.
ArduRover, when used, supplies PWM commands and receives simulated sensors.

`CommitKinematicState` converts the committed state into Unreal centimetres and
coordinates, sets the Chaos particle to Kinematic, and writes X/R/V/W.
`ConfigureBody` enables `SetUpdateKinematicFromSimulation(true)` so Chaos
publishes the pose to the root and attached visuals. Gravity is disabled and
all collision responses ignored. The initial `SetSimulatePhysics(true)` in
the generic spawn routine does **not** mean Chaos integrates the Otter.
The ocean does not feed forces, wave heights, or buoyancy back into this solver.

## Water setup

Epic's Water plugin was already installed under
`Engine/Plugins/Experimental/Water/Water.uplugin`, version 0.1, experimental,
disabled by default. It is now explicitly enabled in the project. EasyWater
was not installed in the project or engine Marketplace directory; the user
chose to proceed with Unreal Water, so no third-party water package was added.

Open `/Game/Tests/OtterWater` in the editor to inspect `OtterVisualOcean`
(Water Body Ocean), `OtterWaterZone`, movable sun/sky lighting, atmosphere,
and fog. The zone and ocean extents are 200,000 cm (2 km), actor scale 1.
Water is at Unreal Z=0, matching this run's `water.surface_down_m=0` and
Cesium origin datum. This is a local planar test, not a globe-wide sea model.
The ocean spline's island opening is moved to the southwest corner, away
from the approximately 46 m northward test route. Landscape carving is off;
there is no Unreal Landscape in this test level.

The ocean explicitly uses `/Water/Materials/WaterSurface/Water_Material_Ocean`.
Waves Source is empty for a calm geometric surface; material surface detail
may still animate. The generic vessel is 2 m long with 0.25 m wide pontoons
at +/-0.395 m lateral offsets. These primitives illustrate pose and scale;
they are not a validated hull or draft mesh.

The optional `--waves` launcher flag passes `-UvdVisualWaves` to Unreal.
At game-world initialization the vehicle module assigns an inline Gerstner
source to the ocean: eight waves, seed 17, wavelengths 2-12 m, individual
amplitudes 5-15 cm, wind direction 30 degrees, and angular spread 45 degrees.
Amplitudes combine, so 15 cm is not a bound on total surface displacement.
The saved level stays calm. These settings are explicit in
`UnrealVehicleDynamicsModule.cpp`; no editor regeneration is needed.
The boat does not follow these decorative waves and may visually intersect
crests. Wave-driven vessel response requires a separate marine-model experiment.

To add water manually, enable **Edit > Plugins > Water**, restart, then use
**Place Actors**, search **Water Body Ocean**, and place it in the level.
Inspect its Water Zone and extents. For waves, use **Content Browser > Add >
Water > Water Waves**, then assign the asset in the ocean's Waves Source /
Water Waves Asset details. Enable Show Engine Content and Show Plugin Content
to browse Epic's Water materials and example wave assets.

The reproducible editor script creates or updates only this test level:

```powershell
& 'C:/Program Files/Epic Games/UE_5.8/Engine/Binaries/Win64/UnrealEditor.exe' `
  "$PWD/unreal/UnrealVehicleDynamics.uproject" `
  -EnablePlugins=PythonScriptPlugin `
  "-ExecutePythonScript=$PWD/unreal/create_otter_water_level.py" -unattended
```

## Experimental wave physics

The `--wave-physics` flag enables an optional sinusoidal disturbance
in the Fossen Otter dynamics model. This feature is disabled by default.

At each fixed physics step, the simulation adjusts the water-surface
elevation using:

    wave_height_m = 0.10 * sin(2 * pi * time_s / 3.0)

The wave has a 0.10 m amplitude and a 3.0 s period (approximately
0.333 Hz). The height is converted into the model's NED coordinate
convention and passed to both `evaluate_fossen_otter` and
`step_fossen_otter_rk4`.

The disturbance is spatially uniform. It is independent of Unreal's
visual Gerstner waves and does not represent spatial wave gradients,
wave-induced rotational moments, or a validated capsizing model.

### Four-configuration validation (November 3, 2026)

Four open-loop simulations were run:

| Configuration | Vertical position range |
| --- | --- |
| Calm physics | 0.03613 to 0.04251 m |
| Calm physics + visual waves | 0.03613 to 0.04251 m |
| Wave physics | -0.07175 to 0.14892 m |
| Wave physics + visual waves | -0.07175 to 0.14892 m |

Across the overlapping recorded samples, enabling visual waves
produced zero numerical difference in `down_m`, `pitch_deg`, and
`roll_deg` for both the calm and wave-physics configurations.

Recordings had different lengths, so this comparison applies only
to their overlapping samples.

A separate frequency analysis of a wave-enabled run showed a
dominant frequency of approximately 0.319 Hz, consistent with the
expected 0.333 Hz forcing frequency given the analysis resolution.

These results support independent control of visual waves and the
experimental physics disturbance. They do not establish physical
accuracy against real-world vessel measurements.


## Results and blockers

The initial launcher edit accidentally put the ocean map argument in the dual
command instead of the single Otter command. The subsequent game log showed
`LoadMap(/Engine/Maps/Templates/OpenWorld)`, so that run did not test the revised
ocean. The argument is now in the single Otter command; its generated command
was checked with process launch mocked. Subsequent open-loop simulations successfully launched with both visual Gerstner waves and optional sinusoidal wave physics. The four-configuration validation on November 3, 2026, confirmed that visual-wave rendering does not alter the recorded Fossen dynamics over the overlapping samples.

The baseline launched successfully using `-NoHotReloadFromIDE`; both module
manifests named current unnumbered DLLs. Its saved truth trace covers 0 to
30 s and ends at north=46.229269797 m, east=0, down=0.042508375 m,
forward body speed=1.607258234 m/s, pitch=-1.730192944 degrees.
Baseline artifacts: `runs/otter-open-loop-20261025-151152/`.

The first ocean render exposed an invisible surface: the native ocean actor
needed an explicit water material. That was corrected in the generation
script. Movable lighting also replaces the initial stationary light that
requested a lighting rebuild.

Docker's Linux engine was unavailable (`dockerDesktopLinuxEngine` named pipe
missing), so the closed-loop ArduRover ACRO run remains unverified. The editor
also logged an existing Cesium metadata enum initialization error during
startup; level creation and saving nevertheless completed. Water mesh creation
reported near-zero tangent/binormal messages. These are recorded separately
from evidence of rendered water and vessel movement.

## References

- [Epic Water System](https://dev.epicgames.com/documentation/unreal-engine/water-system-in-unreal-engine)
- [Water Body actors](https://dev.epicgames.com/documentation/unreal-engine/water-body-actors-in-unreal-engine)
- [Water Waves assets](https://dev.epicgames.com/documentation/unreal-engine/simulating-waves-using-the-water-waves-asset-in-unreal-engine)
