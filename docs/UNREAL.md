# Unreal runtime

Unreal supplies the game clock, Cesium world, vehicle actor, cameras, and
ArduPilot transport. The model sources compile directly into the plugin; no
live CLI process sits between Unreal and the vehicle equations.

```sh
./run.py                    # Aerosonde + ArduPlane
./run.py otter              # Otter + ArduRover
./run.py otter-open-loop    # Otter without ArduRover
```

The launcher builds `UnrealVehicleDynamicsEditor` with
`-NoHotReloadFromIDE`, opens the full game window, and stops the remaining
process when either Unreal or the controller exits.

## Cesium coordinates

The run file provides latitude, longitude, MSL altitude, geoid undulation,
heading, and a local NED initial state. Cesium height is ellipsoidal; ArduPilot
home altitude is MSL:

```text
h_ellipsoid = H_MSL + geoid_undulation
```

At the configured origin, Cesium's Unreal axes are East, South, Up. The plugin
converts them to North, East, Down and converts Unreal centimeters to SI before
calling model or controller code.

`CESIUM_ION_TOKEN` is read from the ignored repository-root `.env` file. A
token loads the configured ion tileset. Without one, Cesium uses its ellipsoid
and the coordinate conversion still runs.

## Vehicle state

The two examples use different state owners because their equations differ.

For the Aerosonde, Chaos owns pose and velocity. Each fixed interval the
plugin:

1. reads the Chaos state;
2. converts it to NED/FRD;
3. evaluates atmosphere, aerodynamics, and propulsion;
4. applies one body force and moment; and
5. sends the committed state to ArduPlane.

For the Otter, the core owns body and shaft state. Each accepted controller
frame advances the marine RK4 solver once, then the plugin publishes the
committed pose through a kinematic actor. Chaos gravity, buoyancy, collision,
and force application are disabled for this actor. This avoids combining the
Otter's coupled 6x6 mass matrix with a second rigid-body solve.

## Visuals

Dynamics and rendering use separate components. The hidden root represents the
physical state; its child mesh has no collision or mass.

Without an asset, the plugin builds a generic Aerosonde or twin-hull Otter from
the model dimensions. An aircraft static mesh can be selected in the aircraft
JSON:

```json
"visual": {
  "asset": "/Game/Aircraft/MyAircraft/SM_MyAircraft.SM_MyAircraft",
  "scale": 1.0,
  "rotation_rpy_deg": [0.0, 0.0, 0.0],
  "offset_body_m": [0.0, 0.0, 0.0]
}
```

The offset runs from the vehicle origin to the mesh origin in FRD meters.
Rotation aligns the imported mesh with Unreal X-forward, Y-right, Z-up. A
missing asset falls back to the generated visual.

## Camera and pause

```text
1   chase
2   orbit
3   onboard
4   side
F   frame the vehicle
Space   pause or resume the model
```

Right-drag orbits and the scroll wheel zooms. The camera uses the visual bounds
instead of aircraft dimensions, so the same controls frame different vehicle
shapes. Camera input and pose updates use real time while the model is paused.

Compile and restart the editor after camera or input changes. Unreal Hot Reload
can leave the module manifest pointing at an older numbered library, which
makes a successful build appear to ignore the edit. `run.py` already uses the
clean build path recorded in [AGENTS.md](../AGENTS.md).

## Current limits

- one fixed Cesium georeference per run;
- aircraft air start with no terrain contact;
- Otter flat-water rendering with no collision;
- no sensor noise, bias, or latency model; and
- no long-route origin rebasing.

Wave rendering and seakeeping notes are in [WATER.md](WATER.md).
