"""Run in Unreal Editor with -EnablePlugins=PythonScriptPlugin -ExecutePythonScript=... ."""

import unreal


level = unreal.get_editor_subsystem(unreal.LevelEditorSubsystem)
actors = unreal.get_editor_subsystem(unreal.EditorActorSubsystem)
if unreal.EditorAssetLibrary.does_asset_exist('/Game/Tests/OtterWater'):
    assert level.load_level('/Game/Tests/OtterWater')
else:
    assert level.new_level('/Game/Tests/OtterWater')


def spawn(cls, label, location=unreal.Vector(), rotation=unreal.Rotator()):
    for existing in actors.get_all_level_actors():
        if existing.get_actor_label() == label:
            return existing
    actor = actors.spawn_actor_from_class(cls, location, rotation)
    actor.set_actor_label(label)
    return actor


sun = spawn(unreal.DirectionalLight, 'WaterTestSun', rotation=unreal.Rotator(-35, -30, 0))
sun.light_component.set_mobility(unreal.ComponentMobility.MOVABLE)
sun.light_component.set_editor_property('intensity', 5.0)
sun.light_component.set_editor_property('atmosphere_sun_light', True)
spawn(unreal.SkyAtmosphere, 'WaterTestAtmosphere')
sky = spawn(unreal.SkyLight, 'WaterTestSkyLight')
sky.light_component.set_mobility(unreal.ComponentMobility.MOVABLE)
sky.light_component.set_editor_property('real_time_capture', True)
spawn(unreal.ExponentialHeightFog, 'WaterTestFog')

# Unreal centimetres; origin is the configured sea datum. No landscape or buoyancy.
zone = spawn(unreal.WaterZone, 'OtterWaterZone')
zone.set_editor_property('zone_extent', unreal.Vector2D(200000, 200000))
ocean = spawn(unreal.WaterBodyOcean, 'OtterVisualOcean')
component = ocean.get_component_by_class(unreal.WaterBodyOceanComponent)
component.set_editor_property('affects_landscape', False)
component.set_editor_property('ocean_extents', unreal.Vector2D(200000, 200000))
component.set_water_material(unreal.load_asset('/Water/Materials/WaterSurface/Water_Material_Ocean'))
component.set_water_zone_override(zone)
# Keep the ocean's island opening away from the vessel's local test route.
spline = ocean.get_component_by_class(unreal.WaterSplineComponent)
for index, point in enumerate([
    unreal.Vector(-90000, -90000, 0),
    unreal.Vector(-89000, -90000, 0),
    unreal.Vector(-89000, -89000, 0),
    unreal.Vector(-90000, -89000, 0),
]):
    spline.set_location_at_spline_point(index, point, unreal.SplineCoordinateSpace.LOCAL, False)
spline.update_spline()
ocean.set_water_waves(None)
assert level.save_current_level()
unreal.log('UVD_WATER_LEVEL_SAVED /Game/Tests/OtterWater')
