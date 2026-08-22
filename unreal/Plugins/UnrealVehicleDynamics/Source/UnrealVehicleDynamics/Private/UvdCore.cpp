// UnrealBuildTool does not consume the CMake target. Compile the same source
// files into the plugin so Unreal and the headless CLI share one force model.
#include "fixed_wing.cpp"
#include "fossen_otter.cpp"
#include "geodesy.cpp"
#include "marine_integrator.cpp"
#include "rigid_body.cpp"
