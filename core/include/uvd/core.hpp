#ifndef UNREALVEHICLEDYNAMICS_CORE_INCLUDE_UVD_CORE_HPP_
#define UNREALVEHICLEDYNAMICS_CORE_INCLUDE_UVD_CORE_HPP_

// Compatibility umbrella for existing consumers, including the Unreal
// component. New core sources should include the narrow header they own. This
// preserves the proven public include while allowing marine code to depend on
// shared mechanics without also inheriting the fixed-wing API.
#include "uvd/fixed_wing.hpp"
#include "uvd/geodesy.hpp"
#include "uvd/rigid_body.hpp"

#endif  // UNREALVEHICLEDYNAMICS_CORE_INCLUDE_UVD_CORE_HPP_
