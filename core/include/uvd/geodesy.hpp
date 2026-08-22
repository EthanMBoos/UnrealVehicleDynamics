#ifndef UNREALVEHICLEDYNAMICS_CORE_INCLUDE_UVD_GEODESY_HPP_
#define UNREALVEHICLEDYNAMICS_CORE_INCLUDE_UVD_GEODESY_HPP_

#include "uvd/rigid_body.hpp"

namespace uvd {

// Geodesy is shared infrastructure rather than vehicle behavior. Keeping it
// independent lets every plant use the same NED/geodetic boundary without
// inheriting aircraft parameters or aerodynamic equations.
struct GeodeticPosition {
  double latitude_deg{};
  double longitude_deg{};
  double ellipsoid_height_m{};
};

[[nodiscard]] double ellipsoid_height_from_msl(
    double altitude_msl_m, double geoid_undulation_m) noexcept;
[[nodiscard]] GeodeticPosition geodetic_from_ned(
    const GeodeticPosition& origin, const Vector3& position_ned_m) noexcept;
[[nodiscard]] Vector3 ned_from_geodetic(
    const GeodeticPosition& origin, const GeodeticPosition& position) noexcept;

}  // namespace uvd

#endif  // UNREALVEHICLEDYNAMICS_CORE_INCLUDE_UVD_GEODESY_HPP_
