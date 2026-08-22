#ifndef UNREALVEHICLEDYNAMICS_CORE_INCLUDE_UVD_RIGID_BODY_HPP_
#define UNREALVEHICLEDYNAMICS_CORE_INCLUDE_UVD_RIGID_BODY_HPP_

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace uvd {

inline constexpr double kGravityMps2 = 9.80665;

// This header is the small vocabulary shared by vehicle models: coordinate
// types, rigid-body state, and an applied body wrench. It intentionally does
// not define a generic "plant" abstraction. Different vehicles can share a
// state representation while still requiring fundamentally different mass,
// force, or integration equations.
using Vector3 = Eigen::Vector3d;
using Vector4 = Eigen::Vector4d;
using Matrix3 = Eigen::Matrix3d;
using Quaternion = Eigen::Quaterniond;

// Quaternion conventions are Hamilton (w, x, y, z), active, body to NED.
[[nodiscard]] Quaternion normalize_quaternion(Quaternion q) noexcept;
[[nodiscard]] Quaternion canonicalize(Quaternion q) noexcept;
[[nodiscard]] Quaternion quaternion_from_euler(double roll_rad,
                                               double pitch_rad,
                                               double yaw_rad) noexcept;
[[nodiscard]] Vector3 rotation_vector(Quaternion q) noexcept;
[[nodiscard]] Quaternion exp_quaternion(
    const Vector3& rotation_vector_rad) noexcept;

struct RigidBodyState {
  Vector3 position_ned_m = Vector3::Zero();
  Quaternion q_body_to_ned = Quaternion::Identity();
  Vector3 velocity_body_mps = Vector3::Zero();
  Vector3 omega_body_radps = Vector3::Zero();
};

struct BodyWrench {
  Vector3 force_body_N = Vector3::Zero();
  Vector3 moment_body_Nm = Vector3::Zero();
};

struct StateDerivative {
  Vector3 position_dot_ned_mps = Vector3::Zero();
  Vector4 quaternion_dot_wxyz = Vector4::Zero();
  Vector3 velocity_dot_body_mps2 = Vector3::Zero();
  Vector3 omega_dot_body_radps2 = Vector3::Zero();
};

// This derivative is deliberately narrower than the shared state above. It
// models a conventional rigid body with scalar translational mass, a 3x3 body
// inertia tensor, and gravity added here. Coupled added mass, hydrostatics, or
// augmented vehicle state belong in a model-specific solver instead of being
// forced through this aircraft-suitable equation.
[[nodiscard]] StateDerivative rigid_body_derivative(
    const RigidBodyState& state, const BodyWrench& applied_wrench,
    double mass_kg, const Matrix3& inertia_body_kgm2) noexcept;

[[nodiscard]] bool is_finite(const RigidBodyState& state) noexcept;
[[nodiscard]] bool is_finite(const BodyWrench& wrench) noexcept;

}  // namespace uvd

#endif  // UNREALVEHICLEDYNAMICS_CORE_INCLUDE_UVD_RIGID_BODY_HPP_
