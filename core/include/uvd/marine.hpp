#ifndef UNREALVEHICLEDYNAMICS_CORE_INCLUDE_UVD_MARINE_HPP_
#define UNREALVEHICLEDYNAMICS_CORE_INCLUDE_UVD_MARINE_HPP_

#include <Eigen/Cholesky>
#include <string>

#include "uvd/rigid_body.hpp"

namespace uvd {

using Vector2 = Eigen::Vector2d;
using Vector6 = Eigen::Matrix<double, 6, 1>;
using Matrix6 = Eigen::Matrix<double, 6, 6>;

// The Otter types are deliberately concrete. Marine vehicles share the
// repository's SI/NED/FRD rigid-body vocabulary, but the coupled fluid/body
// mass and shaft states are not variations of the Aerosonde equations.
struct OtterActuatorState {
  double left_propeller_speed_radps{};
  double right_propeller_speed_radps{};
};

struct OtterState {
  RigidBodyState body;
  OtterActuatorState actuators;
};

// Commands are physical shaft-speed demands. PWM interpretation belongs at a
// controller boundary, keeping the hydrodynamic model useful without
// ArduPilot and making reference comparisons unambiguous.
struct OtterCommand {
  double left_propeller_speed_command_radps{};
  double right_propeller_speed_command_radps{};
};

struct OtterEnvironment {
  double water_surface_down_m{};
  Vector3 current_ned_mps = Vector3::Zero();
};

struct FossenOtterParameters {
  std::string model_id{"fossen_otter_educational"};
  double gravity_mps2{9.81};
  double water_density_kgpm3{1025.0};
  double length_m{2.0};
  double beam_m{1.08};
  double hull_mass_kg{55.0};
  Vector3 hull_cg_body_m{0.2, 0.0, -0.2};
  Vector3 hull_radii_of_gyration_m{0.432, 0.5, 0.5};
  double payload_mass_kg{25.0};
  Vector3 payload_position_body_m{0.05, 0.0, -0.35};
  double pontoon_beam_m{0.25};
  double pontoon_centerline_offset_m{0.395};
  double pontoon_waterline_area_coefficient{0.75};
  double pontoon_block_coefficient{0.4};
  double center_of_flotation_x_m{-0.2};
  double maximum_forward_speed_mps{3.0864};
  double sway_time_constant_s{1.0};
  double yaw_time_constant_s{1.0};
  double heave_damping_ratio{0.3};
  double roll_damping_ratio{0.2};
  double pitch_damping_ratio{0.4};
  double nonlinear_yaw_factor{10.0};
  double surge_added_mass_coefficient{2.7};
  double sway_added_mass_ratio{1.5};
  double heave_added_mass_ratio{1.0};
  double roll_added_inertia_ratio{0.2};
  double pitch_added_inertia_ratio{0.8};
  double yaw_added_inertia_ratio{1.7};
  double forward_thrust_coefficient{0.01108};
  double reverse_thrust_coefficient{0.006445};
  double maximum_forward_bollard_force_N{119.682};
  double maximum_reverse_bollard_force_N{66.708};
  double propeller_response_time_s{0.1};
};

// Construction turns source scalars into the matrices used every step. The
// factorization is cached because the effective mass is constant for a loaded
// configuration; the payload restoring offset is not cached because gravity
// expressed in the body frame changes with attitude.
struct FossenOtterModel {
  FossenOtterParameters parameters;
  double total_mass_kg{};
  Vector3 combined_cg_body_m = Vector3::Zero();
  Matrix3 combined_inertia_cg_kgm2 = Matrix3::Zero();
  double displaced_volume_m3{};
  double loaded_draft_m{};
  double surge_added_mass_kg{};
  double hoerner_crossflow_coefficient{};
  double minimum_propeller_speed_radps{};
  double maximum_propeller_speed_radps{};
  Matrix6 rigid_body_mass = Matrix6::Zero();
  Matrix6 added_mass = Matrix6::Zero();
  Matrix6 effective_mass = Matrix6::Zero();
  Matrix6 hydrostatic_stiffness = Matrix6::Zero();
  Matrix3 hydrostatic_heave_roll_pitch_inverse = Matrix3::Zero();
  Vector6 linear_damping = Vector6::Zero();
  Eigen::LDLT<Matrix6> effective_mass_ldlt;
};

struct OtterDerivative {
  StateDerivative body;
  double left_propeller_speed_dot_radps2{};
  double right_propeller_speed_dot_radps2{};
};

struct OtterModelOutput {
  Vector6 current_velocity_body = Vector6::Zero();
  Vector6 relative_velocity_body = Vector6::Zero();
  Vector3 restoring_offset_down_roll_pitch = Vector3::Zero();
  double left_thrust_N{};
  double right_thrust_N{};
  BodyWrench propeller_wrench;
  BodyWrench linear_damping_wrench;
  BodyWrench crossflow_wrench;
  BodyWrench coriolis_wrench;
  BodyWrench hydrostatic_wrench;
  BodyWrench total_wrench;
  Vector6 velocity_rate_body = Vector6::Zero();
  OtterDerivative derivative;
  bool valid{true};
};

[[nodiscard]] FossenOtterModel make_fossen_otter_model(
    const FossenOtterParameters& parameters);

[[nodiscard]] OtterModelOutput evaluate_fossen_otter(
    const OtterState& state, const OtterCommand& command,
    const OtterEnvironment& environment,
    const FossenOtterModel& model) noexcept;

[[nodiscard]] OtterState step_fossen_otter_rk4(
    const OtterState& state, const OtterCommand& command,
    const OtterEnvironment& environment, const FossenOtterModel& model,
    double dt_s) noexcept;

[[nodiscard]] bool is_finite(const OtterState& state) noexcept;

}  // namespace uvd

#endif  // UNREALVEHICLEDYNAMICS_CORE_INCLUDE_UVD_MARINE_HPP_
