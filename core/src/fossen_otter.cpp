#include <algorithm>
#include <array>
#include <cmath>
#include <stdexcept>

#include "uvd/marine.hpp"

namespace uvd {
namespace {

Matrix3 skew(const Vector3& value) noexcept {
  Matrix3 result;
  result << 0.0, -value.z(), value.y(), value.z(), 0.0, -value.x(), -value.y(),
      value.x(), 0.0;
  return result;
}

Matrix6 transform_matrix(const Vector3& offset_body_m) noexcept {
  Matrix6 result = Matrix6::Identity();
  result.block<3, 3>(0, 3) = skew(offset_body_m).transpose();
  return result;
}

Matrix6 coriolis_from_mass(const Matrix6& raw_mass,
                           const Vector6& velocity) noexcept {
  const Matrix6 mass = 0.5 * (raw_mass + raw_mass.transpose());
  const Matrix3 M11 = mass.block<3, 3>(0, 0);
  const Matrix3 M12 = mass.block<3, 3>(0, 3);
  const Matrix3 M21 = M12.transpose();
  const Matrix3 M22 = mass.block<3, 3>(3, 3);
  const Vector3 linear_momentum =
      M11 * velocity.head<3>() + M12 * velocity.tail<3>();
  const Vector3 angular_momentum =
      M21 * velocity.head<3>() + M22 * velocity.tail<3>();
  Matrix6 result = Matrix6::Zero();
  result.block<3, 3>(0, 3) = -skew(linear_momentum);
  result.block<3, 3>(3, 0) = -skew(linear_momentum);
  result.block<3, 3>(3, 3) = -skew(angular_momentum);
  return result;
}

double hoerner_coefficient(double beam_m, double draft_m) noexcept {
  constexpr std::array<std::array<double, 2>, 20> kData{{
      {0.0108623, 1.96608}, {0.176606, 1.96573},  {0.353025, 1.89756},
      {0.451863, 1.78718},  {0.472838, 1.58374},  {0.492877, 1.27862},
      {0.493252, 1.21082},  {0.558473, 1.08356},  {0.646401, 0.998631},
      {0.833589, 0.87959},  {0.988002, 0.828415}, {1.30807, 0.759941},
      {1.63918, 0.691442},  {1.85998, 0.657076},  {2.31288, 0.630693},
      {2.59998, 0.596186},  {3.00877, 0.586846},  {3.45075, 0.585909},
      {3.7379, 0.559877},   {4.00309, 0.559315},
  }};
  const double ratio = beam_m / (2.0 * draft_m);
  if (ratio >= kData.back()[0]) {
    return kData.back()[1];
  }
  for (std::size_t i = 1; i < kData.size(); ++i) {
    if (ratio <= kData[i][0]) {
      const double fraction =
          (ratio - kData[i - 1][0]) / (kData[i][0] - kData[i - 1][0]);
      return std::lerp(kData[i - 1][1], kData[i][1], fraction);
    }
  }
  return std::numeric_limits<double>::quiet_NaN();
}

Vector3 euler_zyx(const Quaternion& raw_q) noexcept {
  const Matrix3 rotation = normalize_quaternion(raw_q).toRotationMatrix();
  return {std::atan2(rotation(2, 1), rotation(2, 2)),
          std::asin(std::clamp(-rotation(2, 0), -1.0, 1.0)),
          std::atan2(rotation(1, 0), rotation(0, 0))};
}

BodyWrench wrench_from_vector(const Vector6& value) noexcept {
  return {.force_body_N = value.head<3>(), .moment_body_Nm = value.tail<3>()};
}

double propeller_thrust(double speed_radps,
                        const FossenOtterParameters& parameters) noexcept {
  const double coefficient = speed_radps > 0.0
                                 ? parameters.forward_thrust_coefficient
                                 : parameters.reverse_thrust_coefficient;
  return coefficient * speed_radps * std::abs(speed_radps);
}

Vector6 crossflow_drag(const Vector6& relative_velocity,
                       const FossenOtterModel& model) noexcept {
  const auto& p = model.parameters;
  constexpr int kStripCount = 20;
  const double dx_m = p.length_m / kStripCount;
  double sway_N = 0.0;
  double heave_N = 0.0;
  double pitch_Nm = 0.0;
  double yaw_Nm = 0.0;
  for (int strip = 0; strip <= kStripCount; ++strip) {
    const double x_m = -0.5 * p.length_m + strip * dx_m;
    const double sway_mps = relative_velocity(1) + x_m * relative_velocity(5);
    const double heave_mps = relative_velocity(2) + x_m * relative_velocity(4);
    const double scale = 0.5 * p.water_density_kgpm3 * model.loaded_draft_m *
                         model.hoerner_crossflow_coefficient * dx_m;
    sway_N -= scale * std::abs(sway_mps) * sway_mps;
    heave_N -= scale * std::abs(heave_mps) * heave_mps;
    pitch_Nm -= scale * x_m * std::abs(heave_mps) * heave_mps;
    yaw_Nm -= scale * x_m * std::abs(sway_mps) * sway_mps;
  }
  Vector6 result = Vector6::Zero();
  result << 0.0, sway_N, heave_N, 0.0, pitch_Nm, yaw_Nm;
  return result;
}

bool positive_finite(double value) noexcept {
  return std::isfinite(value) && value > 0.0;
}

}  // namespace

FossenOtterModel make_fossen_otter_model(const FossenOtterParameters& p) {
  const auto require_positive = [](double value, const char* name) {
    if (!positive_finite(value)) {
      throw std::runtime_error(std::string(name) + " must be positive");
    }
  };
  require_positive(p.gravity_mps2, "gravity_mps2");
  require_positive(p.water_density_kgpm3, "water_density_kgpm3");
  require_positive(p.length_m, "length_m");
  require_positive(p.beam_m, "beam_m");
  require_positive(p.hull_mass_kg, "hull_mass_kg");
  require_positive(p.pontoon_beam_m, "pontoon_beam_m");
  require_positive(p.pontoon_centerline_offset_m,
                   "pontoon_centerline_offset_m");
  require_positive(p.pontoon_waterline_area_coefficient,
                   "pontoon_waterline_area_coefficient");
  require_positive(p.pontoon_block_coefficient, "pontoon_block_coefficient");
  require_positive(p.maximum_forward_speed_mps, "maximum_forward_speed_mps");
  require_positive(p.sway_time_constant_s, "sway_time_constant_s");
  require_positive(p.yaw_time_constant_s, "yaw_time_constant_s");
  require_positive(p.propeller_response_time_s, "propeller_response_time_s");
  require_positive(p.forward_thrust_coefficient, "forward_thrust_coefficient");
  require_positive(p.reverse_thrust_coefficient, "reverse_thrust_coefficient");
  if (!p.hull_cg_body_m.allFinite() ||
      !p.hull_radii_of_gyration_m.allFinite() ||
      !(p.hull_radii_of_gyration_m.array() > 0.0).all() ||
      !p.payload_position_body_m.allFinite() ||
      !std::isfinite(p.payload_mass_kg) || p.payload_mass_kg < 0.0 ||
      p.payload_mass_kg > 45.0) {
    throw std::runtime_error("Otter mass properties are invalid");
  }

  FossenOtterModel model;
  model.parameters = p;
  model.total_mass_kg = p.hull_mass_kg + p.payload_mass_kg;
  model.combined_cg_body_m = (p.hull_mass_kg * p.hull_cg_body_m +
                              p.payload_mass_kg * p.payload_position_body_m) /
                             model.total_mass_kg;
  const Matrix3 hull_inertia =
      p.hull_mass_kg *
      p.hull_radii_of_gyration_m.array().square().matrix().asDiagonal();
  const Vector3 hull_from_combined =
      p.hull_cg_body_m - model.combined_cg_body_m;
  const Vector3 payload_from_combined =
      p.payload_position_body_m - model.combined_cg_body_m;
  const Matrix3 hull_offset_skew = skew(hull_from_combined);
  const Matrix3 payload_offset_skew = skew(payload_from_combined);
  model.combined_inertia_cg_kgm2 =
      hull_inertia - p.hull_mass_kg * hull_offset_skew * hull_offset_skew -
      p.payload_mass_kg * payload_offset_skew * payload_offset_skew;

  Matrix6 mass_at_cg = Matrix6::Zero();
  mass_at_cg.block<3, 3>(0, 0) = model.total_mass_kg * Matrix3::Identity();
  mass_at_cg.block<3, 3>(3, 3) = model.combined_inertia_cg_kgm2;
  const Matrix6 H = transform_matrix(model.combined_cg_body_m);
  model.rigid_body_mass = H.transpose() * mass_at_cg * H;

  model.displaced_volume_m3 = model.total_mass_kg / p.water_density_kgpm3;
  model.loaded_draft_m =
      model.displaced_volume_m3 /
      (2.0 * p.pontoon_block_coefficient * p.pontoon_beam_m * p.length_m);
  model.surge_added_mass_kg =
      p.surge_added_mass_coefficient * p.water_density_kgpm3 *
      std::pow(p.hull_mass_kg / p.water_density_kgpm3, 5.0 / 3.0) /
      std::pow(p.length_m, 2.0);
  model.added_mass.diagonal() << model.surge_added_mass_kg,
      p.sway_added_mass_ratio * p.hull_mass_kg,
      p.heave_added_mass_ratio * p.hull_mass_kg,
      p.roll_added_inertia_ratio * model.combined_inertia_cg_kgm2(0, 0),
      p.pitch_added_inertia_ratio * model.combined_inertia_cg_kgm2(1, 1),
      p.yaw_added_inertia_ratio * model.combined_inertia_cg_kgm2(2, 2);
  model.effective_mass = model.rigid_body_mass + model.added_mass;
  model.effective_mass_ldlt.compute(model.effective_mass);
  if (model.effective_mass_ldlt.info() != Eigen::Success ||
      !(model.effective_mass_ldlt.vectorD().array() > 0.0).all()) {
    throw std::runtime_error("Otter effective mass must be positive definite");
  }

  const double waterline_area =
      p.pontoon_waterline_area_coefficient * p.length_m * p.pontoon_beam_m;
  const double transverse_waterplane_inertia =
      2.0 * (1.0 / 12.0) * p.length_m * std::pow(p.pontoon_beam_m, 3.0) *
          (6.0 * std::pow(p.pontoon_waterline_area_coefficient, 3.0) /
           ((1.0 + p.pontoon_waterline_area_coefficient) *
            (1.0 + 2.0 * p.pontoon_waterline_area_coefficient))) +
      2.0 * waterline_area * std::pow(p.pontoon_centerline_offset_m, 2.0);
  const double longitudinal_waterplane_inertia =
      0.8 * 2.0 * (1.0 / 12.0) * p.pontoon_beam_m * std::pow(p.length_m, 3.0);
  const double center_of_buoyancy_m =
      (1.0 / 3.0) *
      (2.5 * model.loaded_draft_m -
       0.5 * model.displaced_volume_m3 / (p.length_m * p.pontoon_beam_m));
  const double transverse_metacentric_height_m =
      center_of_buoyancy_m +
      transverse_waterplane_inertia / model.displaced_volume_m3 -
      (model.loaded_draft_m - model.combined_cg_body_m.z());
  const double longitudinal_metacentric_height_m =
      center_of_buoyancy_m +
      longitudinal_waterplane_inertia / model.displaced_volume_m3 -
      (model.loaded_draft_m - model.combined_cg_body_m.z());
  if (!(transverse_metacentric_height_m > 0.0) ||
      !(longitudinal_metacentric_height_m > 0.0)) {
    throw std::runtime_error("Otter metacentric height must be positive");
  }
  Matrix6 stiffness_at_flotation = Matrix6::Zero();
  stiffness_at_flotation(2, 2) =
      p.water_density_kgpm3 * p.gravity_mps2 * (2.0 * waterline_area);
  stiffness_at_flotation(3, 3) = p.water_density_kgpm3 * p.gravity_mps2 *
                                 model.displaced_volume_m3 *
                                 transverse_metacentric_height_m;
  stiffness_at_flotation(4, 4) = p.water_density_kgpm3 * p.gravity_mps2 *
                                 model.displaced_volume_m3 *
                                 longitudinal_metacentric_height_m;
  const Matrix6 flotation_transform =
      transform_matrix({p.center_of_flotation_x_m, 0.0, 0.0});
  model.hydrostatic_stiffness = flotation_transform.transpose() *
                                stiffness_at_flotation * flotation_transform;
  const Matrix3 stiffness_sub = model.hydrostatic_stiffness.block<3, 3>(2, 2);
  if (std::abs(stiffness_sub.determinant()) < 1e-12) {
    throw std::runtime_error("Otter hydrostatic stiffness is singular");
  }
  model.hydrostatic_heave_roll_pitch_inverse = stiffness_sub.inverse();

  const double heave_frequency =
      std::sqrt(stiffness_at_flotation(2, 2) / model.effective_mass(2, 2));
  const double roll_frequency =
      std::sqrt(stiffness_at_flotation(3, 3) / model.effective_mass(3, 3));
  const double pitch_frequency =
      std::sqrt(stiffness_at_flotation(4, 4) / model.effective_mass(4, 4));
  // MSS derives the surge term from 24.4 kgf because that is the Otter's
  // measured total bollard thrust. Use the configured two-propeller force here
  // so changing the propulsion data does not silently leave the old drag fit.
  model.linear_damping << -2.0 * p.maximum_forward_bollard_force_N /
                              p.maximum_forward_speed_mps,
      -model.effective_mass(1, 1) / p.sway_time_constant_s,
      -2.0 * p.heave_damping_ratio * heave_frequency *
          model.effective_mass(2, 2),
      -2.0 * p.roll_damping_ratio * roll_frequency * model.effective_mass(3, 3),
      -2.0 * p.pitch_damping_ratio * pitch_frequency *
          model.effective_mass(4, 4),
      -model.effective_mass(5, 5) / p.yaw_time_constant_s;

  const double ratio = p.pontoon_beam_m / (2.0 * model.loaded_draft_m);
  if (ratio < 0.0108623) {
    throw std::runtime_error("Otter pontoon beam/draft is below Hoerner data");
  }
  model.hoerner_crossflow_coefficient =
      hoerner_coefficient(p.pontoon_beam_m, model.loaded_draft_m);
  model.maximum_propeller_speed_radps = std::sqrt(
      p.maximum_forward_bollard_force_N / p.forward_thrust_coefficient);
  model.minimum_propeller_speed_radps = -std::sqrt(
      p.maximum_reverse_bollard_force_N / p.reverse_thrust_coefficient);
  if (!std::isfinite(model.hoerner_crossflow_coefficient) ||
      !(model.minimum_propeller_speed_radps < 0.0) ||
      !(model.maximum_propeller_speed_radps > 0.0)) {
    throw std::runtime_error(
        "Otter cross-flow or propeller limits are invalid");
  }
  return model;
}

OtterModelOutput evaluate_fossen_otter(const OtterState& state,
                                       const OtterCommand& command,
                                       const OtterEnvironment& environment,
                                       const FossenOtterModel& model) noexcept {
  OtterModelOutput output;
  const auto& p = model.parameters;
  if (!is_finite(state) || !environment.current_ned_mps.allFinite() ||
      !std::isfinite(environment.water_surface_down_m) ||
      std::abs(environment.current_ned_mps.z()) > 1e-12 ||
      !std::isfinite(command.left_propeller_speed_command_radps) ||
      !std::isfinite(command.right_propeller_speed_command_radps)) {
    output.valid = false;
    return output;
  }

  const Quaternion q = normalize_quaternion(state.body.q_body_to_ned);
  const Matrix3 body_to_ned = q.toRotationMatrix();
  const Vector3 rpy = euler_zyx(q);
  Vector6 velocity;
  velocity << state.body.velocity_body_mps, state.body.omega_body_radps;

  const double current_speed = environment.current_ned_mps.head<2>().norm();
  const double current_direction = std::atan2(environment.current_ned_mps.y(),
                                              environment.current_ned_mps.x());
  output.current_velocity_body(0) =
      current_speed * std::cos(current_direction - rpy.z());
  output.current_velocity_body(1) =
      current_speed * std::sin(current_direction - rpy.z());
  output.relative_velocity_body = velocity - output.current_velocity_body;
  Vector6 current_acceleration = Vector6::Zero();
  current_acceleration.head<3>() = -state.body.omega_body_radps.cross(
      output.current_velocity_body.head<3>());

  Matrix6 rigid_body_coriolis_at_cg = Matrix6::Zero();
  rigid_body_coriolis_at_cg.block<3, 3>(0, 0) =
      model.total_mass_kg * skew(state.body.omega_body_radps);
  rigid_body_coriolis_at_cg.block<3, 3>(3, 3) =
      -skew(model.combined_inertia_cg_kgm2 * state.body.omega_body_radps);
  const Matrix6 H = transform_matrix(model.combined_cg_body_m);
  const Matrix6 coriolis =
      H.transpose() * rigid_body_coriolis_at_cg * H +
      coriolis_from_mass(model.added_mass, output.relative_velocity_body);
  const Vector6 coriolis_load = -coriolis * output.relative_velocity_body;
  output.coriolis_wrench = wrench_from_vector(coriolis_load);

  const double left_speed = std::clamp(
      state.actuators.left_propeller_speed_radps,
      model.minimum_propeller_speed_radps, model.maximum_propeller_speed_radps);
  const double right_speed = std::clamp(
      state.actuators.right_propeller_speed_radps,
      model.minimum_propeller_speed_radps, model.maximum_propeller_speed_radps);
  output.left_thrust_N = propeller_thrust(left_speed, p);
  output.right_thrust_N = propeller_thrust(right_speed, p);
  Vector6 propeller_load = Vector6::Zero();
  propeller_load(0) = output.left_thrust_N + output.right_thrust_N;
  propeller_load(5) = p.pontoon_centerline_offset_m *
                      (output.left_thrust_N - output.right_thrust_N);
  output.propeller_wrench = wrench_from_vector(propeller_load);

  Vector6 damping_load =
      model.linear_damping.cwiseProduct(output.relative_velocity_body);
  damping_load(5) *=
      1.0 + p.nonlinear_yaw_factor * std::abs(output.relative_velocity_body(5));
  output.linear_damping_wrench = wrench_from_vector(damping_load);
  const Vector6 crossflow_load =
      crossflow_drag(output.relative_velocity_body, model);
  output.crossflow_wrench = wrench_from_vector(crossflow_load);

  const Vector3 payload_force_body =
      body_to_ned.transpose() *
      Vector3{0.0, 0.0, p.payload_mass_kg * p.gravity_mps2};
  const Vector3 payload_moment_body =
      p.payload_position_body_m.cross(payload_force_body);
  const Vector3 payload_heave_roll_pitch{
      payload_force_body.z(), payload_moment_body.x(), payload_moment_body.y()};
  output.restoring_offset_down_roll_pitch =
      model.hydrostatic_heave_roll_pitch_inverse * payload_heave_roll_pitch;
  Vector6 displacement = Vector6::Zero();
  displacement << 0.0, 0.0,
      state.body.position_ned_m.z() - environment.water_surface_down_m -
          output.restoring_offset_down_roll_pitch.x(),
      rpy.x() - output.restoring_offset_down_roll_pitch.y(),
      rpy.y() - output.restoring_offset_down_roll_pitch.z(), 0.0;
  const Vector6 hydrostatic_load = -model.hydrostatic_stiffness * displacement;
  output.hydrostatic_wrench = wrench_from_vector(hydrostatic_load);

  const Vector6 total_load = propeller_load + damping_load + crossflow_load +
                             coriolis_load + hydrostatic_load;
  output.total_wrench = wrench_from_vector(total_load);
  output.velocity_rate_body =
      current_acceleration + model.effective_mass_ldlt.solve(total_load);

  const Quaternion omega{0.0, state.body.omega_body_radps.x(),
                         state.body.omega_body_radps.y(),
                         state.body.omega_body_radps.z()};
  const Quaternion q_dot = q * omega;
  output.derivative.body.position_dot_ned_mps =
      body_to_ned * state.body.velocity_body_mps;
  output.derivative.body.quaternion_dot_wxyz =
      0.5 * Vector4{q_dot.w(), q_dot.x(), q_dot.y(), q_dot.z()};
  output.derivative.body.velocity_dot_body_mps2 =
      output.velocity_rate_body.head<3>();
  output.derivative.body.omega_dot_body_radps2 =
      output.velocity_rate_body.tail<3>();
  const double left_command = std::clamp(
      command.left_propeller_speed_command_radps,
      model.minimum_propeller_speed_radps, model.maximum_propeller_speed_radps);
  const double right_command = std::clamp(
      command.right_propeller_speed_command_radps,
      model.minimum_propeller_speed_radps, model.maximum_propeller_speed_radps);
  output.derivative.left_propeller_speed_dot_radps2 =
      (left_command - state.actuators.left_propeller_speed_radps) /
      p.propeller_response_time_s;
  output.derivative.right_propeller_speed_dot_radps2 =
      (right_command - state.actuators.right_propeller_speed_radps) /
      p.propeller_response_time_s;

  output.valid =
      is_finite(output.total_wrench) && output.velocity_rate_body.allFinite() &&
      output.derivative.body.position_dot_ned_mps.allFinite() &&
      output.derivative.body.quaternion_dot_wxyz.allFinite() &&
      std::isfinite(output.derivative.left_propeller_speed_dot_radps2) &&
      std::isfinite(output.derivative.right_propeller_speed_dot_radps2);
  return output;
}

bool is_finite(const OtterState& state) noexcept {
  return is_finite(state.body) &&
         std::isfinite(state.actuators.left_propeller_speed_radps) &&
         std::isfinite(state.actuators.right_propeller_speed_radps);
}

}  // namespace uvd
