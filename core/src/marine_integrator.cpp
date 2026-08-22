#include <algorithm>

#include "uvd/marine.hpp"

namespace uvd {
namespace {

Vector4 quaternion_wxyz(const Quaternion& q) noexcept {
  return {q.w(), q.x(), q.y(), q.z()};
}

Quaternion quaternion_from_wxyz(const Vector4& q) noexcept {
  return {q(0), q(1), q(2), q(3)};
}

OtterState add(const OtterState& state, const OtterDerivative& derivative,
               double scale) noexcept {
  OtterState result = state;
  result.body.position_ned_m += scale * derivative.body.position_dot_ned_mps;
  result.body.q_body_to_ned = normalize_quaternion(
      quaternion_from_wxyz(quaternion_wxyz(state.body.q_body_to_ned) +
                           scale * derivative.body.quaternion_dot_wxyz));
  result.body.velocity_body_mps +=
      scale * derivative.body.velocity_dot_body_mps2;
  result.body.omega_body_radps += scale * derivative.body.omega_dot_body_radps2;
  result.actuators.left_propeller_speed_radps +=
      scale * derivative.left_propeller_speed_dot_radps2;
  result.actuators.right_propeller_speed_radps +=
      scale * derivative.right_propeller_speed_dot_radps2;
  return result;
}

OtterDerivative average(const OtterDerivative& k1, const OtterDerivative& k2,
                        const OtterDerivative& k3,
                        const OtterDerivative& k4) noexcept {
  OtterDerivative result;
  result.body.position_dot_ned_mps =
      (k1.body.position_dot_ned_mps + 2.0 * k2.body.position_dot_ned_mps +
       2.0 * k3.body.position_dot_ned_mps + k4.body.position_dot_ned_mps) /
      6.0;
  result.body.quaternion_dot_wxyz =
      (k1.body.quaternion_dot_wxyz + 2.0 * k2.body.quaternion_dot_wxyz +
       2.0 * k3.body.quaternion_dot_wxyz + k4.body.quaternion_dot_wxyz) /
      6.0;
  result.body.velocity_dot_body_mps2 =
      (k1.body.velocity_dot_body_mps2 + 2.0 * k2.body.velocity_dot_body_mps2 +
       2.0 * k3.body.velocity_dot_body_mps2 + k4.body.velocity_dot_body_mps2) /
      6.0;
  result.body.omega_dot_body_radps2 =
      (k1.body.omega_dot_body_radps2 + 2.0 * k2.body.omega_dot_body_radps2 +
       2.0 * k3.body.omega_dot_body_radps2 + k4.body.omega_dot_body_radps2) /
      6.0;
  result.left_propeller_speed_dot_radps2 =
      (k1.left_propeller_speed_dot_radps2 +
       2.0 * k2.left_propeller_speed_dot_radps2 +
       2.0 * k3.left_propeller_speed_dot_radps2 +
       k4.left_propeller_speed_dot_radps2) /
      6.0;
  result.right_propeller_speed_dot_radps2 =
      (k1.right_propeller_speed_dot_radps2 +
       2.0 * k2.right_propeller_speed_dot_radps2 +
       2.0 * k3.right_propeller_speed_dot_radps2 +
       k4.right_propeller_speed_dot_radps2) /
      6.0;
  return result;
}

}  // namespace

OtterState step_fossen_otter_rk4(const OtterState& state,
                                 const OtterCommand& command,
                                 const OtterEnvironment& environment,
                                 const FossenOtterModel& model,
                                 double dt_s) noexcept {
  const auto derivative = [&](const OtterState& stage) {
    return evaluate_fossen_otter(stage, command, environment, model).derivative;
  };
  const OtterDerivative k1 = derivative(state);
  const OtterDerivative k2 = derivative(add(state, k1, 0.5 * dt_s));
  const OtterDerivative k3 = derivative(add(state, k2, 0.5 * dt_s));
  const OtterDerivative k4 = derivative(add(state, k3, dt_s));
  OtterState result = add(state, average(k1, k2, k3, k4), dt_s);
  result.body.q_body_to_ned = canonicalize(result.body.q_body_to_ned);
  result.actuators.left_propeller_speed_radps = std::clamp(
      result.actuators.left_propeller_speed_radps,
      model.minimum_propeller_speed_radps, model.maximum_propeller_speed_radps);
  result.actuators.right_propeller_speed_radps = std::clamp(
      result.actuators.right_propeller_speed_radps,
      model.minimum_propeller_speed_radps, model.maximum_propeller_speed_radps);
  return result;
}

}  // namespace uvd
