#include <cmath>
#include <cstdlib>
#include <iomanip>
#include <iostream>
#include <numbers>
#include <sstream>
#include <stdexcept>
#include <string>

#include "uvd/marine.hpp"

namespace {

void expect_near(double actual, double expected, double tolerance,
                 const std::string& name) {
  if (!std::isfinite(actual) || std::abs(actual - expected) > tolerance) {
    std::ostringstream message;
    message << std::setprecision(17) << name << ": expected " << expected
            << ", got " << actual << ", difference " << actual - expected;
    throw std::runtime_error(message.str());
  }
}

void expect_true(bool condition, const std::string& name) {
  if (!condition) {
    throw std::runtime_error(name);
  }
}

void expect_vector_near(const uvd::Vector6& actual,
                        const uvd::Vector6& expected, double tolerance,
                        const std::string& name) {
  for (int i = 0; i < 6; ++i) {
    expect_near(actual(i), expected(i), tolerance,
                name + "[" + std::to_string(i) + "]");
  }
}

uvd::Vector6 wrench_vector(const uvd::BodyWrench& wrench) {
  uvd::Vector6 result;
  result << wrench.force_body_N, wrench.moment_body_Nm;
  return result;
}

uvd::OtterState unloaded_equilibrium() { return {}; }

void construction_test() {
  const uvd::FossenOtterModel model =
      uvd::make_fossen_otter_model(uvd::FossenOtterParameters{});
  expect_near(model.total_mass_kg, 80.0, 1e-12, "total mass");
  expect_near(model.combined_cg_body_m.x(), 0.153125, 1e-12, "combined CG x");
  expect_near(model.combined_cg_body_m.z(), -0.246875, 1e-12, "combined CG z");
  expect_near(model.loaded_draft_m, 0.1951219512195122, 1e-12, "draft");
  expect_near(model.surge_added_mass_kg, 5.2815210096, 1e-9,
              "surge added mass");
  expect_near(model.combined_inertia_cg_kgm2(0, 0), 10.65103875, 1e-10,
              "combined Ixx");
  expect_near(model.combined_inertia_cg_kgm2(1, 1), 14.5234375, 1e-10,
              "combined Iyy");
  expect_near(model.combined_inertia_cg_kgm2(2, 2), 14.13671875, 1e-10,
              "combined Izz");
  expect_near(model.combined_inertia_cg_kgm2(0, 2), -0.38671875, 1e-10,
              "combined Ixz");
  expect_near(model.minimum_propeller_speed_radps, -101.7366655, 1e-7,
              "minimum propeller speed");
  expect_near(model.maximum_propeller_speed_radps, 103.9308643, 1e-7,
              "maximum propeller speed");
  expect_near(model.hoerner_crossflow_coefficient, 1.00420999535984, 1e-12,
              "Hoerner interpolation");
  expect_true(
      model.effective_mass.isApprox(model.effective_mass.transpose(), 1e-13),
      "effective mass symmetry");
  expect_true(model.hydrostatic_stiffness.isApprox(
                  model.hydrostatic_stiffness.transpose(), 1e-13),
              "hydrostatic stiffness symmetry");

  uvd::FossenOtterParameters changed_thrust;
  changed_thrust.maximum_forward_bollard_force_N = 100.0;
  const uvd::FossenOtterModel changed_model =
      uvd::make_fossen_otter_model(changed_thrust);
  expect_near(changed_model.linear_damping(0),
              -2.0 * changed_thrust.maximum_forward_bollard_force_N /
                  changed_thrust.maximum_forward_speed_mps,
              1e-12, "surge damping follows configured bollard thrust");
}

void direction_test() {
  const uvd::FossenOtterModel model =
      uvd::make_fossen_otter_model(uvd::FossenOtterParameters{});
  uvd::OtterState state;
  state.actuators = {80.0, 80.0};
  uvd::OtterModelOutput output =
      uvd::evaluate_fossen_otter(state, {}, {}, model);
  expect_true(output.valid, "equal-thrust evaluation valid");
  expect_true(output.propeller_wrench.force_body_N.x() > 0.0,
              "equal thrust produces positive surge");
  expect_near(output.propeller_wrench.moment_body_Nm.z(), 0.0, 1e-12,
              "equal thrust produces zero yaw moment");

  state.actuators = {80.0, 40.0};
  output = uvd::evaluate_fossen_otter(state, {}, {}, model);
  expect_true(output.propeller_wrench.moment_body_Nm.z() > 0.0,
              "greater left thrust produces positive yaw moment");

  state.actuators = {-80.0, -80.0};
  output = uvd::evaluate_fossen_otter(state, {}, {}, model);
  expect_true(output.propeller_wrench.force_body_N.x() < 0.0,
              "reverse thrust produces negative surge");
}

void actuator_test() {
  const uvd::FossenOtterModel model =
      uvd::make_fossen_otter_model(uvd::FossenOtterParameters{});
  uvd::OtterState state = unloaded_equilibrium();
  const uvd::OtterCommand command{80.0, 80.0};
  constexpr double dt_s = 0.001;
  for (int step = 0; step < 100; ++step) {
    state = uvd::step_fossen_otter_rk4(state, command, {}, model, dt_s);
  }
  expect_near(state.actuators.left_propeller_speed_radps,
              80.0 * (1.0 - std::exp(-1.0)), 1e-8,
              "left shaft one-time-constant response");
  expect_near(state.actuators.right_propeller_speed_radps,
              80.0 * (1.0 - std::exp(-1.0)), 1e-8,
              "right shaft one-time-constant response");
}

void unloaded_equilibrium_test() {
  uvd::FossenOtterParameters parameters;
  parameters.payload_mass_kg = 0.0;
  parameters.payload_position_body_m = uvd::Vector3::Zero();
  const uvd::FossenOtterModel model = uvd::make_fossen_otter_model(parameters);
  const uvd::OtterModelOutput output =
      uvd::evaluate_fossen_otter({}, {}, {}, model);
  expect_true(output.valid, "unloaded equilibrium evaluation valid");
  expect_true(output.velocity_rate_body.norm() < 1e-12,
              "unloaded equilibrium acceleration");
}

void loaded_equilibrium_test() {
  const uvd::FossenOtterModel model =
      uvd::make_fossen_otter_model(uvd::FossenOtterParameters{});
  uvd::OtterState state;
  state.body.position_ned_m.z() = 0.037712608567758;
  state.body.q_body_to_ned = {0.999915394282329, 0.0, -0.013007854443166792,
                              0.0};
  const uvd::OtterModelOutput output =
      uvd::evaluate_fossen_otter(state, {}, {}, model);
  expect_true(output.valid, "loaded equilibrium evaluation valid");
  expect_true(wrench_vector(output.total_wrench).norm() < 1e-10,
              "loaded equilibrium total wrench");
  expect_true(output.velocity_rate_body.norm() < 1e-12,
              "loaded equilibrium acceleration");
}

void convergence_test() {
  const uvd::FossenOtterModel model =
      uvd::make_fossen_otter_model(uvd::FossenOtterParameters{});
  uvd::OtterState initial;
  initial.body.position_ned_m.z() = 0.04;
  const uvd::OtterCommand command{65.0, 52.0};
  const auto run = [&](double dt_s) {
    uvd::OtterState state = initial;
    const int steps = static_cast<int>(2.0 / dt_s);
    for (int i = 0; i < steps; ++i) {
      state = uvd::step_fossen_otter_rk4(state, command, {}, model, dt_s);
    }
    return state;
  };
  const uvd::OtterState coarse = run(0.02);
  const uvd::OtterState medium = run(0.01);
  const uvd::OtterState fine = run(0.005);
  const double coarse_error =
      (coarse.body.position_ned_m - medium.body.position_ned_m).norm();
  const double fine_error =
      (medium.body.position_ned_m - fine.body.position_ned_m).norm();
  expect_true(fine_error < coarse_error,
              "RK4 position error decreases with timestep");
  expect_true(coarse_error / fine_error > 8.0,
              "smooth Otter trajectory approaches fourth order");
}

void mss_derivative_test() {
  // Reference values were generated from MSS otter.m at commit d7c1e969856f
  // (source SHA-256
  // 66c144fea629cee4bea0ad55b62a1bc5e55b46e68ac8e94860ddc7f0aff12248). MSS
  // shifts its Euler position before computing kinematics, so this test
  // intentionally compares its physical load terms and nu_dot only.
  const auto evaluate = [](const uvd::Vector6& nu, const uvd::Vector6& eta,
                           double left_speed, double right_speed,
                           double current_speed, double current_direction) {
    uvd::OtterState state;
    state.body.position_ned_m = eta.head<3>();
    state.body.q_body_to_ned =
        uvd::quaternion_from_euler(eta(3), eta(4), eta(5));
    state.body.velocity_body_mps = nu.head<3>();
    state.body.omega_body_radps = nu.tail<3>();
    state.actuators = {left_speed, right_speed};
    uvd::OtterEnvironment environment;
    environment.current_ned_mps = {current_speed * std::cos(current_direction),
                                   current_speed * std::sin(current_direction),
                                   0.0};
    return uvd::evaluate_fossen_otter(
        state, {left_speed, right_speed}, environment,
        uvd::make_fossen_otter_model(uvd::FossenOtterParameters{}));
  };

  const uvd::Vector6 zero = uvd::Vector6::Zero();
  expect_vector_near(
      evaluate(zero, zero, 0.0, 0.0, 0.0, 0.0).velocity_rate_body,
      (uvd::Vector6() << 0.085044813257849, 0.0, 1.8499891983498953, 0.0,
       0.367227900182519, 0.0)
          .finished(),
      1e-10, "MSS loaded zero nu_dot");
  expect_vector_near(
      evaluate(zero, zero, 80.0, 80.0, 0.0, 0.0).velocity_rate_body,
      (uvd::Vector6() << 2.027612463824566, 0.0, 1.959526476485115, 0.0,
       1.574373414325756, 0.0)
          .finished(),
      1e-10, "MSS forward nu_dot");
  expect_vector_near(
      evaluate(zero, zero, -60.0, -60.0, 0.0, 0.0).velocity_rate_body,
      (uvd::Vector6() << -0.550552189061208, 0.0, 1.8141492287073333, 0.0,
       -0.027743193837553613, 0.0)
          .finished(),
      1e-10, "MSS reverse nu_dot");

  const uvd::Vector6 asymmetric_nu =
      (uvd::Vector6() << 1.2, -0.3, 0.15, 0.08, -0.04, 0.12).finished();
  const uvd::Vector6 asymmetric_eta =
      (uvd::Vector6() << 3.0, -2.0, 0.06, 0.1, -0.08, 0.7).finished();
  const uvd::OtterModelOutput asymmetric =
      evaluate(asymmetric_nu, asymmetric_eta, 70.0, 35.0, 0.0, 0.0);
  expect_vector_near(asymmetric.velocity_rate_body,
                     (uvd::Vector6() << 0.5547137891199286, 1.1299398093349393,
                      -1.0138336998666364, -6.823426029736206,
                      3.906914150844221, 0.7570892245011723)
                         .finished(),
                     1e-9, "MSS asymmetric nu_dot");
  expect_vector_near(wrench_vector(asymmetric.total_wrench),
                     (uvd::Vector6() << -29.854718818040443, 58.12689792977695,
                      -184.72724782983764, -96.16828869313763,
                      129.9769228375798, 26.162555358477544)
                         .finished(),
                     1e-9, "MSS asymmetric total load");

  const uvd::Vector6 current_nu =
      (uvd::Vector6() << 1.0, 0.2, -0.1, 0.05, 0.03, -0.08).finished();
  const uvd::Vector6 current_eta =
      (uvd::Vector6() << -1.0, 2.0, 0.04, -0.12, 0.09, -1.1).finished();
  const uvd::OtterModelOutput current =
      evaluate(current_nu, current_eta, 50.0, 45.0, 0.4, 0.3);
  expect_vector_near(current.velocity_rate_body,
                     (uvd::Vector6() << -3.155396238139782, -0.5312142260898989,
                      -2.103357406431958, 6.501133950186974,
                      -12.516592560960472, 0.3614136701085273)
                         .finished(),
                     1e-9, "MSS current nu_dot");
  expect_vector_near(wrench_vector(current.crossflow_wrench),
                     (uvd::Vector6() << 0.0, 8.446434769298262, 2.1784327429341,
                      0.0, -0.4639450178562457, 2.4023678255765972)
                         .finished(),
                     1e-9, "MSS current cross-flow load");
  expect_vector_near(wrench_vector(current.hydrostatic_wrench),
                     (uvd::Vector6() << 0.0, 0.0, -194.90251119348176,
                      105.83773537146402, -311.6449371259807, 0.0)
                         .finished(),
                     1e-9, "MSS current hydrostatic load");
}

}  // namespace

int main() {
  try {
    construction_test();
    direction_test();
    actuator_test();
    unloaded_equilibrium_test();
    loaded_equilibrium_test();
    mss_derivative_test();
    convergence_test();
    std::cout << "fossen_otter_test: all checks passed\n";
    return EXIT_SUCCESS;
  } catch (const std::exception& error) {
    std::cerr << "fossen_otter_test: " << error.what() << '\n';
    return EXIT_FAILURE;
  }
}
