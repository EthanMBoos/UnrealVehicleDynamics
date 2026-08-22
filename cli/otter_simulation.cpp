#include <cmath>
#include <fstream>
#include <iomanip>
#include <stdexcept>

#include "config.hpp"

namespace uvd::cli {
namespace {

Json vector6_json(const Vector6& value) {
  Json result = Json::array();
  for (int i = 0; i < 6; ++i) {
    result.push_back(value(i));
  }
  return result;
}

Json wrench_json(const BodyWrench& wrench) {
  return {{"force_body_N", uvd::cli::to_json(wrench.force_body_N)},
          {"moment_body_Nm", uvd::cli::to_json(wrench.moment_body_Nm)}};
}

Json actuator_json(const OtterActuatorState& actuators) {
  return {
      {"left_propeller_speed_radps", actuators.left_propeller_speed_radps},
      {"right_propeller_speed_radps", actuators.right_propeller_speed_radps}};
}

Json command_json(const OtterCommand& command) {
  return {{"left_propeller_speed_command_radps",
           command.left_propeller_speed_command_radps},
          {"right_propeller_speed_command_radps",
           command.right_propeller_speed_command_radps}};
}

OtterState state_from_input(const Json& value, const OtterState& fallback) {
  if (!value.contains("state")) {
    return fallback;
  }
  const Json& state_json = value.at("state");
  OtterState state;
  state.body = state_from_json(state_json);
  const auto& shaft = state_json.at("propeller_speed_radps");
  state.actuators = {shaft.at(0), shaft.at(1)};
  return state;
}

OtterCommand command_from_input(const Json& value,
                                const OtterCommand& fallback) {
  if (!value.contains("command")) {
    return fallback;
  }
  const auto& command = value.at("command");
  return {.left_propeller_speed_command_radps =
              command.at("left_propeller_speed_radps"),
          .right_propeller_speed_command_radps =
              command.at("right_propeller_speed_radps")};
}

OtterEnvironment environment_from_input(const Json& value,
                                        const OtterEnvironment& fallback) {
  if (!value.contains("water")) {
    return fallback;
  }
  const auto& water = value.at("water");
  return {.water_surface_down_m = water.at("surface_down_m"),
          .current_ned_mps = vector3_from_json(water.at("current_ned_mps"))};
}

void write_csv_row(std::ostream& stream, double time_s,
                   const OtterState& state) {
  stream << time_s << ',' << state.body.position_ned_m.x() << ','
         << state.body.position_ned_m.y() << ','
         << state.body.position_ned_m.z() << ',' << state.body.q_body_to_ned.w()
         << ',' << state.body.q_body_to_ned.x() << ','
         << state.body.q_body_to_ned.y() << ',' << state.body.q_body_to_ned.z()
         << ',' << state.body.velocity_body_mps.x() << ','
         << state.body.velocity_body_mps.y() << ','
         << state.body.velocity_body_mps.z() << ','
         << state.body.omega_body_radps.x() << ','
         << state.body.omega_body_radps.y() << ','
         << state.body.omega_body_radps.z() << ','
         << state.actuators.left_propeller_speed_radps << ','
         << state.actuators.right_propeller_speed_radps << '\n';
}

}  // namespace

Json evaluate(const OtterRunConfig& run, const std::optional<Json>& input) {
  const Json empty = Json::object();
  const Json& value = input ? *input : empty;
  const OtterState state = state_from_input(value, run.initial_state);
  const OtterCommand command = command_from_input(value, run.command);
  const OtterEnvironment environment =
      environment_from_input(value, run.environment);
  if (std::abs(environment.current_ned_mps.z()) > 1e-12) {
    throw std::runtime_error("the Otter supports horizontal current only");
  }
  const OtterModelOutput model =
      evaluate_fossen_otter(state, command, environment, run.model);
  return {
      {"kind", "model_evaluation"},
      {"model_id", run.model.parameters.model_id},
      {"state", uvd::cli::to_json(state.body)},
      {"actuators", actuator_json(state.actuators)},
      {"command", command_json(command)},
      {"current_velocity_body", vector6_json(model.current_velocity_body)},
      {"relative_velocity_body", vector6_json(model.relative_velocity_body)},
      {"restoring_offset_down_roll_pitch",
       uvd::cli::to_json(model.restoring_offset_down_roll_pitch)},
      {"left_thrust_N", model.left_thrust_N},
      {"right_thrust_N", model.right_thrust_N},
      {"propeller_wrench", wrench_json(model.propeller_wrench)},
      {"linear_damping_wrench", wrench_json(model.linear_damping_wrench)},
      {"crossflow_wrench", wrench_json(model.crossflow_wrench)},
      {"coriolis_wrench", wrench_json(model.coriolis_wrench)},
      {"hydrostatic_wrench", wrench_json(model.hydrostatic_wrench)},
      {"total_wrench", wrench_json(model.total_wrench)},
      {"velocity_rate_body", vector6_json(model.velocity_rate_body)},
      {"propeller_speed_dot_radps2",
       Json::array({model.derivative.left_propeller_speed_dot_radps2,
                    model.derivative.right_propeller_speed_dot_radps2})},
      {"valid", model.valid}};
}

Json simulate(const OtterRunConfig& run, const SimulationOptions& options) {
  const double dt_s = options.dt_s.value_or(run.fixed_dt_s);
  if (!(dt_s > 0.0) || !(options.duration_s >= 0.0)) {
    throw std::runtime_error("duration must be nonnegative and dt positive");
  }
  const auto step_count =
      static_cast<std::uint64_t>(std::llround(options.duration_s / dt_s));
  OtterState state = run.initial_state;
  std::ofstream csv;
  if (options.output) {
    if (!options.output->parent_path().empty()) {
      std::filesystem::create_directories(options.output->parent_path());
    }
    csv.open(*options.output);
    if (!csv) {
      throw std::runtime_error("cannot write " + options.output->string());
    }
    csv << std::setprecision(17)
        << "time_s,north_m,east_m,down_m,qw,qx,qy,qz,u_mps,v_mps,w_mps,"
           "p_radps,q_radps,r_radps,left_propeller_radps,"
           "right_propeller_radps\n";
    write_csv_row(csv, 0.0, state);
  }
  for (std::uint64_t step = 0; step < step_count; ++step) {
    state = step_fossen_otter_rk4(state, run.command, run.environment,
                                  run.model, dt_s);
    if (!is_finite(state)) {
      throw std::runtime_error("simulation became nonfinite at step " +
                               std::to_string(step + 1));
    }
    if (csv) {
      write_csv_row(csv, static_cast<double>(step + 1) * dt_s, state);
    }
  }
  Json result = {{"kind", "headless_simulation"},
                 {"model_id", run.model.parameters.model_id},
                 {"integrator", "rk4"},
                 {"dt_s", dt_s},
                 {"steps", step_count},
                 {"duration_s", static_cast<double>(step_count) * dt_s},
                 {"command", command_json(run.command)},
                 {"final_state", uvd::cli::to_json(state.body)},
                 {"final_actuators", actuator_json(state.actuators)}};
  if (options.output) {
    result["trajectory_csv"] = options.output->string();
  }
  return result;
}

}  // namespace uvd::cli
