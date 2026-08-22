#ifndef UNREALVEHICLEDYNAMICS_CLI_CONFIG_HPP_
#define UNREALVEHICLEDYNAMICS_CLI_CONFIG_HPP_

#include <filesystem>
#include <nlohmann/json.hpp>
#include <optional>
#include <string>
#include <variant>

#include "uvd/fixed_wing.hpp"
#include "uvd/marine.hpp"

namespace uvd::cli {

using Json = nlohmann::json;

// These are the Aerosonde run settings read from examples/run.json today, not
// a universal set of inputs for every vehicle. The explicit name keeps the
// aircraft assumptions visible while leaving the user's run file unchanged.
struct AerosondeRunConfig {
  std::filesystem::path path;
  std::filesystem::path aircraft_path;
  AerosondeParameters aircraft;
  double fixed_dt_s{};
  double origin_altitude_msl_m{};
  Vector3 wind_ned_mps = Vector3::Zero();
  RigidBodyState initial_state;
  AircraftCommand trim;
};

struct OtterRunConfig {
  std::filesystem::path path;
  std::filesystem::path plant_path;
  FossenOtterModel model;
  double fixed_dt_s{};
  OtterState initial_state;
  OtterEnvironment environment;
  OtterCommand command;
};

using RunConfig = std::variant<AerosondeRunConfig, OtterRunConfig>;

struct SimulationOptions {
  double duration_s{10.0};
  std::optional<double> dt_s;
  std::optional<std::filesystem::path> output;
};

[[nodiscard]] AerosondeRunConfig load_aerosonde_run(
    const std::filesystem::path& path);
[[nodiscard]] OtterRunConfig load_otter_run(const std::filesystem::path& path);
[[nodiscard]] RunConfig load_run(const std::filesystem::path& path);
[[nodiscard]] Json load_json(const std::filesystem::path& path);
void write_json(const std::filesystem::path& path, const Json& value);

[[nodiscard]] Vector3 vector3_from_json(const Json& value);
[[nodiscard]] RigidBodyState state_from_json(const Json& value);
[[nodiscard]] Json to_json(const Vector3& value);
[[nodiscard]] Json to_json(const Quaternion& value);
[[nodiscard]] Json to_json(const RigidBodyState& state);
[[nodiscard]] Json to_json(const AircraftCommand& command);
[[nodiscard]] Json to_json(const AircraftEffectorState& effectors);

[[nodiscard]] Json simulate(const AerosondeRunConfig& run,
                            const SimulationOptions& options);
[[nodiscard]] Json simulate(const OtterRunConfig& run,
                            const SimulationOptions& options);
[[nodiscard]] Json evaluate(const AerosondeRunConfig& run,
                            const std::optional<Json>& input);
[[nodiscard]] Json evaluate(const OtterRunConfig& run,
                            const std::optional<Json>& input);
[[nodiscard]] Json trim(const AerosondeRunConfig& run);
[[nodiscard]] Json linearize(const AerosondeRunConfig& run);

}  // namespace uvd::cli

#endif  // UNREALVEHICLEDYNAMICS_CLI_CONFIG_HPP_
