#pragma once

#include <functional>
#include <memory>
#include <mutex>
#include <nlohmann/json.hpp>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "simulation_instance.hpp"

class SimulationInstanceManager {
public:
  using ModelPtr = SimulationInstance::ModelPtr;
  using PublishFn = SimulationInstance::PublishFn;

  nlohmann::json create(const nlohmann::json& data, ModelPtr shared_model = {});
  nlohmann::json recreate(const nlohmann::json& data, ModelPtr shared_model = {});
  nlohmann::json start(const nlohmann::json& data, PublishFn publisher);
  nlohmann::json pause(const nlohmann::json& data);
  nlohmann::json stop(const nlohmann::json& data);
  nlohmann::json step(const nlohmann::json& data);
  nlohmann::json reset(const nlohmann::json& data);
  nlohmann::json apply_runtime(const nlohmann::json& data);
  nlohmann::json state(const nlohmann::json& data) const;
  nlohmann::json metadata(const nlohmann::json& data) const;
  nlohmann::json inspect_model(const nlohmann::json& data) const;
  nlohmann::json joint_state(const nlohmann::json& data) const;
  nlohmann::json sensor_state(const nlohmann::json& data) const;
  nlohmann::json visual_model(const nlohmann::json& data) const;
  nlohmann::json write_ctrl(const nlohmann::json& data);
  nlohmann::json write_qpos(const nlohmann::json& data);
  nlohmann::json list() const;
  nlohmann::json destroy(const nlohmann::json& data);
  void stop_all() noexcept;

private:
  std::shared_ptr<SimulationInstance> find_or_throw(const std::string& id) const;
  static std::string require_id(const nlohmann::json& data);
  // Snapshot of all live (fully-constructed) instances, taken under the lock.
  std::vector<std::shared_ptr<SimulationInstance>> snapshot_instances() const;

  mutable std::mutex mutex_;
  std::unordered_map<std::string, std::shared_ptr<SimulationInstance>> instances_;
  // Ids reserved while SimulationInstance::create() (slow: loads the MuJoCo model) is
  // running outside the lock, so a concurrent create() with the same id is rejected
  // before that work starts. `instances_` itself never holds a null/partial entry.
  std::unordered_set<std::string> pending_;
  uint64_t next_id_{1};
};
