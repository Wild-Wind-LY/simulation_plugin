#pragma once

#include <mutex>
#include <nlohmann/json.hpp>
#include <string>
#include <unordered_map>

#include "simulation_instance_manager.hpp"

class SimulationTaskManager {
public:
  nlohmann::json create(const nlohmann::json& data);
  nlohmann::json remove(const nlohmann::json& data);
  nlohmann::json list() const;
  nlohmann::json info(const nlohmann::json& data) const;
  nlohmann::json run(const nlohmann::json& data, SimulationInstanceManager& instances);

private:
  static std::string require_id(const nlohmann::json& data);
  static nlohmann::json execute_step(const nlohmann::json& task, const nlohmann::json& step,
                                     SimulationInstanceManager& instances);

  mutable std::mutex mutex_;
  std::unordered_map<std::string, nlohmann::json> tasks_;
};