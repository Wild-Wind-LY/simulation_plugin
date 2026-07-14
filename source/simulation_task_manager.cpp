#include "simulation_task_manager.hpp"

#include <stdexcept>
#include <utility>

nlohmann::json SimulationTaskManager::create(const nlohmann::json& data) {
  if (!data.is_object()) throw std::invalid_argument("task must be an object");
  const auto id = require_id(data);
  if (!data.contains("instance_id") || !data["instance_id"].is_string()) {
    throw std::invalid_argument("task missing 'instance_id'");
  }
  if (!data.contains("steps") || !data["steps"].is_array()) {
    throw std::invalid_argument("task missing 'steps' array");
  }

  nlohmann::json task = data;
  task["status"] = "created";
  task["last_result"] = nlohmann::json::object();

  std::lock_guard lock{mutex_};
  if (tasks_.find(id) != tasks_.end()) throw std::invalid_argument("task already exists: " + id);
  tasks_[id] = task;
  return task;
}

nlohmann::json SimulationTaskManager::remove(const nlohmann::json& data) {
  const auto id = require_id(data);
  std::lock_guard lock{mutex_};
  const auto erased = tasks_.erase(id);
  if (!erased) throw std::out_of_range("task not found: " + id);
  return {{"id", id}, {"removed", true}};
}

nlohmann::json SimulationTaskManager::list() const {
  std::lock_guard lock{mutex_};
  nlohmann::json out = nlohmann::json::array();
  for (const auto& [id, task] : tasks_) {
    out.push_back({
        {"id", id},
        {"instance_id", task.value("instance_id", "")},
        {"status", task.value("status", "")},
        {"step_count", task.value("steps", nlohmann::json::array()).size()},
    });
  }
  return out;
}

nlohmann::json SimulationTaskManager::info(const nlohmann::json& data) const {
  const auto id = require_id(data);
  std::lock_guard lock{mutex_};
  auto it = tasks_.find(id);
  if (it == tasks_.end()) throw std::out_of_range("task not found: " + id);
  return it->second;
}

nlohmann::json SimulationTaskManager::run(const nlohmann::json& data,
                                          SimulationInstanceManager& instances) {
  const auto id = require_id(data);
  nlohmann::json task;
  {
    std::lock_guard lock{mutex_};
    auto it = tasks_.find(id);
    if (it == tasks_.end()) throw std::out_of_range("task not found: " + id);
    task = it->second;
    it->second["status"] = "running";
  }

  nlohmann::json results = nlohmann::json::array();
  try {
    for (const auto& step : task.at("steps")) {
      results.push_back(execute_step(task, step, instances));
    }
  } catch (...) {
    std::lock_guard lock{mutex_};
    auto it = tasks_.find(id);
    if (it != tasks_.end()) it->second["status"] = "error";
    throw;
  }

  nlohmann::json result = {{"id", id}, {"status", "done"}, {"results", results}};
  {
    std::lock_guard lock{mutex_};
    auto it = tasks_.find(id);
    if (it != tasks_.end()) {
      it->second["status"] = "done";
      it->second["last_result"] = result;
    }
  }
  return result;
}

std::string SimulationTaskManager::require_id(const nlohmann::json& data) {
  const std::string id = data.value("id", "");
  if (id.empty()) throw std::invalid_argument("missing 'id'");
  return id;
}

nlohmann::json SimulationTaskManager::execute_step(const nlohmann::json& task,
                                                   const nlohmann::json& step,
                                                   SimulationInstanceManager& instances) {
  if (!step.is_object()) throw std::invalid_argument("task step must be an object");
  const std::string op = step.value("op", "");
  const std::string instance_id = step.value("instance_id", task.value("instance_id", ""));
  if (instance_id.empty()) throw std::invalid_argument("task step missing instance_id");

  if (op == "step") {
    return instances.step({{"id", instance_id}, {"count", step.value("count", 1)}});
  }
  if (op == "set_ctrl") {
    return instances.write_ctrl(
        {{"id", instance_id}, {"ctrl", step.value("ctrl", nlohmann::json::array())}});
  }
  if (op == "reset") {
    return instances.reset({{"id", instance_id}});
  }
  if (op == "pause") {
    return instances.pause({{"id", instance_id}});
  }
  if (op == "stop") {
    return instances.stop({{"id", instance_id}});
  }

  throw std::invalid_argument("unsupported task op: " + op);
}