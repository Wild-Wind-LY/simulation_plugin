#include "simulation_instance_manager.hpp"

#include <filesystem>
#include <stdexcept>
#include <utility>
#include <vector>

namespace {

  std::string normalize_model_path(const nlohmann::json& data) {
    const std::string model_path = data.value("model_path", "");
    if (model_path.empty()) {
      throw std::invalid_argument("missing 'model_path'");
    }

    std::filesystem::path path{model_path};
    if (path.is_relative()) {
      path = std::filesystem::current_path() / path;
    }
    return std::filesystem::weakly_canonical(path).string();
  }

}  // namespace

nlohmann::json SimulationInstanceManager::create(const nlohmann::json& data,
                                                 ModelPtr shared_model) {
  const std::string model_path = normalize_model_path(data);

  std::string id = data.value("id", "");
  {
    std::lock_guard lock{mutex_};
    if (id.empty()) {
      id = "sim-" + std::to_string(next_id_++);
    }
    if (instances_.count(id) || pending_.count(id)) {
      throw std::invalid_argument("instance already exists: " + id);
    }
    pending_.insert(id);
  }

  auto cleanup_reservation = [this, &id] {
    std::lock_guard lock{mutex_};
    pending_.erase(id);
  };

  auto result = SimulationInstance::create(id, model_path, std::move(shared_model));
  if (!result.instance) {
    cleanup_reservation();
    throw std::runtime_error(result.error);
  }

  auto state = result.instance->configure(data);
  auto instance = std::shared_ptr<SimulationInstance>(std::move(result.instance));
  {
    std::lock_guard lock{mutex_};
    instances_[id] = std::move(instance);
    pending_.erase(id);
  }
  return state;
}

nlohmann::json SimulationInstanceManager::recreate(const nlohmann::json& data,
                                                   ModelPtr shared_model) {
  const auto id = require_id(data);
  nlohmann::json previous_state = nlohmann::json::object();
  {
    std::shared_ptr<SimulationInstance> old_instance;
    {
      std::lock_guard lock{mutex_};
      auto it = instances_.find(id);
      if (it != instances_.end()) {
        old_instance = std::move(it->second);
        instances_.erase(it);
      }
    }
    if (old_instance) {
      previous_state = old_instance->stop();
    }
  }

  auto config = data;
  config["id"] = id;
  auto state = create(config, std::move(shared_model));
  return {{"id", id}, {"recreated", true}, {"previous_state", previous_state}, {"state", state}};
}
nlohmann::json SimulationInstanceManager::start(const nlohmann::json& data, PublishFn publisher) {
  const auto id = require_id(data);
  const double step_hz = data.value("step_hz", 100.0);
  const double publish_hz = data.value("publish_hz", 10.0);
  return find_or_throw(id)->start(step_hz, publish_hz, std::move(publisher));
}

nlohmann::json SimulationInstanceManager::pause(const nlohmann::json& data) {
  return find_or_throw(require_id(data))->pause();
}

nlohmann::json SimulationInstanceManager::stop(const nlohmann::json& data) {
  return find_or_throw(require_id(data))->stop();
}

nlohmann::json SimulationInstanceManager::step(const nlohmann::json& data) {
  const auto id = require_id(data);
  const int count = data.value("count", 1);
  if (count < 1 || count > 100000) {
    throw std::invalid_argument("'count' must be in [1, 100000]");
  }
  return find_or_throw(id)->step(count, data.value("include_state", true));
}

nlohmann::json SimulationInstanceManager::reset(const nlohmann::json& data) {
  return find_or_throw(require_id(data))->reset();
}

nlohmann::json SimulationInstanceManager::apply_runtime(const nlohmann::json& data) {
  const auto id = require_id(data);
  return find_or_throw(id)->apply_runtime(data);
}
nlohmann::json SimulationInstanceManager::state(const nlohmann::json& data) const {
  return find_or_throw(require_id(data))->state();
}

nlohmann::json SimulationInstanceManager::metadata(const nlohmann::json& data) const {
  return find_or_throw(require_id(data))->metadata();
}

nlohmann::json SimulationInstanceManager::inspect_model(const nlohmann::json& data) const {
  return find_or_throw(require_id(data))->inspect_model();
}

nlohmann::json SimulationInstanceManager::joint_state(const nlohmann::json& data) const {
  return find_or_throw(require_id(data))->joint_state();
}

nlohmann::json SimulationInstanceManager::sensor_state(const nlohmann::json& data) const {
  return find_or_throw(require_id(data))->sensor_state();
}

nlohmann::json SimulationInstanceManager::visual_model(const nlohmann::json& data) const {
  return find_or_throw(require_id(data))->visual_model(data.value("include_geometry", true));
}

nlohmann::json SimulationInstanceManager::write_ctrl(const nlohmann::json& data) {
  return find_or_throw(require_id(data))->write_ctrl(data);
}
nlohmann::json SimulationInstanceManager::write_qpos(const nlohmann::json& data) {
  return find_or_throw(require_id(data))->write_qpos(data);
}
nlohmann::json SimulationInstanceManager::write_equality(const nlohmann::json& data) {
  return find_or_throw(require_id(data))->write_equality(data);
}
nlohmann::json SimulationInstanceManager::list() const {
  nlohmann::json out = nlohmann::json::array();
  for (const auto& instance : snapshot_instances()) {
    out.push_back(instance->state());
  }
  return out;
}

nlohmann::json SimulationInstanceManager::destroy(const nlohmann::json& data) {
  const auto id = require_id(data);
  std::shared_ptr<SimulationInstance> instance;
  {
    std::lock_guard lock{mutex_};
    auto it = instances_.find(id);
    if (it == instances_.end()) throw std::out_of_range("instance not found: " + id);
    instance = std::move(it->second);
    instances_.erase(it);
  }

  auto final_state = instance->state();
  return {{"id", id}, {"destroyed", true}, {"final_state", std::move(final_state)}};
}

void SimulationInstanceManager::stop_all() noexcept {
  try {
    for (const auto& instance : snapshot_instances()) {
      instance->stop();
    }
  } catch (...) {
  }
}

std::shared_ptr<SimulationInstance> SimulationInstanceManager::find_or_throw(
    const std::string& id) const {
  std::lock_guard lock{mutex_};
  auto it = instances_.find(id);
  if (it == instances_.end()) throw std::out_of_range("instance not found: " + id);
  return it->second;
}

std::vector<std::shared_ptr<SimulationInstance>> SimulationInstanceManager::snapshot_instances()
    const {
  std::lock_guard lock{mutex_};
  std::vector<std::shared_ptr<SimulationInstance>> out;
  out.reserve(instances_.size());
  for (const auto& [id, instance] : instances_) {
    (void)id;
    out.push_back(instance);
  }
  return out;
}

std::string SimulationInstanceManager::require_id(const nlohmann::json& data) {
  const std::string id = data.value("id", "");
  if (id.empty()) {
    throw std::invalid_argument("missing 'id'");
  }
  return id;
}