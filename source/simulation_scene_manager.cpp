#include "simulation_scene_manager.hpp"

#include <filesystem>
#include <fstream>
#include <stdexcept>

namespace {

  std::filesystem::path normalize_path(const std::filesystem::path& path) {
    if (path.is_relative()) {
      return std::filesystem::weakly_canonical(std::filesystem::current_path() / path);
    }
    return std::filesystem::weakly_canonical(path);
  }

}  // namespace

nlohmann::json SimulationSceneManager::load(const nlohmann::json& data) {
  const std::string path_text = data.value("path", "");
  if (path_text.empty()) throw std::invalid_argument("missing 'path'");

  const auto scene_path = normalize_path(path_text);
  std::ifstream input(scene_path);
  if (!input) throw std::runtime_error("failed to open scene file: " + scene_path.string());

  nlohmann::json scene = nlohmann::json::parse(input);
  std::string id = scene.value("id", "");
  if (id.empty()) id = scene_path.stem().string();
  scene["id"] = id;
  scene["path"] = scene_path.string();

  if (scene.contains("model_path") && !scene["model_path"].is_string())
    throw std::invalid_argument("scene 'model_path' must be a string");
  if (!scene.value("model_path", "").empty()) {
    std::filesystem::path model_path{scene.value("model_path", "")};
    if (model_path.is_relative()) model_path = scene_path.parent_path() / model_path;
    scene["model_path"] = normalize_path(model_path).string();
  }

  if (!scene.contains("defaults") || !scene["defaults"].is_object()) {
    scene["defaults"] = nlohmann::json::object();
  }

  std::lock_guard lock{mutex_};
  scenes_[id] = scene;
  return scene;
}

nlohmann::json SimulationSceneManager::update(const nlohmann::json& data) {
  nlohmann::json patch
      = data.contains("scene") && data["scene"].is_object() ? data.at("scene") : data;
  const auto id = require_id(patch.contains("id") ? patch : data);
  patch["id"] = id;

  std::lock_guard lock{mutex_};
  auto it = scenes_.find(id);
  if (it == scenes_.end()) throw std::out_of_range("scene not found: " + id);

  nlohmann::json scene = it->second;
  for (auto item = patch.begin(); item != patch.end(); ++item) {
    if (item.key() == "scene") continue;
    scene[item.key()] = item.value();
  }

  if (scene.contains("model_path") && scene["model_path"].is_string()) {
    std::filesystem::path model_path{scene.value("model_path", "")};
    if (model_path.is_relative()) {
      const std::filesystem::path scene_path{scene.value("path", "")};
      const auto base_dir
          = scene_path.empty() ? std::filesystem::current_path() : scene_path.parent_path();
      model_path = base_dir / model_path;
    }
    scene["model_path"] = normalize_path(model_path).string();
  }
  if (!scene.contains("defaults") || !scene["defaults"].is_object()) {
    scene["defaults"] = nlohmann::json::object();
  }

  it->second = scene;
  return scene;
}
nlohmann::json SimulationSceneManager::unload(const nlohmann::json& data) {
  const auto id = require_id(data);
  std::lock_guard lock{mutex_};
  const auto erased = scenes_.erase(id);
  if (!erased) throw std::out_of_range("scene not found: " + id);
  return {{"id", id}, {"unloaded", true}};
}

nlohmann::json SimulationSceneManager::list() const {
  std::lock_guard lock{mutex_};
  nlohmann::json out = nlohmann::json::array();
  for (const auto& [id, scene] : scenes_) {
    out.push_back({
        {"id", id},
        {"name", scene.value("name", id)},
        {"model_path", scene.value("model_path", "")},
        {"model_count", scene.value("models", nlohmann::json::array()).size()},
        {"path", scene.value("path", "")},
    });
  }
  return out;
}

nlohmann::json SimulationSceneManager::info(const nlohmann::json& data) const {
  const auto id = require_id(data);
  std::lock_guard lock{mutex_};
  auto it = scenes_.find(id);
  if (it == scenes_.end()) throw std::out_of_range("scene not found: " + id);
  return it->second;
}

bool SimulationSceneManager::references_model(const std::string& model_id,
                                              const std::string& version,
                                              const std::string& latest_version) const {
  std::lock_guard lock{mutex_};
  for (const auto& [scene_id, scene] : scenes_) {
    (void)scene_id;
    for (const auto& model : scene.value("models", nlohmann::json::array())) {
      if (model.value("model_id", "") != model_id) continue;
      const std::string referenced_version = model.value("version", "");
      if (!referenced_version.empty()) {
        if (referenced_version == version) return true;
        continue;
      }
      // Unpinned reference resolves to the latest version. Only block removal of
      // the version it currently resolves to; if the latest is unknown, stay
      // conservative and block any removal.
      if (latest_version.empty() || latest_version == version) return true;
    }
  }
  return false;
}

std::string SimulationSceneManager::require_id(const nlohmann::json& data) {
  const std::string id = data.value("id", "");
  if (id.empty()) throw std::invalid_argument("missing 'id'");
  return id;
}