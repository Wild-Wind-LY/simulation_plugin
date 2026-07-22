#include "simulation_scene_manager.hpp"

#include <filesystem>
#include <fstream>
#include <set>
#include <stdexcept>
#include <system_error>

#include "simulation_paths.hpp"
#include "simulation_scene_refs.hpp"

namespace {

  std::filesystem::path normalize_path(const std::filesystem::path& path) {
    if (path.is_relative()) {
      return std::filesystem::weakly_canonical(std::filesystem::current_path() / path);
    }
    return std::filesystem::weakly_canonical(path);
  }

  // Stamps schema_version on every in-memory scene and migrates the legacy
  // `defaults` (runtime initial state: step_hz/publish_hz/qpos/qvel/ctrl)
  // field forward to `initial_state`. Starting schema_version 2, `defaults` is
  // reserved for a future MJCF <default>/class tree, so a scene that predates
  // schema_version (or is explicitly < 2) has its `defaults` content copied
  // into `initial_state` once; the original `defaults` key is left in place
  // untouched (harmless, and lets an old client that still only writes
  // `defaults` be diagnosed rather than silently misinterpreted).
  void normalize_schema(nlohmann::json& scene) {
    const int version = scene.value("schema_version", 1);
    if (version < 2 && (!scene.contains("initial_state") || !scene["initial_state"].is_object())) {
      scene["initial_state"] = scene.value("defaults", nlohmann::json::object());
    }
    if (!scene.contains("initial_state") || !scene["initial_state"].is_object()) {
      scene["initial_state"] = nlohmann::json::object();
    }
    scene["schema_version"] = 2;
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
  normalize_schema(scene);

  std::lock_guard lock{mutex_};
  scenes_[id] = scene;
  return scene;
}

nlohmann::json SimulationSceneManager::create(const nlohmann::json& data) {
  const auto id = require_id(data);
  const bool replace = data.value("replace", false);

  nlohmann::json scene = data.contains("scene") && data["scene"].is_object()
                             ? data.at("scene")
                             : nlohmann::json::object();
  for (const char* key :
       {"name",     "physics",        "environment", "models",        "objects", "sensors",
        "contacts", "bodies",         "actuators",   "equality",      "tendon",  "keyframes",
        "assets",   "defaultClasses", "cameras",     "custom",        "flexes",  "skins",
        "plugins",  "initial_state",  "defaults",    "schema_version"}) {
    if (data.contains(key)) scene[key] = data.at(key);
  }
  scene["id"] = id;
  if (!scene.contains("models") || !scene["models"].is_array())
    scene["models"] = nlohmann::json::array();
  if (!scene.contains("objects") || !scene["objects"].is_array())
    scene["objects"] = nlohmann::json::array();
  if (!scene.contains("contacts") || !scene["contacts"].is_object())
    scene["contacts"] = nlohmann::json::object();
  if (!scene["contacts"].contains("excludes") || !scene["contacts"]["excludes"].is_array())
    scene["contacts"]["excludes"] = nlohmann::json::array();
  normalize_schema(scene);

  std::lock_guard lock{mutex_};
  if (!replace && scenes_.count(id) != 0)
    throw std::invalid_argument("scene already exists: " + id + " (pass replace:true)");
  scenes_[id] = scene;
  return scene;
}

nlohmann::json SimulationSceneManager::save(const nlohmann::json& data) {
  const auto id = require_id(data);
  std::string path_text = data.value("path", "");

  nlohmann::json scene;
  {
    std::lock_guard lock{mutex_};
    auto it = scenes_.find(id);
    if (it == scenes_.end()) throw std::out_of_range("scene not found: " + id);
    if (path_text.empty()) path_text = it->second.value("path", "");
    scene = it->second;
  }
  if (path_text.empty())
    path_text = (simulation_data_dir("scenes", "scenes") / (id + ".json")).string();

  const auto scene_path = normalize_path(path_text);
  std::error_code ec;
  std::filesystem::create_directories(scene_path.parent_path(), ec);

  // "path" is derived bookkeeping; the file itself must stay relocatable.
  nlohmann::json to_write = scene;
  to_write.erase("path");

  const auto tmp_path = scene_path.string() + ".tmp";
  {
    std::ofstream out(tmp_path, std::ios::trunc);
    if (!out) throw std::runtime_error("failed to open scene file for write: " + tmp_path);
    out << to_write.dump(2) << '\n';
    if (!out.good()) {
      out.close();
      std::filesystem::remove(tmp_path, ec);
      throw std::runtime_error("failed to write scene file: " + tmp_path);
    }
  }
  std::filesystem::rename(tmp_path, scene_path, ec);
  if (ec) {
    std::filesystem::remove(tmp_path, ec);
    throw std::runtime_error("failed to replace scene file: " + scene_path.string());
  }

  {
    std::lock_guard lock{mutex_};
    auto it = scenes_.find(id);
    if (it != scenes_.end()) it->second["path"] = scene_path.string();
  }
  return {{"id", id}, {"path", scene_path.string()}, {"saved", true}};
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

  normalize_schema(scene);

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
  // Build the in-memory portion under the lock (cheap: just a map walk), then
  // release it before the directory scan/JSON parsing below -- those do disk I/O
  // and previously held `mutex_` for their whole duration, blocking every other
  // scene op (create/save/update/unload/info/references_model all take the same
  // lock) until the scan finished. A scene loaded/unloaded concurrently mid-scan
  // just means this best-effort listing may lag by one call; it isn't used for
  // anything that needs strict consistency.
  nlohmann::json out = nlohmann::json::array();
  std::set<std::string> listed_ids;
  std::set<std::string> listed_paths;
  {
    std::lock_guard lock{mutex_};
    for (const auto& [id, scene] : scenes_) {
      out.push_back({
          {"id", id},
          {"name", scene.value("name", id)},
          {"model_count", scene.value("models", nlohmann::json::array()).size()},
          {"path", scene.value("path", "")},
          {"loaded", true},
      });
      listed_ids.insert(id);
      const std::string path = scene.value("path", "");
      if (!path.empty()) listed_paths.insert(path);
    }
  }
  // 默认场景目录里的 JSON 文件也列出来（loaded:false）：网关重启清空内存后，
  // 编辑器仍能发现保存过的场景并按 path 重新 scene.load。
  std::error_code ec;
  const auto dir = simulation_data_dir("scenes", "scenes");
  if (std::filesystem::is_directory(dir, ec)) {
    try {
      for (const auto& entry : std::filesystem::directory_iterator(dir, ec)) {
        if (!entry.is_regular_file(ec) || entry.path().extension() != ".json") continue;
        const std::string path = std::filesystem::weakly_canonical(entry.path(), ec).string();
        if (listed_paths.count(path)) continue;
        std::string id = entry.path().stem().string();
        std::string name = id;
        size_t model_count = 0;
        std::ifstream input(entry.path());
        if (input) {
          try {
            const auto scene = nlohmann::json::parse(input);
            if (!scene.value("id", "").empty()) id = scene["id"].get<std::string>();
            name = scene.value("name", id);
            model_count = scene.value("models", nlohmann::json::array()).size();
          } catch (...) {
            // 损坏的 JSON 仍按文件名列出，交给 scene.load 报具体错误
          }
        }
        if (listed_ids.count(id)) continue;
        out.push_back({
            {"id", id},
            {"name", name},
            {"model_count", model_count},
            {"path", path},
            {"loaded", false},
        });
        listed_ids.insert(id);
      }
    } catch (const std::exception&) {
      // 目录遍历失败不影响内存场景列表
    }
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
    bool found = false;
    simulation::walk_model_references(scene, [&](const nlohmann::json& model) {
      if (found || model.value("model_id", "") != model_id) return;
      const std::string referenced_version = model.value("version", "");
      if (!referenced_version.empty()) {
        if (referenced_version == version) found = true;
        return;
      }
      // Unpinned reference resolves to the latest version. Only block removal of
      // the version it currently resolves to; if the latest is unknown, stay
      // conservative and block any removal.
      if (latest_version.empty() || latest_version == version) found = true;
    });
    if (found) return true;
  }
  return false;
}

std::string SimulationSceneManager::require_id(const nlohmann::json& data) {
  const std::string id = data.value("id", "");
  if (id.empty()) throw std::invalid_argument("missing 'id'");
  return id;
}