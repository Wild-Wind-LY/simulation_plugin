#include "simulation_compiler.hpp"
#include "simulation_hash.hpp"
#include "simulation_mujoco_utils.hpp"

#include <algorithm>
#include <cctype>
#include <fstream>
#include <functional>
#include <iomanip>
#include <optional>
#include <set>
#include <sstream>
#include <stdexcept>
#include <system_error>
#include <utility>
#include <vector>

namespace {

  bool has_array_items(const nlohmann::json& value, const char* key) {
    return value.contains(key) && value[key].is_array() && !value[key].empty();
  }

  bool has_structural_items(const nlohmann::json& scene) {
    return scene.value("model_path", "").empty() || has_array_items(scene, "models")
           || has_array_items(scene, "objects") || has_array_items(scene, "sensors")
           || (scene.contains("contacts") && !scene["contacts"].empty());
  }

  bool is_sensor_type(const std::string& type) {
    return type == "sensor.force" || type == "sensor.torque";
  }

  bool is_model_include_type(const std::string& type) { return type == "model.include"; }

  void insert_before_required(std::string& xml, const std::string& marker,
                              const std::string& insertion, const std::string& missing_message) {
    const auto pos = xml.rfind(marker);
    if (pos == std::string::npos) throw std::runtime_error(missing_message);
    xml.insert(pos, insertion);
  }

  void append_section_xml(std::string& xml, const std::string& section,
                          const std::string& content) {
    if (content.empty()) return;
    const std::string close = "</" + section + ">";
    const auto position = xml.rfind(close);
    if (position != std::string::npos) {
      xml.insert(position, content);
      return;
    }
    std::ostringstream block;
    block << "  <" << section << ">\n" << content << "  </" << section << ">\n";
    insert_before_required(xml, "</mujoco>", block.str(), "base model missing </mujoco>");
  }

  void add_issue(nlohmann::json& issues, const std::string& path, const std::string& message) {
    issues.push_back({{"path", path}, {"message", message}});
  }

  nlohmann::json structural_signature(const nlohmann::json& scene) {
    return {
        {"model_path", scene.value("model_path", "")},
        {"models", scene.value("models", nlohmann::json::array())},
        {"objects", scene.value("objects", nlohmann::json::array())},
        {"sensors", scene.value("sensors", nlohmann::json::array())},
        {"contacts", scene.value("contacts", nlohmann::json::object())},
    };
  }

  void append_change(nlohmann::json& changes, const std::string& path, const std::string& type) {
    changes.push_back({{"path", path}, {"type", type}});
  }
}  // namespace

SimulationCompiler::SimulationCompiler(std::filesystem::path output_dir, size_t max_cached_models)
    : output_dir_(std::move(output_dir)),
      max_cached_models_(max_cached_models == 0 ? 1 : max_cached_models) {}

nlohmann::json SimulationCompiler::validate_scene(const nlohmann::json& scene) const {
  nlohmann::json errors = nlohmann::json::array();
  nlohmann::json warnings = nlohmann::json::array();

  if (!scene.is_object()) {
    add_issue(errors, "$", "scene must be an object");
    return {{"valid", false}, {"errors", errors}, {"warnings", warnings}};
  }

  if (!scene.contains("id") || !scene["id"].is_string() || scene.value("id", "").empty()) {
    add_issue(errors, "$.id", "scene missing string id");
  }
  if (scene.contains("model_path") && !scene["model_path"].is_string()) {
    add_issue(errors, "$.model_path", "model_path must be a string");
  } else if (!scene.value("model_path", "").empty()) {
    try {
      const auto model_path = resolve_scene_path(scene, scene.value("model_path", ""));
      if (!std::filesystem::exists(model_path))
        add_issue(errors, "$.model_path", "model_path does not exist: " + model_path.string());
    } catch (const std::exception& e) {
      add_issue(errors, "$.model_path", e.what());
    }
  }

  std::set<std::string> ids;
  if (scene.contains("models")) {
    if (!scene["models"].is_array()) {
      add_issue(errors, "$.models", "models must be an array");
    } else {
      for (size_t i = 0; i < scene["models"].size(); ++i) {
        const auto& model = scene["models"][i];
        const std::string path = "$.models[" + std::to_string(i) + "]";
        if (!model.is_object()) {
          add_issue(errors, path, "model must be an object");
          continue;
        }
        const std::string id = model.value("id", "");
        const std::string source = model.value("source", "");
        if (id.empty()) add_issue(errors, path + ".id", "model missing id");
        else if (!ids.insert(id).second)
          add_issue(errors, path + ".id", "duplicate scene id: " + id);
        if (source.empty()) {
          add_issue(errors, path + ".source", "model missing source");
        } else {
          try {
            const auto source_path = resolve_scene_path(scene, source);
            if (!std::filesystem::exists(source_path))
              add_issue(errors, path + ".source", "model source does not exist: " + source_path.string());
            if (source_path.extension() == ".urdf")
              add_issue(errors, path + ".source",
                        "attached models must be MJCF; URDF is only supported as a base model");
          } catch (const std::exception& e) {
            add_issue(errors, path + ".source", e.what());
          }
        }
      }
    }
  }
  if (scene.contains("objects")) {
    if (!scene["objects"].is_array()) {
      add_issue(errors, "$.objects", "objects must be an array");
    } else {
      for (size_t i = 0; i < scene["objects"].size(); ++i) {
        const auto& object = scene["objects"][i];
        const std::string path = "$.objects[" + std::to_string(i) + "]";
        if (!object.is_object()) {
          add_issue(errors, path, "object must be an object");
          continue;
        }
        const std::string id = object.value("id", "");
        const std::string type = object.value("type", "");
        if (id.empty()) {
          add_issue(errors, path + ".id", "object missing id");
        } else if (!ids.insert(id).second) {
          add_issue(errors, path + ".id", "duplicate scene object id: " + id);
        }
        if (type.empty()) add_issue(errors, path + ".type", "object missing type");
        if (type == "model.include") {
          if (!object.contains("source") || !object["source"].is_string()) {
            add_issue(errors, path + ".source", "model.include missing source");
          } else {
            try {
              const auto source = resolve_scene_path(scene, object.value("source", ""));
              if (!std::filesystem::exists(source)) {
                add_issue(errors, path + ".source",
                          "include source does not exist: " + source.string());
              }
            } catch (const std::exception& e) {
              add_issue(errors, path + ".source", e.what());
            }
          }
        } else if (is_sensor_type(type)) {
          if (object.value("site", "").empty())
            add_issue(errors, path + ".site", "sensor missing site");
        } else if (type != "obstacle.box" && type != "obstacle.sphere"
                   && type != "obstacle.cylinder") {
          add_issue(errors, path + ".type", "unsupported object type: " + type);
        }
      }
    }
  }

  if (scene.contains("sensors")) {
    if (!scene["sensors"].is_array()) {
      add_issue(errors, "$.sensors", "sensors must be an array");
    } else {
      for (size_t i = 0; i < scene["sensors"].size(); ++i) {
        const auto& sensor = scene["sensors"][i];
        const std::string path = "$.sensors[" + std::to_string(i) + "]";
        if (!sensor.is_object()) {
          add_issue(errors, path, "sensor must be an object");
          continue;
        }
        const std::string id = sensor.value("id", "");
        if (id.empty())
          add_issue(errors, path + ".id", "sensor missing id");
        else if (!ids.insert(id).second)
          add_issue(errors, path + ".id", "duplicate scene object id: " + id);
        if (!is_sensor_type(sensor.value("type", ""))) {
          add_issue(errors, path + ".type", "unsupported sensor type: " + sensor.value("type", ""));
        }
        if (sensor.value("site", "").empty())
          add_issue(errors, path + ".site", "sensor missing site");
      }
    }
  }

  if (scene.contains("contacts")) {
    const auto& contacts = scene["contacts"];
    if (!contacts.is_object() || !contacts.value("excludes", nlohmann::json::array()).is_array()) {
      add_issue(errors, "$.contacts.excludes", "contacts.excludes must be an array");
    } else {
      const auto excludes = contacts.value("excludes", nlohmann::json::array());
      for (size_t i = 0; i < excludes.size(); ++i) {
        const auto& item = excludes[i];
        const std::string path = "$.contacts.excludes[" + std::to_string(i) + "]";
        if (!item.is_object() || item.value("body1", "").empty()
            || item.value("body2", "").empty())
          add_issue(errors, path, "contact exclude requires body1 and body2");
      }
    }
  }

  if (!scene.contains("defaults")) {
    add_issue(warnings, "$.defaults", "scene has no defaults; instance will use runtime defaults");
  }

  return {{"valid", errors.empty()}, {"errors", errors}, {"warnings", warnings}};
}

nlohmann::json SimulationCompiler::diff_scenes(const nlohmann::json& old_scene,
                                               const nlohmann::json& new_scene) const {
  nlohmann::json structural_changes = nlohmann::json::array();
  nlohmann::json runtime_changes = nlohmann::json::array();

  if (old_scene.value("model_path", "") != new_scene.value("model_path", "")) {
    append_change(structural_changes, "$.model_path", "model_path");
  }
  if (old_scene.value("models", nlohmann::json::array())
      != new_scene.value("models", nlohmann::json::array())) {
    append_change(structural_changes, "$.models", "scene_models");
  }
  if (old_scene.value("objects", nlohmann::json::array())
      != new_scene.value("objects", nlohmann::json::array())) {
    append_change(structural_changes, "$.objects", "scene_objects");
  }
  if (old_scene.value("sensors", nlohmann::json::array())
      != new_scene.value("sensors", nlohmann::json::array())) {
    append_change(structural_changes, "$.sensors", "scene_sensors");
  }
  if (old_scene.value("contacts", nlohmann::json::object())
      != new_scene.value("contacts", nlohmann::json::object())) {
    append_change(structural_changes, "$.contacts", "contact_rules");
  }
  if (old_scene.value("defaults", nlohmann::json::object())
      != new_scene.value("defaults", nlohmann::json::object())) {
    append_change(runtime_changes, "$.defaults", "initial_state_defaults");
  }

  const bool structural = !structural_changes.empty();
  const bool runtime = !runtime_changes.empty();
  return {
      {"structural_changed", structural},
      {"runtime_changed", runtime},
      {"requires_recreate", structural},
      {"can_apply_runtime", runtime && !structural},
      {"structural_changes", structural_changes},
      {"runtime_changes", runtime_changes},
      {"old_signature", structural_signature(old_scene)},
      {"new_signature", structural_signature(new_scene)},
  };
}

nlohmann::json SimulationCompiler::compile_scene(const nlohmann::json& scene) const {
  const auto validation = validate_scene(scene);
  if (!validation.value("valid", false)) {
    throw std::invalid_argument("scene validation failed: " + validation["errors"].dump());
  }

  const bool generated = has_structural_items(scene);
  const auto compiled_model_path = generated
                                       ? write_compiled_mjcf(scene)
                                       : resolve_scene_path(scene, scene.value("model_path", ""));
  const std::string compiled_model_id = structural_model_id(scene);
  ModelPtr model;
  {
    std::lock_guard lock{model_mutex_};
    auto it = models_.find(compiled_model_id);
    if (it != models_.end()) model = touch_and_reclaim_locked(compiled_model_id, it->second.model);
  }
  if (!model) {
    auto loaded = load_model(compiled_model_path);
    std::lock_guard lock{model_mutex_};
    model = touch_and_reclaim_locked(compiled_model_id, std::move(loaded));
  }

  nlohmann::json compiled;
  compiled["scene_id"] = scene.value("id", "");
  compiled["compiled_model_id"] = compiled_model_id;
  compiled["backend"] = "mujoco";
  compiled["model_path"] = compiled_model_path.string();
  compiled["generated"] = generated;
  compiled["initial_state"] = scene.value("defaults", nlohmann::json::object());
  compiled["sizes"] = {{"nq", model->nq}, {"nv", model->nv}, {"nu", model->nu},
                       {"nbody", model->nbody}, {"nsensor", model->nsensor}};
  compiled["mapping"] = {
      {"actuators", nlohmann::json::object()},
      {"sensors", nlohmann::json::object()},
      {"bodies", nlohmann::json::object()},
      {"models", nlohmann::json::object()},
  };

  if (scene.contains("models") && scene["models"].is_array()) {
    for (const auto& asset : scene["models"]) {
      const std::string id = asset.value("id", "");
      if (!id.empty()) {
        compiled["mapping"]["models"][id] = {
            {"source", resolve_scene_path(scene, asset.value("source", "")).string()},
            {"prefix", asset.value("prefix", id + "_")},
            {"body", asset.value("body", "")},
        };
      }
    }
  }

  if (scene.contains("objects") && scene["objects"].is_array()) {
    for (const auto& object : scene["objects"]) {
      const std::string id = object.value("id", "");
      const std::string type = object.value("type", "");
      if (id.empty()) continue;
      if (is_sensor_type(type)) {
        compiled["mapping"]["sensors"][id] = {
            {"type", type},
            {"site", object.value("site", "")},
        };
      } else if (is_model_include_type(type)) {
        compiled["mapping"]["models"][id] = {
            {"type", type},
            {"source", resolve_scene_path(scene, object.value("source", "")).string()},
            {"body", id},
        };
      } else {
        compiled["mapping"]["bodies"][id] = id;
      }
    }
  }

  if (scene.contains("sensors") && scene["sensors"].is_array()) {
    for (const auto& sensor : scene["sensors"]) {
      const std::string id = sensor.value("id", "");
      if (!id.empty()) {
        compiled["mapping"]["sensors"][id] = {
            {"type", sensor.value("type", "")},
            {"site", sensor.value("site", "")},
        };
      }
    }
  }

  return compiled;
}

nlohmann::json SimulationCompiler::build_visual_scene(const nlohmann::json& scene) const {
  const auto validation = validate_scene(scene);
  if (!validation.value("valid", false)) {
    throw std::invalid_argument("scene validation failed: " + validation["errors"].dump());
  }

  nlohmann::json visual = {
      {"scene_id", scene.value("id", "")},
      {"model_path", scene.value("model_path", "")},
      {"backend", "threejs"},
      {"units", "m"},
      {"coordinate_system", "mujoco_xyz"},
      {"items", nlohmann::json::array()},
  };

  const auto append_item = [&visual](nlohmann::json item) {
    if (!item.contains("pos")) item["pos"] = nlohmann::json::array({0.0, 0.0, 0.0});
    if (!item.contains("quat")) item["quat"] = nlohmann::json::array({1.0, 0.0, 0.0, 0.0});
    visual["items"].push_back(std::move(item));
  };

  if (scene.contains("models") && scene["models"].is_array()) {
    for (const auto& model : scene["models"]) {
      const std::string id = model.value("id", "");
      if (id.empty()) continue;
      append_item({{"id", id},
                   {"source", "scene.models"},
                   {"kind", "model_ref"},
                   {"shape", "model_ref"},
                   {"model_path", resolve_scene_path(scene, model.value("source", "")).string()},
                   {"pos", numeric_array(model.value("pos", nlohmann::json::array()), {0, 0, 0}, 3)},
                   {"quat", numeric_array(model.value("quat", nlohmann::json::array()), {1, 0, 0, 0}, 4)},
                   {"size", nlohmann::json::array({0.18, 0.18, 0.18})}});
    }
  }

  if (scene.contains("objects") && scene["objects"].is_array()) {
    for (const auto& object : scene["objects"]) {
      const std::string id = object.value("id", "");
      const std::string type = object.value("type", "");
      if (id.empty()) continue;

      if (type == "obstacle.box") {
        append_item(
            {{"id", id},
             {"source", "scene.objects"},
             {"kind", "primitive"},
             {"shape", "box"},
             {"pos", numeric_array(object.value("pos", nlohmann::json::array()), {0, 0, 0}, 3)},
             {"quat",
              numeric_array(object.value("quat", nlohmann::json::array()), {1, 0, 0, 0}, 4)},
             {"size",
              numeric_array(object.value("size", nlohmann::json::array()), {0.1, 0.1, 0.1}, 3)},
             {"rgba", numeric_array(object.value("rgba", nlohmann::json::array()),
                                    {0.7, 0.2, 0.2, 1}, 4)}});
      } else if (type == "obstacle.sphere") {
        append_item(
            {{"id", id},
             {"source", "scene.objects"},
             {"kind", "primitive"},
             {"shape", "sphere"},
             {"pos", numeric_array(object.value("pos", nlohmann::json::array()), {0, 0, 0}, 3)},
             {"quat",
              numeric_array(object.value("quat", nlohmann::json::array()), {1, 0, 0, 0}, 4)},
             {"size", numeric_array(object.value("size", nlohmann::json::array()), {0.1}, 1)},
             {"rgba", numeric_array(object.value("rgba", nlohmann::json::array()),
                                    {0.2, 0.45, 0.9, 1}, 4)}});
      } else if (type == "obstacle.cylinder") {
        append_item(
            {{"id", id},
             {"source", "scene.objects"},
             {"kind", "primitive"},
             {"shape", "cylinder"},
             {"pos", numeric_array(object.value("pos", nlohmann::json::array()), {0, 0, 0}, 3)},
             {"quat",
              numeric_array(object.value("quat", nlohmann::json::array()), {1, 0, 0, 0}, 4)},
             {"size", numeric_array(object.value("size", nlohmann::json::array()), {0.1, 0.1}, 2)},
             {"rgba", numeric_array(object.value("rgba", nlohmann::json::array()),
                                    {0.1, 0.55, 0.35, 1}, 4)}});
      } else if (is_model_include_type(type)) {
        append_item(
            {{"id", id},
             {"source", "scene.objects"},
             {"kind", "model_ref"},
             {"shape", "model_ref"},
             {"model_path", resolve_scene_path(scene, object.value("source", "")).string()},
             {"pos", numeric_array(object.value("pos", nlohmann::json::array()), {0, 0, 0}, 3)},
             {"quat",
              numeric_array(object.value("quat", nlohmann::json::array()), {1, 0, 0, 0}, 4)},
             {"size", nlohmann::json::array({0.12, 0.12, 0.12})},
             {"rgba", nlohmann::json::array({0.49, 0.83, 1.0, 1.0})}});
      } else if (is_sensor_type(type)) {
        append_item(
            {{"id", id},
             {"source", "scene.objects"},
             {"kind", "sensor"},
             {"shape", "sensor"},
             {"sensor_type", type},
             {"site", object.value("site", "")},
             {"pos", numeric_array(object.value("pos", nlohmann::json::array()), {0, 0, 0}, 3)},
             {"rgba", nlohmann::json::array({0.98, 0.75, 0.14, 1.0})}});
      }
    }
  }

  if (scene.contains("sensors") && scene["sensors"].is_array()) {
    int index = 0;
    for (const auto& sensor : scene["sensors"]) {
      const std::string id = sensor.value("id", "");
      if (id.empty()) continue;
      append_item({{"id", id},
                   {"source", "scene.sensors"},
                   {"kind", "sensor"},
                   {"shape", "sensor"},
                   {"sensor_type", sensor.value("type", "")},
                   {"site", sensor.value("site", "")},
                   {"pos", numeric_array(sensor.value("pos", nlohmann::json::array()),
                                         {-0.42 + index * 0.12, 0, 0.18}, 3)},
                   {"rgba", nlohmann::json::array({0.98, 0.75, 0.14, 1.0})}});
      ++index;
    }
  }

  return visual;
}
nlohmann::json SimulationCompiler::build_instance_config(
    const nlohmann::json& request, const nlohmann::json& compiled_scene) const {
  if (!request.is_object()) throw std::invalid_argument("instance create data must be an object");

  const std::string scene_id = request.value("scene_id", "");
  if (scene_id.empty()) return request;
  if (!compiled_scene.is_object()) throw std::invalid_argument("compiled scene must be an object");
  if (compiled_scene.value("scene_id", "") != scene_id) {
    throw std::invalid_argument("compiled scene id mismatch");
  }

  nlohmann::json config = compiled_scene.value("initial_state", nlohmann::json::object());
  for (auto it = request.begin(); it != request.end(); ++it) {
    config[it.key()] = it.value();
  }
  config["scene_id"] = scene_id;
  config["compiled_model_id"] = compiled_scene.value("compiled_model_id", "");
  config["model_path"] = compiled_scene.value("model_path", "");
  return config;
}

SimulationCompiler::ModelPtr SimulationCompiler::get_compiled_model(
    const std::string& compiled_model_id) const {
  std::lock_guard lock{model_mutex_};
  auto it = models_.find(compiled_model_id);
  if (it == models_.end()) throw std::out_of_range("compiled model not found: " + compiled_model_id);
  return touch_and_reclaim_locked(compiled_model_id, it->second.model);
}

std::pair<std::string, SimulationCompiler::ModelPtr> SimulationCompiler::get_or_load_model(
    const std::string& model_path) const {
  if (model_path.empty()) throw std::invalid_argument("missing 'model_path'");
  std::filesystem::path path{model_path};
  if (path.is_relative()) path = std::filesystem::current_path() / path;
  path = std::filesystem::weakly_canonical(path);
  if (!std::filesystem::exists(path))
    throw std::invalid_argument("model path does not exist: " + path.string());

  std::error_code ec;
  const auto timestamp = std::filesystem::last_write_time(path, ec);
  const std::string input = path.string() + (ec ? "" : std::to_string(timestamp.time_since_epoch().count()));
  const std::string model_id = "direct-" + simulation::sha256_string(input).substr(0, 32);

  {
    std::lock_guard lock{model_mutex_};
    auto it = models_.find(model_id);
    if (it != models_.end()) return {model_id, touch_and_reclaim_locked(model_id, it->second.model)};
  }
  auto loaded = load_model(path);
  std::lock_guard lock{model_mutex_};
  auto model = touch_and_reclaim_locked(model_id, std::move(loaded));
  return {model_id, std::move(model)};
}

std::string SimulationCompiler::structural_model_id(const nlohmann::json& scene) {
  std::ostringstream input;
  input << structural_signature(scene).dump();
  const auto append_timestamp = [&input](const std::filesystem::path& path) {
    std::error_code ec;
    const auto timestamp = std::filesystem::last_write_time(path, ec);
    if (!ec) input << ':' << timestamp.time_since_epoch().count();
  };
  if (!scene.value("model_path", "").empty())
    append_timestamp(resolve_scene_path(scene, scene.value("model_path", "")));
  for (const auto& model : scene.value("models", nlohmann::json::array()))
    append_timestamp(resolve_scene_path(scene, model.value("source", "")));

  return scene.value("id", "scene") + '-' + simulation::sha256_string(input.str()).substr(0, 32);
}

SimulationCompiler::ModelPtr SimulationCompiler::touch_and_reclaim_locked(
    const std::string& model_id, ModelPtr model) const {
  auto& entry = models_[model_id];
  entry.model = std::move(model);
  entry.last_used = ++access_tick_;
  ModelPtr result = entry.model;

  // Evict least-recently-used entries that no live instance still references
  // (use_count of 1 means only the cache holds the model).
  while (models_.size() > max_cached_models_) {
    auto victim = models_.end();
    for (auto it = models_.begin(); it != models_.end(); ++it) {
      if (it->second.model.use_count() != 1) continue;
      if (victim == models_.end() || it->second.last_used < victim->second.last_used) victim = it;
    }
    if (victim == models_.end()) break;  // everything is still referenced
    models_.erase(victim);
  }
  return result;
}

size_t SimulationCompiler::cached_model_count() const {
  std::lock_guard lock{model_mutex_};
  return models_.size();
}

size_t SimulationCompiler::prune_unused_models() const {
  std::lock_guard lock{model_mutex_};
  const size_t before = models_.size();
  for (auto it = models_.begin(); it != models_.end();) {
    if (it->second.model.use_count() == 1)
      it = models_.erase(it);
    else
      ++it;
  }
  return before - models_.size();
}

SimulationCompiler::ModelPtr SimulationCompiler::load_model(const std::filesystem::path& path) {
  char error[2048] = {};
  std::lock_guard xml_lock{simulation_mujoco_xml_mutex()};
  mjModel* model = mj_loadXML(path.string().c_str(), nullptr, error, sizeof(error));
  if (!model) throw std::runtime_error(error[0] ? error : "failed to compile MuJoCo model");
  return ModelPtr(model, mj_deleteModel);
}

std::filesystem::path SimulationCompiler::default_output_dir() {
  return std::filesystem::current_path() / "build" / "generated_scenes";
}

std::filesystem::path SimulationCompiler::default_export_dir() {
  return std::filesystem::current_path() / "build" / "scene_exports";
}

std::filesystem::path SimulationCompiler::resolve_scene_path(const nlohmann::json& scene,
                                                             const std::string& path_text) {
  if (path_text.empty()) throw std::invalid_argument("missing source path");

  std::filesystem::path path{path_text};
  if (path.is_relative()) {
    const std::filesystem::path scene_path{scene.value("path", "")};
    const auto base_dir
        = scene_path.empty() ? std::filesystem::current_path() : scene_path.parent_path();
    path = base_dir / path;
  }
  return std::filesystem::weakly_canonical(path);
}

std::string SimulationCompiler::xml_escape(const std::string& value) {
  std::string out;
  out.reserve(value.size());
  for (const char c : value) {
    switch (c) {
      case '&':
        out += "&amp;";
        break;
      case '"':
        out += "&quot;";
        break;
      case '<':
        out += "&lt;";
        break;
      case '>':
        out += "&gt;";
        break;
      default:
        out.push_back(c);
        break;
    }
  }
  return out;
}

std::string SimulationCompiler::numeric_list(const nlohmann::json& values,
                                             const std::string& fallback, int expected_size) {
  if (!values.is_array() || values.empty()) return fallback;
  if (expected_size > 0 && static_cast<int>(values.size()) != expected_size) {
    throw std::invalid_argument("numeric list size mismatch");
  }

  std::ostringstream out;
  for (size_t i = 0; i < values.size(); ++i) {
    if (i > 0) out << ' ';
    out << values.at(i).get<double>();
  }
  return out.str();
}

nlohmann::json SimulationCompiler::numeric_array(const nlohmann::json& values,
                                                 std::initializer_list<double> fallback,
                                                 int expected_size) {
  if (values.is_array() && !values.empty()) {
    if (expected_size > 0 && static_cast<int>(values.size()) != expected_size) {
      throw std::invalid_argument("numeric array size mismatch");
    }
    nlohmann::json out = nlohmann::json::array();
    for (const auto& value : values) out.push_back(value.get<double>());
    return out;
  }

  nlohmann::json out = nlohmann::json::array();
  for (const double value : fallback) out.push_back(value);
  return out;
}
std::string SimulationCompiler::compile_primitive_object(const nlohmann::json& object) {
  if (!object.is_object()) throw std::invalid_argument("scene object must be an object");

  const std::string id = object.value("id", "");
  const std::string type = object.value("type", "");
  if (id.empty()) throw std::invalid_argument("scene object missing 'id'");

  std::string geom_type;
  std::string size_default;
  int size_count = 0;
  if (type == "obstacle.box") {
    geom_type = "box";
    size_default = "0.1 0.1 0.1";
    size_count = 3;
  } else if (type == "obstacle.sphere") {
    geom_type = "sphere";
    size_default = "0.1";
    size_count = 1;
  } else if (type == "obstacle.cylinder") {
    geom_type = "cylinder";
    size_default = "0.1 0.1";
    size_count = 2;
  } else {
    throw std::invalid_argument("unsupported scene object type: " + type);
  }

  const std::string pos = numeric_list(object.value("pos", nlohmann::json::array()), "0 0 0", 3);
  const std::string quat
      = numeric_list(object.value("quat", nlohmann::json::array()), "1 0 0 0", 4);
  const std::string size
      = numeric_list(object.value("size", nlohmann::json::array()), size_default, size_count);
  const double mass = object.value("mass", 0.0);
  const std::string rgba
      = numeric_list(object.value("rgba", nlohmann::json::array()), "0.7 0.2 0.2 1", 4);

  std::ostringstream xml;
  xml << "    <body name=\"" << xml_escape(id) << "\" pos=\"" << pos << "\" quat=\"" << quat
      << "\">\n";
  xml << "      <geom name=\"" << xml_escape(id + "_geom") << "\" type=\"" << geom_type
      << "\" size=\"" << size << "\" rgba=\"" << rgba << "\"";
  if (mass > 0.0) xml << " mass=\"" << mass << "\"";
  xml << "/>\n";
  xml << "    </body>\n";
  return xml.str();
}

std::string SimulationCompiler::compile_sensor_object(const nlohmann::json& sensor) {
  if (!sensor.is_object()) throw std::invalid_argument("sensor object must be an object");
  const std::string id = sensor.value("id", "");
  const std::string type = sensor.value("type", "");
  const std::string site = sensor.value("site", "");
  if (id.empty()) throw std::invalid_argument("sensor missing 'id'");
  if (site.empty()) throw std::invalid_argument("sensor missing 'site'");

  std::string mujoco_type;
  if (type == "sensor.force") {
    mujoco_type = "force";
  } else if (type == "sensor.torque") {
    mujoco_type = "torque";
  } else {
    throw std::invalid_argument("unsupported sensor type: " + type);
  }

  std::ostringstream xml;
  xml << "    <" << mujoco_type << " name=\"" << xml_escape(id) << "\" site=\"" << xml_escape(site)
      << "\"/>\n";
  return xml.str();
}

std::string SimulationCompiler::compile_model_include_object(const nlohmann::json& scene,
                                                             const nlohmann::json& object) {
  if (!object.is_object()) throw std::invalid_argument("model include object must be an object");

  const std::string id = object.value("id", "");
  if (id.empty()) throw std::invalid_argument("model include missing 'id'");

  const auto source_path = resolve_scene_path(scene, object.value("source", ""));
  const std::string pos = numeric_list(object.value("pos", nlohmann::json::array()), "0 0 0", 3);
  const std::string quat
      = numeric_list(object.value("quat", nlohmann::json::array()), "1 0 0 0", 4);

  std::ifstream input(source_path);
  if (!input) throw std::runtime_error("failed to open include model: " + source_path.string());
  std::ostringstream included;
  included << input.rdbuf();
  const std::string included_xml = included.str();
  if (included_xml.find_first_not_of(" \t\r\n") == std::string::npos) {
    throw std::runtime_error("include model is empty: " + source_path.string());
  }

  std::ostringstream xml;
  xml << "    <body name=\"" << xml_escape(id) << "\" pos=\"" << pos << "\" quat=\"" << quat
      << "\">\n";
  xml << included_xml;
  if (!included_xml.empty() && included_xml.back() != '\n') xml << "\n";
  xml << "    </body>\n";
  return xml.str();
}

std::string SimulationCompiler::generate_scene_xml(
    const nlohmann::json& scene,
    const std::function<std::string(const nlohmann::json&, const std::filesystem::path&)>&
        model_file_ref) const {
  std::string xml;
  if (scene.value("model_path", "").empty()) {
    xml = "<mujoco model=\"" + xml_escape(scene.value("id", "scene"))
          + "\">\n  <worldbody>\n  </worldbody>\n</mujoco>\n";
  } else {
    const auto base_model = resolve_scene_path(scene, scene.value("model_path", ""));
    std::ifstream input(base_model);
    if (!input) throw std::runtime_error("failed to open base model: " + base_model.string());
    std::ostringstream buffer;
    buffer << input.rdbuf();
    xml = buffer.str();
  }

  std::ostringstream assets_xml;
  std::ostringstream objects_xml;
  std::ostringstream sensors_xml;
  std::ostringstream contacts_xml;

  for (const auto& asset : scene.value("models", nlohmann::json::array())) {
    const std::string id = asset.value("id", "");
    const std::string asset_name = "scene_model_" + id;
    const std::string prefix = asset.value("prefix", id + "_");
    const auto source = resolve_scene_path(scene, asset.value("source", ""));
    assets_xml << "    <model name=\"" << xml_escape(asset_name) << "\" file=\""
               << xml_escape(model_file_ref(asset, source)) << "\"/>\n";
    objects_xml << "    <frame name=\"" << xml_escape(id + "_mount") << "\" pos=\""
                << numeric_list(asset.value("pos", nlohmann::json::array()), "0 0 0", 3)
                << "\" quat=\""
                << numeric_list(asset.value("quat", nlohmann::json::array()), "1 0 0 0", 4)
                << "\">\n      <attach model=\"" << xml_escape(asset_name) << "\"";
    if (!asset.value("body", "").empty())
      objects_xml << " body=\"" << xml_escape(asset.value("body", "")) << "\"";
    objects_xml << " prefix=\"" << xml_escape(prefix) << "\"/>\n    </frame>\n";
  }

  if (scene.contains("objects") && scene["objects"].is_array()) {
    for (const auto& object : scene["objects"]) {
      const std::string type = object.value("type", "");
      if (is_sensor_type(type)) {
        sensors_xml << compile_sensor_object(object);
      } else if (is_model_include_type(type)) {
        objects_xml << compile_model_include_object(scene, object);
      } else {
        objects_xml << compile_primitive_object(object);
      }
    }
  }

  if (scene.contains("sensors") && scene["sensors"].is_array()) {
    for (const auto& sensor : scene["sensors"]) sensors_xml << compile_sensor_object(sensor);
  }

  const auto excludes = scene.value("contacts", nlohmann::json::object())
                            .value("excludes", nlohmann::json::array());
  for (const auto& item : excludes) {
    contacts_xml << "    <exclude";
    if (!item.value("name", "").empty())
      contacts_xml << " name=\"" << xml_escape(item.value("name", "")) << "\"";
    contacts_xml << " body1=\"" << xml_escape(item.value("body1", ""))
                 << "\" body2=\"" << xml_escape(item.value("body2", "")) << "\"/>\n";
  }

  append_section_xml(xml, "asset", assets_xml.str());
  insert_before_required(xml, "</worldbody>", objects_xml.str(),
                         "base model missing </worldbody>");
  append_section_xml(xml, "sensor", sensors_xml.str());
  append_section_xml(xml, "contact", contacts_xml.str());
  return xml;
}

std::filesystem::path SimulationCompiler::write_compiled_mjcf(const nlohmann::json& scene) const {
  const std::string xml = generate_scene_xml(
      scene, [](const nlohmann::json&, const std::filesystem::path& source) {
        return source.string();
      });

  std::filesystem::create_directories(output_dir_);
  const auto output_path
      = output_dir_ / (scene.value("id", "scene") + std::string(".compiled.xml"));
  std::ofstream output(output_path);
  if (!output) throw std::runtime_error("failed to write compiled model: " + output_path.string());
  output << xml;
  output.close();
  return std::filesystem::weakly_canonical(output_path);
}

namespace {

  std::string sanitize_folder(const std::string& value) {
    std::string out;
    out.reserve(value.size());
    for (unsigned char c : value)
      out.push_back((std::isalnum(c) || c == '_' || c == '-' || c == '.') ? static_cast<char>(c)
                                                                          : '_');
    return out.empty() ? std::string{"model"} : out;
  }

  void copy_tree(const std::filesystem::path& from, const std::filesystem::path& to) {
    namespace fs = std::filesystem;
    fs::create_directories(to);
    for (auto it = fs::recursive_directory_iterator(
             from, fs::directory_options::skip_permission_denied);
         it != fs::recursive_directory_iterator(); ++it) {
      if (it->is_symlink()) continue;
      const auto rel = fs::relative(it->path(), from);
      const auto dest = to / rel;
      std::error_code ec;
      if (it->is_directory()) {
        fs::create_directories(dest, ec);
      } else if (it->is_regular_file()) {
        fs::create_directories(dest.parent_path(), ec);
        fs::copy_file(it->path(), dest, fs::copy_options::overwrite_existing, ec);
        if (ec)
          throw std::runtime_error("failed to copy asset " + it->path().string() + ": "
                                   + ec.message());
      }
    }
  }

  // First regular file under any root whose filename matches `basename`. Used to
  // locate an asset source when flattening discards the original directory.
  std::optional<std::filesystem::path> find_by_basename(
      const std::vector<std::filesystem::path>& roots, const std::string& basename) {
    namespace fs = std::filesystem;
    for (const auto& root : roots) {
      if (!fs::is_directory(root)) continue;
      for (auto it = fs::recursive_directory_iterator(
               root, fs::directory_options::skip_permission_denied);
           it != fs::recursive_directory_iterator(); ++it) {
        if (it->is_regular_file() && it->path().filename().string() == basename)
          return it->path();
      }
    }
    return std::nullopt;
  }

  std::string parse_xml_attr(const std::string& xml, const std::string& attr) {
    const std::string key = attr + "=\"";
    const auto pos = xml.find(key);
    if (pos == std::string::npos) return "";
    const auto start = pos + key.size();
    const auto end = xml.find('"', start);
    if (end == std::string::npos) return "";
    return xml.substr(start, end - start);
  }

}  // namespace

nlohmann::json SimulationCompiler::export_scene(const nlohmann::json& scene,
                                                const std::string& out_dir_text,
                                                bool flatten) const {
  namespace fs = std::filesystem;
  const std::string scene_id = scene.value("id", "scene");

  fs::path out_dir;
  if (out_dir_text.empty()) {
    out_dir = default_export_dir() / scene_id;
  } else {
    out_dir = fs::path(out_dir_text);
    if (out_dir.is_relative()) out_dir = fs::current_path() / out_dir;
  }
  out_dir = fs::weakly_canonical(out_dir);
  fs::create_directories(out_dir);
  std::error_code ec;

  nlohmann::json warnings = nlohmann::json::array();
  const fs::path model_file = out_dir / "scene.xml";

  if (!flatten) {
    // ----- modular <attach> bundle (MuJoCo 3.2+) -----
    // This mode embeds a base model_path / model.include verbatim and only
    // relocates the packages of `models[]` assets, so external assets pulled in
    // by those legacy paths are not bundled (the flattened mode below is).
    if (!scene.value("model_path", "").empty())
      warnings.push_back(
          "scene uses a base model_path; its embedded assets are not relocated into the export");
    for (const auto& object : scene.value("objects", nlohmann::json::array())) {
      if (is_model_include_type(object.value("type", ""))) {
        warnings.push_back(
            "scene contains a model.include object; its embedded assets are not relocated");
        break;
      }
    }

    const fs::path assets_dir = out_dir / "assets";
    fs::remove_all(assets_dir, ec);
    fs::create_directories(assets_dir);

    nlohmann::json bundled = nlohmann::json::array();
    int index = 0;
    auto resolver = [&](const nlohmann::json& asset, const fs::path& source) -> std::string {
      const std::string id = asset.value("id", asset.value("model_id", "model"));
      fs::path pkg;
      const std::string pkg_text = asset.value("package_dir", "");
      if (!pkg_text.empty() && fs::is_directory(pkg_text))
        pkg = fs::weakly_canonical(pkg_text);
      else
        pkg = source.parent_path();

      const std::string folder = std::to_string(index++) + "_" + sanitize_folder(id);
      const fs::path dest = assets_dir / folder;
      copy_tree(pkg, dest);
      const fs::path file_rel = fs::path("assets") / folder / fs::relative(source, pkg);
      bundled.push_back(
          {{"id", id}, {"file", file_rel.generic_string()}, {"package", pkg.string()}});
      return file_rel.generic_string();
    };

    const std::string xml = generate_scene_xml(scene, resolver);
    {
      std::ofstream output(model_file);
      if (!output)
        throw std::runtime_error("failed to write exported scene: " + model_file.string());
      output << xml;
    }

    char error[2048] = {};
    nlohmann::json sizes;
    {
      std::lock_guard xml_lock{simulation_mujoco_xml_mutex()};
      mjModel* model = mj_loadXML(model_file.string().c_str(), nullptr, error, sizeof(error));
      if (!model)
        throw std::runtime_error(std::string("exported scene failed to load standalone: ")
                                 + (error[0] ? error : "unknown error"));
      sizes = {{"nq", model->nq}, {"nv", model->nv}, {"nu", model->nu},
               {"nbody", model->nbody}, {"nsensor", model->nsensor}};
      mj_deleteModel(model);
    }

    return {
        {"scene_id", scene_id},        {"export_dir", out_dir.string()},
        {"model_file", model_file.string()},
        {"open_with", "simulate \"" + model_file.string() + "\""},
        {"mode", "attach"},            {"models", bundled},
        {"sizes", sizes},              {"warnings", warnings},
        {"self_contained", warnings.empty()},
        {"min_mujoco", "3.2.0"},
    };
  }

  // ----- flattened single-file MJCF (default; opens in MuJoCo 3.1+) -----
  // Compose with absolute model refs so the temporary document loads regardless
  // of the export location, then let MuJoCo inline everything via mj_saveLastXML.
  const std::string composed = generate_scene_xml(
      scene, [](const nlohmann::json&, const fs::path& source) { return source.string(); });
  const fs::path compose_path = out_dir / ".compose.xml";
  {
    std::ofstream output(compose_path);
    if (!output)
      throw std::runtime_error("failed to write compose file: " + compose_path.string());
    output << composed;
  }

  // Package roots to locate asset files that flattening reduces to bare names.
  std::vector<fs::path> roots;
  for (const auto& asset : scene.value("models", nlohmann::json::array())) {
    const std::string pkg_text = asset.value("package_dir", "");
    if (!pkg_text.empty() && fs::is_directory(pkg_text))
      roots.push_back(fs::weakly_canonical(pkg_text));
    else {
      const auto src = resolve_scene_path(scene, asset.value("source", ""));
      if (!src.empty()) roots.push_back(src.parent_path());
    }
  }
  if (!scene.value("model_path", "").empty())
    roots.push_back(resolve_scene_path(scene, scene.value("model_path", "")).parent_path());
  for (const auto& object : scene.value("objects", nlohmann::json::array())) {
    if (is_model_include_type(object.value("type", ""))) {
      const auto src = resolve_scene_path(scene, object.value("source", ""));
      if (!src.empty()) roots.push_back(src.parent_path());
    }
  }

  char error[2048] = {};
  char save_error[2048] = {};
  nlohmann::json sizes;
  std::vector<std::pair<std::string, std::string>> refs;  // (ref, kind)
  bool saved = false;
  {
    std::lock_guard xml_lock{simulation_mujoco_xml_mutex()};
    mjModel* model = mj_loadXML(compose_path.string().c_str(), nullptr, error, sizeof(error));
    if (!model) {
      fs::remove(compose_path, ec);
      throw std::runtime_error(std::string("scene composition failed: ")
                               + (error[0] ? error : "unknown error"));
    }
    saved = mj_saveLastXML(model_file.string().c_str(), model, save_error, sizeof(save_error)) != 0;
    auto collect = [&](int count, const int* pathadr, const char* kind) {
      for (int i = 0; i < count; ++i) {
        if (pathadr && pathadr[i] >= 0) {
          std::string ref(model->paths + pathadr[i]);
          if (!ref.empty()) refs.emplace_back(ref, kind);
        }
      }
    };
    collect(model->nmesh, model->mesh_pathadr, "mesh");
    collect(model->ntex, model->tex_pathadr, "texture");
    collect(model->nhfield, model->hfield_pathadr, "hfield");
    collect(model->nskin, model->skin_pathadr, "skin");
    sizes = {{"nq", model->nq}, {"nv", model->nv}, {"nu", model->nu},
             {"nbody", model->nbody}, {"nsensor", model->nsensor}};
    mj_deleteModel(model);
    mj_freeLastXML();
  }
  fs::remove(compose_path, ec);
  if (!saved)
    throw std::runtime_error(std::string("failed to flatten scene: ")
                             + (save_error[0] ? save_error : "unknown error"));

  // Copy referenced assets next to scene.xml, honouring any dir the saver wrote.
  std::string flat_text;
  {
    std::ifstream in(model_file);
    std::ostringstream ss;
    ss << in.rdbuf();
    flat_text = ss.str();
  }
  const std::string meshdir = parse_xml_attr(flat_text, "meshdir");
  const std::string texturedir = parse_xml_attr(flat_text, "texturedir");
  nlohmann::json assets_copied = nlohmann::json::array();
  std::set<std::string> seen;
  for (const auto& [ref, kind] : refs) {
    if (!seen.insert(kind + '\0' + ref).second) continue;
    const std::string basename = fs::path(ref).filename().string();
    const auto found = find_by_basename(roots, basename);
    if (!found) {
      warnings.push_back("asset not found for bundling: " + ref);
      continue;
    }
    std::string sub = kind == std::string("texture") ? texturedir : meshdir;
    fs::path target = out_dir;
    if (!sub.empty() && fs::path(sub).is_relative()) target /= sub;
    target /= ref;
    fs::create_directories(target.parent_path(), ec);
    fs::copy_file(*found, target, fs::copy_options::overwrite_existing, ec);
    if (ec) {
      warnings.push_back("failed to copy asset " + ref + ": " + ec.message());
      continue;
    }
    assets_copied.push_back({{"ref", ref}, {"kind", kind}, {"source", found->string()}});
  }

  // Verify the flattened bundle loads standalone with its copied assets.
  {
    char verify_error[2048] = {};
    std::lock_guard xml_lock{simulation_mujoco_xml_mutex()};
    mjModel* model = mj_loadXML(model_file.string().c_str(), nullptr, verify_error,
                                sizeof(verify_error));
    if (!model)
      throw std::runtime_error(std::string("flattened scene failed to load standalone: ")
                               + (verify_error[0] ? verify_error : "unknown error"));
    mj_deleteModel(model);
  }

  return {
      {"scene_id", scene_id},
      {"export_dir", out_dir.string()},
      {"model_file", model_file.string()},
      {"open_with", "simulate \"" + model_file.string() + "\""},
      {"mode", "flatten"},
      {"assets", assets_copied},
      {"sizes", sizes},
      {"warnings", warnings},
      {"self_contained", warnings.empty()},
      {"min_mujoco", "3.1.0"},
  };
}
