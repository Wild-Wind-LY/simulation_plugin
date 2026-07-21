#pragma once

#include <nlohmann/json.hpp>
#include <string>

namespace simulation {

  // Invokes `visit(model_entry)` for every models[] entry in `scene` that
  // references a registered asset (has a "model_id" key). Shared by
  // SimulationModelRegistry::resolve_scene() (mutates each entry to attach the
  // resolved file path/version/package_dir) and
  // SimulationSceneManager::references_model() (reads model_id/version to check
  // whether a version is still in use), so neither keeps its own copy of "how a
  // scene references model assets". Template over the scene/model json
  // reference type so both the mutable (resolve) and const (reference-check)
  // call sites can share one implementation.
  template <typename Json, typename Visit> void walk_model_references(Json& scene, Visit&& visit) {
    if (!scene.contains("models") || !scene["models"].is_array()) return;
    for (auto& model : scene["models"]) {
      if (!model.is_object() || !model.contains("model_id")) continue;
      visit(model);
    }
  }

}  // namespace simulation
