#pragma once

#include <mujoco/mujoco.h>

#include <cstdint>
#include <filesystem>
#include <functional>
#include <initializer_list>
#include <memory>
#include <mutex>
#include <nlohmann/json.hpp>
#include <string>
#include <unordered_map>
#include <utility>

class SimulationCompiler {
public:
  using ModelPtr = std::shared_ptr<mjModel>;

  explicit SimulationCompiler(std::filesystem::path output_dir = default_output_dir(),
                              size_t max_cached_models = 64);

  nlohmann::json validate_scene(const nlohmann::json& scene) const;
  nlohmann::json diff_scenes(const nlohmann::json& old_scene,
                             const nlohmann::json& new_scene) const;
  // `out_model`, when non-null, receives the same ModelPtr this call just
  // inserted into (or found in) the cache -- kept alive by the caller's own
  // copy from this point on, so a caller that needs the mjModel right after
  // compiling never has to re-look-it-up via a separate get_compiled_model()
  // call. That separate-lookup pattern used to be the only way to get the
  // model, which left a window between this call returning and the follow-up
  // lookup where the cache's own copy could be the *only* live reference
  // (use_count()==1) and therefore eligible for LRU eviction by an unrelated
  // concurrent compile_scene() -- so a scene that had just compiled
  // successfully could fail immediately after with "compiled model not
  // found". Passing out_model closes that window entirely.
  nlohmann::json compile_scene(const nlohmann::json& scene, ModelPtr* out_model = nullptr) const;
  // Emit a self-contained MJCF bundle at `<out_dir>/scene.xml` that opens
  // directly in MuJoCo `simulate` on any machine. `out_dir` empty -> default
  // export directory keyed by scene id.
  //
  // flatten=true (default) writes a single flattened MJCF (via mj_saveLastXML)
  // with all bodies inlined and referenced meshes/textures copied alongside --
  // it avoids the `<model>`/`<attach>` composition syntax and therefore opens in
  // older `simulate` builds (MuJoCo 3.1+). flatten=false keeps the modular
  // `<attach>` bundle under `assets/`, which requires MuJoCo 3.2+.
  nlohmann::json export_scene(const nlohmann::json& scene, const std::string& out_dir = "",
                              bool flatten = true) const;
  nlohmann::json build_visual_scene(const nlohmann::json& scene) const;
  nlohmann::json build_instance_config(const nlohmann::json& request,
                                       const nlohmann::json& compiled_scene) const;
  ModelPtr get_compiled_model(const std::string& compiled_model_id) const;
  std::pair<std::string, ModelPtr> get_or_load_model(const std::string& model_path) const;
  size_t prune_unused_models() const;
  size_t cached_model_count() const;

private:
  static std::filesystem::path default_output_dir();
  static std::filesystem::path default_export_dir();
  // Generate the full scene MJCF as a string. `model_file_ref` decides the
  // `file=` attribute for each `<model>` asset given the asset entry and its
  // resolved absolute source, letting callers emit absolute (compile) or
  // relative-and-bundled (export) references. Section-generation helpers
  // (asset/worldbody/contact/equality/tendon/actuator/sensor/keyframe) and the
  // small stateless utilities they share (xml_escape, numeric_list,
  // resolve_scene_path, ...) live as free functions in simulation_compiler.cpp's
  // anonymous namespace rather than class members, since none of them touch
  // instance state and every new MJCF element type would otherwise need its own
  // header declaration.
  std::string generate_scene_xml(
      const nlohmann::json& scene,
      const std::function<std::string(const nlohmann::json&, const std::filesystem::path&)>&
          model_file_ref) const;
  static std::string structural_model_id(const nlohmann::json& scene);
  static ModelPtr load_model(const std::filesystem::path& path);
  std::filesystem::path write_compiled_mjcf(const nlohmann::json& scene,
                                            const std::string& compiled_model_id) const;

  struct CacheEntry {
    ModelPtr model;
    uint64_t last_used = 0;
    std::filesystem::path compiled_path;  // only set for compile_scene()'s entries
  };
  // Records `model_id` as most-recently-used and, if the cache is over capacity,
  // evicts the least-recently-used entries that no live instance still holds.
  // `compiled_path`, when non-empty, is stashed on the entry so a later cache hit
  // can still report it without recompiling.
  ModelPtr touch_and_reclaim_locked(const std::string& model_id, ModelPtr model,
                                    std::filesystem::path compiled_path = {}) const;

  std::filesystem::path output_dir_;
  size_t max_cached_models_;
  mutable std::mutex model_mutex_;
  mutable std::unordered_map<std::string, CacheEntry> models_;
  mutable uint64_t access_tick_ = 0;
};
