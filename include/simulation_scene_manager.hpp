#pragma once

#include <mutex>
#include <nlohmann/json.hpp>
#include <string>
#include <unordered_map>

class SimulationSceneManager {
public:
  nlohmann::json load(const nlohmann::json& data);
  // Create an empty (or seeded) scene in memory without a backing file.
  // Fields: id (required), replace (default false), and optional scene content
  // (name/model_path/models/objects/sensors/contacts/defaults or a full `scene`).
  nlohmann::json create(const nlohmann::json& data);
  // Persist a loaded scene to disk as JSON. Fields: id (required), path
  // (optional; defaults to the scene's source path, else build/scenes/<id>.json).
  nlohmann::json save(const nlohmann::json& data);
  nlohmann::json unload(const nlohmann::json& data);
  nlohmann::json update(const nlohmann::json& data);
  nlohmann::json list() const;
  nlohmann::json info(const nlohmann::json& data) const;
  // Returns true if a loaded scene would use `model_id@version`. A scene that
  // pins no version resolves to `latest_version`; pass it so removing a
  // non-latest version is not blocked. When `latest_version` is empty the check
  // stays conservative and treats an unpinned reference as matching any version.
  bool references_model(const std::string& model_id, const std::string& version,
                        const std::string& latest_version = "") const;

private:
  static std::string require_id(const nlohmann::json& data);

  mutable std::mutex mutex_;
  std::unordered_map<std::string, nlohmann::json> scenes_;
};