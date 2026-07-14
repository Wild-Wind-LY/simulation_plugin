#pragma once

#include <cstdint>
#include <filesystem>
#include <mutex>
#include <nlohmann/json.hpp>
#include <string>
#include <unordered_map>

// Production-grade registry for reusable robot model assets (MJCF/URDF).
//
// Guarantees provided beyond a plain path bookkeeping table:
//  - Self-contained packaging: on registration the model and its neighbouring
//    asset directory are snapshotted into `storage/<id>/<version>/`, so a
//    registered version keeps working even if the original files move or are
//    deleted (see `copy_assets` / `asset_root`).
//  - Portability: the manifest stores paths relative to the storage root; they
//    are hydrated back to absolute paths on read.
//  - Integrity: a SHA-256 digest, byte size and mtime of the effective model
//    file are recorded so drift/tampering can be detected via `verify`.
//  - Robot metadata: actuator/joint/sensor summary and `controllable` flag are
//    captured from the compiled model at registration time.
//  - Robustness: a corrupt manifest is backed up and the registry starts empty
//    instead of taking the whole plugin down.
class SimulationModelRegistry {
public:
  explicit SimulationModelRegistry(std::filesystem::path storage_dir = default_storage_dir());

  // Register a versioned MJCF/URDF asset. Recognised fields in `data`:
  //   id | model_id   (required)  stable asset id
  //   version         (default "1")
  //   path | model_path (required) source model file
  //   replace         (default false) replace an existing version
  //   copy_assets     (default true)  snapshot the model + asset dir into storage
  //   asset_root      (optional) directory to snapshot; defaults to the model's
  //                   parent directory; must be an ancestor of the model file
  //   require_actuators (default false) reject models with no actuators
  //   max_asset_bytes (optional) cap for the snapshotted package size
  nlohmann::json register_model(const nlohmann::json& data);
  nlohmann::json list() const;
  nlohmann::json info(const nlohmann::json& data) const;
  nlohmann::json remove(const nlohmann::json& data);
  // Re-check that a registered version's effective file still matches the digest
  // recorded at registration time.
  nlohmann::json verify(const nlohmann::json& data) const;
  nlohmann::json resolve_scene(nlohmann::json scene) const;

private:
  static std::filesystem::path default_storage_dir();
  static std::string key(const std::string& id, const std::string& version);
  static std::string detect_format(const std::filesystem::path& path);
  static int64_t now_ms();
  static std::filesystem::path normalize_path(const std::string& path_text);
  static bool is_newer_version(const nlohmann::json& candidate, const nlohmann::json& current);

  std::filesystem::path convert_urdf(const std::filesystem::path& source,
                                     const std::filesystem::path& output) const;
  std::filesystem::path package_dir(const std::string& id, const std::string& version) const;
  std::filesystem::path snapshot_package(const std::filesystem::path& asset_root,
                                         const std::filesystem::path& pkg_dir,
                                         uint64_t max_bytes) const;
  // Hydrate manifest-form entry (relative paths) into an outward entry with
  // absolute `source_path` / `effective_path`.
  nlohmann::json hydrate(const nlohmann::json& entry) const;
  std::filesystem::path effective_abs(const nlohmann::json& entry) const;
  nlohmann::json resolve_locked(const std::string& id, const std::string& version) const;
  nlohmann::json integrity_status(const nlohmann::json& entry) const;
  void load_manifest();
  void save_manifest_locked() const;

  std::filesystem::path storage_dir_;
  std::filesystem::path manifest_path_;
  mutable std::mutex mutex_;
  std::unordered_map<std::string, nlohmann::json> entries_;
};
