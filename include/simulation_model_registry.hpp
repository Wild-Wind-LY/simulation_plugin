#pragma once

#include <cstdint>
#include <filesystem>
#include <mutex>
#include <nlohmann/json.hpp>
#include <string>
#include <unordered_map>
#include <unordered_set>

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
  //   require_single_asset (default false) reject models that look like a whole
  //                   scene (multiple independently-jointed root subtrees, or a
  //                   world-attached ground plane) rather than one reusable
  //                   asset; when false (default) this only adds a warning to
  //                   entry.warnings/scene_like instead of rejecting
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
  // Builds the {schema, models} manifest JSON from `entries_`. Called under
  // `mutex_` -- cheap (no I/O), so it doesn't hold up other registry ops.
  nlohmann::json snapshot_manifest_locked() const;
  // Actually writes `manifest` to disk. Called *without* `mutex_` held (so
  // list()/info()/verify()/resolve_scene() aren't blocked for the write's
  // duration) -- serialized instead by `manifest_io_mutex_`, with `generation`
  // used to drop a write that's older than one that already landed.
  void persist_manifest(nlohmann::json manifest, uint64_t generation) const;

  std::filesystem::path storage_dir_;
  std::filesystem::path manifest_path_;
  mutable std::mutex mutex_;
  std::unordered_map<std::string, nlohmann::json> entries_;
  // id@version keys currently mid-registration (guarded by mutex_ like entries_).
  // Without this, two concurrent register_model() calls for the same not-yet-
  // registered id@version can both pass the "not already registered" check,
  // both target the same package_dir(id,version), and the loser's cleanup-on-
  // failure remove_all() deletes the files the winner just committed.
  std::unordered_set<std::string> registering_;
  mutable std::mutex manifest_io_mutex_;
  mutable uint64_t manifest_generation_{0};
  mutable uint64_t last_written_generation_{0};
};
