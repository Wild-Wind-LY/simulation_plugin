#include "simulation_model_registry.hpp"

#include <mujoco/mujoco.h>

#include <algorithm>
#include <cctype>
#include <chrono>
#include <cstdint>
#include <fstream>
#include <optional>
#include <set>
#include <sstream>
#include <stdexcept>
#include <system_error>
#include <vector>

#include "simulation_hash.hpp"
#include "simulation_mujoco_utils.hpp"
#include "simulation_paths.hpp"
#include "simulation_scene_refs.hpp"

namespace {

  using simulation::sha256_directory;
  using simulation::sha256_file;

  // ----------------------------- identifiers -----------------------------

  bool safe_identifier(const std::string& value) {
    if (value.empty() || value == "." || value == "..") return false;
    return std::all_of(value.begin(), value.end(), [](unsigned char c) {
      return std::isalnum(c) || c == '_' || c == '-' || c == '.';
    });
  }

  // Best-effort projection of an arbitrary string (an XML attribute value, a
  // filename stem) onto the safe_identifier charset: unsupported characters
  // are dropped, runs of whitespace collapse to a single '_', and leading/
  // trailing separators are trimmed so the result can't collide with "." or
  // "..". Returns "" if nothing usable survives -- callers must check that.
  std::string sanitize_identifier_token(const std::string& text) {
    std::string out;
    for (const unsigned char ch : text) {
      if (std::isalnum(ch) || ch == '_' || ch == '-' || ch == '.') {
        out.push_back(static_cast<char>(ch));
      } else if (std::isspace(ch) && !out.empty() && out.back() != '_') {
        out.push_back('_');
      }
    }
    while (!out.empty() && (out.front() == '.' || out.front() == '-')) out.erase(out.begin());
    while (!out.empty() && (out.back() == '.' || out.back() == '-' || out.back() == '_'))
      out.pop_back();
    return out;
  }

  // Finds the `attribute="value"` (or '...') pair inside the opening tag
  // named `tag` (e.g. "<mujoco", "<robot"), searching only up to that tag's
  // closing '>' so it can't match an attribute belonging to some later,
  // unrelated element.
  std::optional<std::string> extract_xml_attribute(const std::string& head, const std::string& tag,
                                                   const std::string& attribute) {
    const auto tag_pos = head.find(tag);
    if (tag_pos == std::string::npos) return std::nullopt;
    const auto tag_end = head.find('>', tag_pos);
    const auto region = tag_end != std::string::npos ? head.substr(tag_pos, tag_end - tag_pos)
                                                     : head.substr(tag_pos);
    for (const char quote : {'"', '\''}) {
      const std::string needle = attribute + "=" + quote;
      const auto attr_pos = region.find(needle);
      if (attr_pos == std::string::npos) continue;
      const auto value_start = attr_pos + needle.size();
      const auto value_end = region.find(quote, value_start);
      if (value_end == std::string::npos) continue;
      return region.substr(value_start, value_end - value_start);
    }
    return std::nullopt;
  }

  // Infers a model id when the caller didn't supply one: prefer the model's
  // own declared name (MJCF `<mujoco model="...">` / URDF `<robot name="...">`)
  // so drag-and-drop registration lines up with what the asset calls itself,
  // falling back to the file's stem. Throws rather than guessing something
  // that would collide or violate safe_identifier, since a wrong silent guess
  // is worse than asking the caller to be explicit.
  std::string infer_model_id(const std::filesystem::path& path, const std::string& format) {
    std::ifstream input(path, std::ios::binary);
    if (input) {
      std::string head(8192, '\0');
      input.read(head.data(), static_cast<std::streamsize>(head.size()));
      head.resize(static_cast<size_t>(input.gcount()));
      const auto attr = format == "urdf" ? extract_xml_attribute(head, "<robot", "name")
                                         : extract_xml_attribute(head, "<mujoco", "model");
      if (attr) {
        const auto sanitized = sanitize_identifier_token(*attr);
        if (safe_identifier(sanitized)) return sanitized;
      }
    }
    const auto sanitized = sanitize_identifier_token(path.stem().string());
    if (safe_identifier(sanitized)) return sanitized;
    throw std::invalid_argument(
        "could not infer model 'id' from file content or filename; specify 'id' explicitly");
  }

  // ----------------------------- semver ----------------------------------

  int compare_versions(const std::string& lhs, const std::string& rhs) {
    auto split = [](const std::string& value) {
      std::vector<std::string> parts;
      std::string current;
      for (char c : value) {
        if (c == '.') {
          parts.push_back(current);
          current.clear();
        } else {
          current.push_back(c);
        }
      }
      parts.push_back(current);
      return parts;
    };
    const auto a = split(lhs);
    const auto b = split(rhs);
    const size_t count = std::max(a.size(), b.size());
    for (size_t i = 0; i < count; ++i) {
      const std::string pa = i < a.size() ? a[i] : "0";
      const std::string pb = i < b.size() ? b[i] : "0";
      const bool a_num = !pa.empty() && std::all_of(pa.begin(), pa.end(), ::isdigit);
      const bool b_num = !pb.empty() && std::all_of(pb.begin(), pb.end(), ::isdigit);
      if (a_num && b_num) {
        // safe_identifier only restricts the character set, not the length, so
        // an all-digit segment can still be long enough to overflow unsigned
        // long long and make stoull throw std::out_of_range -- that would
        // otherwise propagate out of this comparator (used from list()'s
        // std::sort), so fall back to a lexical compare for that segment
        // instead of letting it escape as an unrelated-looking exception.
        try {
          const unsigned long long na = std::stoull(pa);
          const unsigned long long nb = std::stoull(pb);
          if (na != nb) return na < nb ? -1 : 1;
        } catch (const std::exception&) {
          if (pa != pb) return pa < pb ? -1 : 1;
        }
      } else if (pa != pb) {
        return pa < pb ? -1 : 1;
      }
    }
    return 0;
  }

  // ------------------------- filesystem helpers --------------------------

  // Opaque, comparable filesystem tick for the effective file. Not a wall-clock
  // timestamp (the file_time_type epoch is implementation defined), but enough to
  // notice that the file changed without re-hashing.
  int64_t file_mtime_tick(const std::filesystem::path& path) {
    std::error_code ec;
    const auto ftime = std::filesystem::last_write_time(path, ec);
    if (ec) return 0;
    return static_cast<int64_t>(ftime.time_since_epoch().count());
  }

  bool is_within(const std::filesystem::path& child, const std::filesystem::path& ancestor) {
    auto c = child.begin();
    for (auto a = ancestor.begin(); a != ancestor.end(); ++a, ++c) {
      if (c == child.end() || *c != *a) return false;
    }
    return true;
  }

  // Lets register_model() take a directory as `path` (e.g. a robot's asset
  // folder dropped in as-is) by locating the single model file inside it.
  // Non-recursive: the top-level model file is expected to sit alongside its
  // meshes/textures subfolders, not buried inside one of them.
  std::filesystem::path find_model_file_in_directory(const std::filesystem::path& dir) {
    std::vector<std::filesystem::path> candidates;
    std::error_code ec;
    for (const auto& dir_entry : std::filesystem::directory_iterator(dir, ec)) {
      // is_regular_file() follows symlinks, but snapshot_package()/copy_directory_tree
      // explicitly skips symlinked entries when packaging -- picking a symlink here
      // would select a "source" that never actually gets copied into the package,
      // so effective_path ends up missing and mj_loadXML fails with an unrelated-
      // looking "failed to compile model" instead of a clear "no usable model file".
      if (dir_entry.is_symlink() || !dir_entry.is_regular_file()) continue;
      auto ext = dir_entry.path().extension().string();
      std::transform(ext.begin(), ext.end(), ext.begin(),
                     [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
      if (ext == ".xml" || ext == ".urdf" || ext == ".mjcf") candidates.push_back(dir_entry.path());
    }
    if (candidates.empty())
      throw std::invalid_argument("no model file (.xml/.urdf/.mjcf) found in directory: "
                                  + dir.string());
    if (candidates.size() > 1) {
      std::sort(candidates.begin(), candidates.end());
      std::string list;
      for (const auto& candidate : candidates) {
        if (!list.empty()) list += ", ";
        list += candidate.filename().string();
      }
      throw std::invalid_argument("multiple candidate model files found in directory "
                                  + dir.string() + " (" + list
                                  + "); specify 'path' to the exact model file");
    }
    return candidates.front();
  }

  // Heuristics for "this registered file is actually a whole scene (several
  // independent robots/tools composed together), not a single reusable asset".
  // Neither signal is 100% reliable (a legitimate multi-fingered gripper can
  // have several independently-jointed subtrees; some asset files do bundle a
  // ground plane for standalone `simulate` preview), so this only ever
  // produces warnings for register_model() to surface -- it never rejects on
  // its own; a caller that wants strict rejection opts in via
  // `require_single_asset` (see register_model()).
  nlohmann::json scene_like_warnings(const mjModel* model) {
    nlohmann::json warnings = nlohmann::json::array();

    // Signal 1 (strongest): count distinct world-anchored root subtrees that
    // have at least one joint somewhere inside them. MuJoCo guarantees
    // body_parentid[i] < i for every body, so a single forward pass can fold
    // each body up to the root it hangs off of directly under the world body
    // (id 0). Two or more independently-jointed roots is the shape of "several
    // robots/tools composed into one file", not a single reusable asset.
    if (model->nbody > 1 && model->njnt > 0) {
      std::vector<int> root_of(static_cast<size_t>(model->nbody), 0);
      for (int i = 1; i < model->nbody; ++i) {
        const int parent = model->body_parentid[i];
        root_of[static_cast<size_t>(i)] = parent == 0 ? i : root_of[static_cast<size_t>(parent)];
      }
      std::set<int> roots_with_joints;
      for (int j = 0; j < model->njnt; ++j) {
        const int body = model->jnt_bodyid[j];
        if (body > 0) roots_with_joints.insert(root_of[static_cast<size_t>(body)]);
      }
      if (roots_with_joints.size() >= 2) {
        warnings.push_back("model has " + std::to_string(roots_with_joints.size())
                           + " independently-jointed root subtrees under the world body -- this "
                             "looks like a scene with multiple robots/tools composed together, "
                             "not a single reusable asset");
      }
    }

    // Signal 2 (supplementary): a plane geom attached directly to the world
    // body (bodyid 0 -- the common case: a bare <geom type="plane"/> sitting
    // straight inside <worldbody>, exactly how this project's own compiler
    // emits its ground plane in generate_worldbody_xml) reads as "this file
    // brought its own ground", which scenes do and standalone assets usually
    // don't.
    for (int i = 0; i < model->ngeom; ++i) {
      if (model->geom_type[i] == mjGEOM_PLANE && model->geom_bodyid[i] == 0) {
        warnings.push_back(
            "model has a ground-like plane geom attached directly to the world "
            "body -- scenes usually supply their own ground, a reusable asset "
            "usually shouldn't");
        break;
      }
    }

    return warnings;
  }

  // Inspect a compiled model and report robot-relevant metadata / warnings.
  nlohmann::json inspect_compiled_model(const mjModel* model, const std::string& format) {
    nlohmann::json warnings = nlohmann::json::array();
    if (model->nu == 0) warnings.push_back("model has no actuators; ctrl commands cannot move it");
    if (model->njnt == 0) warnings.push_back("model has no joints");
    if (model->nsensor == 0) warnings.push_back("model has no sensors");
    if (format == "urdf" && model->nu == 0)
      warnings.push_back("URDF loaded, but no MuJoCo actuators were created");
    const auto scene_warnings = scene_like_warnings(model);
    for (const auto& w : scene_warnings) warnings.push_back(w);

    return {
        {"controllable", model->nu > 0},
        {"scene_like", !scene_warnings.empty()},
        {"warnings", warnings},
        {"sizes",
         {{"nq", model->nq},
          {"nv", model->nv},
          {"nu", model->nu},
          {"njnt", model->njnt},
          {"nbody", model->nbody},
          {"nsensor", model->nsensor},
          {"nsensordata", model->nsensordata}}},
        {"summary",
         {{"joints", model->njnt}, {"actuators", model->nu}, {"sensors", model->nsensor}}},
    };
  }

}  // namespace

SimulationModelRegistry::SimulationModelRegistry(std::filesystem::path storage_dir)
    : storage_dir_(std::filesystem::weakly_canonical(std::move(storage_dir))),
      manifest_path_(storage_dir_ / "registry.json") {
  load_manifest();
}

nlohmann::json SimulationModelRegistry::register_model(const nlohmann::json& data) {
  std::string id = data.value("id", data.value("model_id", ""));
  const std::string version = data.value("version", "1");
  const std::string path_text = data.value("path", data.value("model_path", ""));
  const bool replace = data.value("replace", false);
  const bool copy_assets = data.value("copy_assets", true);
  const bool require_actuators = data.value("require_actuators", false);
  // See scene_like_warnings(): opt-in strict rejection for "this looks like a
  // whole scene, not a single reusable asset" -- default false (warn only via
  // entry.warnings/scene_like) so this can't break existing registration flows.
  const bool require_single_asset = data.value("require_single_asset", false);
  const uint64_t max_asset_bytes
      = data.value("max_asset_bytes", static_cast<uint64_t>(1) << 30);  // 1 GiB default

  if (!id.empty() && !safe_identifier(id))
    throw std::invalid_argument(
        "model 'id' may only contain letters, digits, '.', '_', '-' "
        "and must not be '.' or '..'");
  if (!safe_identifier(version))
    throw std::invalid_argument(
        "model 'version' may only contain letters, digits, '.', '_', '-' "
        "and must not be '.' or '..'");
  if (path_text.empty()) throw std::invalid_argument("missing model 'path'");

  auto source = normalize_path(path_text);
  if (!std::filesystem::exists(source))
    throw std::invalid_argument("model path does not exist: " + source.string());
  // A directory drop: locate the single model file inside it so `asset_root`
  // (below) still defaults to the whole folder, meshes/textures included.
  if (std::filesystem::is_directory(source)) source = find_model_file_in_directory(source);
  const std::string format = simulation_detect_model_format(source);
  if (format != "mjcf" && format != "urdf")
    throw std::invalid_argument("unsupported model format: " + format);

  if (id.empty()) id = infer_model_id(source, format);

  const auto entry_key = key(id, version);
  {
    std::lock_guard lock{mutex_};
    if (entries_.find(entry_key) != entries_.end() && !replace)
      throw std::invalid_argument("model version already registered: " + entry_key);
    if (!registering_.insert(entry_key).second)
      throw std::invalid_argument("model version registration already in progress: " + entry_key);
  }
  // RAII release of the `registering_` reservation above -- fires on every exit
  // path (success or any of the throws below) so a failed/aborted registration
  // never permanently blocks retrying the same id@version.
  struct RegistrationGuard {
    SimulationModelRegistry* self;
    std::string key;
    ~RegistrationGuard() {
      std::lock_guard lock{self->mutex_};
      self->registering_.erase(key);
    }
  } registration_guard{this, entry_key};

  std::filesystem::create_directories(storage_dir_);

  // Decide where the effective (loadable) model lives and whether assets are
  // snapshotted into a self-contained package.
  const auto pkg = package_dir(id, version);
  std::error_code ec;
  std::filesystem::remove_all(pkg, ec);  // clean slate for replace

  std::filesystem::path effective_path;  // file that is compiled / attached
  std::filesystem::path source_copy;     // original-format file within package
  bool packaged = false;

  if (copy_assets) {
    std::filesystem::path asset_root;
    if (data.contains("asset_root") && data["asset_root"].is_string()
        && !data["asset_root"].get<std::string>().empty()) {
      asset_root = normalize_path(data["asset_root"].get<std::string>());
      if (!std::filesystem::is_directory(asset_root))
        throw std::invalid_argument("asset_root is not a directory: " + asset_root.string());
      if (!is_within(source, asset_root))
        throw std::invalid_argument("asset_root must be an ancestor of the model file");
    } else {
      asset_root = source.parent_path();
    }

    snapshot_package(asset_root, pkg, max_asset_bytes);
    source_copy = pkg / std::filesystem::relative(source, asset_root);

    if (format == "urdf") {
      effective_path = source_copy;
      effective_path.replace_extension(".mjcf.xml");
      convert_urdf(source_copy, effective_path);
    } else {
      effective_path = source_copy;
    }
    packaged = true;
  } else {
    // Reference mode: keep the original file in place (not portable / not durable).
    source_copy = source;
    if (format == "urdf") {
      effective_path = storage_dir_ / (id + "-" + version + ".mjcf.xml");
      convert_urdf(source, effective_path);
    } else {
      effective_path = source;
    }
  }

  // Validate the effective artifact and capture robot metadata from it.
  char error[2048] = {};
  mjModel* checked = nullptr;
  {
    std::lock_guard xml_lock{simulation_mujoco_xml_mutex()};
    checked = mj_loadXML(effective_path.string().c_str(), nullptr, error, sizeof(error));
  }
  if (!checked) {
    std::filesystem::remove_all(pkg, ec);
    throw std::invalid_argument(error[0] ? error : "failed to compile model");
  }
  const auto metadata = inspect_compiled_model(checked, format);
  const bool controllable = metadata.value("controllable", false);
  mj_deleteModel(checked);

  if (require_actuators && !controllable) {
    std::filesystem::remove_all(pkg, ec);
    throw std::invalid_argument("model has no actuators but require_actuators was set");
  }
  const bool scene_like = metadata.value("scene_like", false);
  if (require_single_asset && scene_like) {
    std::filesystem::remove_all(pkg, ec);
    throw std::invalid_argument(
        "model looks like a multi-robot scene, not a single reusable asset "
        "(pass require_single_asset=false to register anyway)");
  }

  nlohmann::json entry = {
      {"id", id},
      {"version", version},
      {"format", format},
      {"packaged", packaged},
      {"origin_path", source.string()},
      {"registered_ms", now_ms()},
      {"sizes", metadata.value("sizes", nlohmann::json::object())},
      {"controllable", controllable},
      {"scene_like", scene_like},
      {"warnings", metadata.value("warnings", nlohmann::json::array())},
      {"summary", metadata.value("summary", nlohmann::json::object())},
  };
  if (packaged) {
    entry["effective_rel"] = std::filesystem::relative(effective_path, storage_dir_).string();
    entry["source_rel"] = std::filesystem::relative(source_copy, storage_dir_).string();
    // Whole-package digest so tampering with any snapshotted mesh/texture/model
    // file is detectable, not only the top-level model file. integrity_status()
    // only ever checks this digest once packaged, so there's no need to also
    // hash `effective_path` on its own (that would just re-read+re-hash bytes
    // already covered by the directory digest, for a value nothing consults).
    const auto fingerprint = simulation::directory_fingerprint(pkg);
    entry["package_sha256"] = sha256_directory(pkg);
    entry["package_file_count"] = fingerprint.file_count;
    entry["package_total_bytes"] = fingerprint.total_bytes;
    entry["package_mtime_tick"] = fingerprint.max_mtime_ticks;
  } else {
    entry["effective_path"] = effective_path.string();
    entry["source_path"] = source_copy.string();
    entry["content_sha256"] = sha256_file(effective_path);
    entry["content_bytes"] = static_cast<uint64_t>(std::filesystem::file_size(effective_path, ec));
    entry["content_mtime_tick"] = file_mtime_tick(effective_path);
  }

  nlohmann::json manifest_snapshot;
  uint64_t generation = 0;
  {
    std::lock_guard lock{mutex_};
    if (entries_.find(entry_key) != entries_.end() && !replace) {
      std::filesystem::remove_all(pkg, ec);
      throw std::invalid_argument("model version already registered: " + entry_key);
    }
    entries_[entry_key] = entry;
    manifest_snapshot = snapshot_manifest_locked();
    generation = ++manifest_generation_;
  }
  // Manifest write happens without `mutex_` held so it doesn't block concurrent
  // list()/info()/verify()/resolve_scene() calls (a quick map lookup each) for
  // the write's duration.
  persist_manifest(std::move(manifest_snapshot), generation);
  return hydrate(entry);
}

nlohmann::json SimulationModelRegistry::list() const {
  std::lock_guard lock{mutex_};
  nlohmann::json out = nlohmann::json::array();
  for (const auto& [entry_key, entry] : entries_) {
    (void)entry_key;
    out.push_back(hydrate(entry));
  }
  std::sort(out.begin(), out.end(), [](const auto& lhs, const auto& rhs) {
    if (lhs.value("id", "") != rhs.value("id", ""))
      return lhs.value("id", "") < rhs.value("id", "");
    return compare_versions(lhs.value("version", ""), rhs.value("version", "")) < 0;
  });
  return out;
}

nlohmann::json SimulationModelRegistry::info(const nlohmann::json& data) const {
  const std::string id = data.value("id", data.value("model_id", ""));
  if (id.empty()) throw std::invalid_argument("missing model 'id'");
  std::lock_guard lock{mutex_};
  const auto entry = resolve_locked(id, data.value("version", ""));
  auto out = hydrate(entry);
  out["integrity"] = integrity_status(entry);
  return out;
}

nlohmann::json SimulationModelRegistry::remove(const nlohmann::json& data) {
  const std::string id = data.value("id", data.value("model_id", ""));
  const std::string version = data.value("version", "");
  if (id.empty() || version.empty())
    throw std::invalid_argument("model remove requires id and version");

  nlohmann::json entry;
  bool other_versions = false;
  nlohmann::json manifest_snapshot;
  uint64_t generation = 0;
  {
    std::lock_guard lock{mutex_};
    auto it = entries_.find(key(id, version));
    if (it == entries_.end()) throw std::out_of_range("model version not found");
    entry = it->second;
    entries_.erase(it);
    // Last version of this id gone: the now-empty "<storage_dir_>/<id>/" parent
    // is otherwise orphaned on disk forever, since nothing else ever cleans it up.
    other_versions = std::any_of(entries_.begin(), entries_.end(),
                                 [&](const auto& kv) { return kv.second.value("id", "") == id; });
    manifest_snapshot = snapshot_manifest_locked();
    generation = ++manifest_generation_;
  }
  // Filesystem cleanup happens without mutex_ held -- mirrors persist_manifest's
  // own "don't hold the lock across slow IO" rule just below; deleting a large
  // packaged model's meshes/textures would otherwise block every concurrent
  // list()/info()/verify()/resolve_scene() call for the duration.
  std::error_code ec;
  if (entry.value("packaged", false)) {
    const auto pkg = package_dir(id, version);
    if (is_within(pkg, storage_dir_)) std::filesystem::remove_all(pkg, ec);
    if (!other_versions) {
      const auto model_dir = pkg.parent_path();
      if (is_within(model_dir, storage_dir_)) std::filesystem::remove_all(model_dir, ec);
    }
  } else if (entry.value("format", "") == "urdf") {
    std::filesystem::remove(entry.value("effective_path", ""), ec);
  }
  persist_manifest(std::move(manifest_snapshot), generation);
  return {{"id", id}, {"version", version}, {"removed", true}};
}

nlohmann::json SimulationModelRegistry::verify(const nlohmann::json& data) const {
  const std::string id = data.value("id", data.value("model_id", ""));
  if (id.empty()) throw std::invalid_argument("missing model 'id'");
  std::lock_guard lock{mutex_};
  const auto entry = resolve_locked(id, data.value("version", ""));
  nlohmann::json out = {
      {"id", entry.value("id", "")},
      {"version", entry.value("version", "")},
  };
  out["integrity"] = integrity_status(entry);
  out["ok"] = out["integrity"].value("ok", false);
  return out;
}

nlohmann::json SimulationModelRegistry::resolve_scene(nlohmann::json scene) const {
  std::lock_guard lock{mutex_};
  simulation::walk_model_references(scene, [&](nlohmann::json& model) {
    const auto entry = resolve_locked(model.value("model_id", ""), model.value("version", ""));
    model["source"] = effective_abs(entry).string();
    model["resolved_version"] = entry.value("version", "");
    // Expose the package root so a self-contained scene export can bundle the
    // whole asset snapshot (meshes/textures), not only the model file.
    if (entry.value("packaged", false))
      model["package_dir"]
          = package_dir(entry.value("id", ""), entry.value("version", "")).string();
    if (model.value("id", "").empty()) model["id"] = model.value("model_id", "");
  });
  return scene;
}

// ------------------------------ helpers ---------------------------------

std::filesystem::path SimulationModelRegistry::default_storage_dir() {
  return simulation_data_dir("model_assets", "model_assets");
}

std::string SimulationModelRegistry::key(const std::string& id, const std::string& version) {
  return id + "@" + version;
}

int64_t SimulationModelRegistry::now_ms() {
  using clock = std::chrono::system_clock;
  return std::chrono::duration_cast<std::chrono::milliseconds>(clock::now().time_since_epoch())
      .count();
}

std::filesystem::path SimulationModelRegistry::normalize_path(const std::string& path_text) {
  std::filesystem::path path{path_text};
  if (path.is_relative()) path = std::filesystem::current_path() / path;
  return std::filesystem::weakly_canonical(path);
}

bool SimulationModelRegistry::is_newer_version(const nlohmann::json& candidate,
                                               const nlohmann::json& current) {
  const int cmp = compare_versions(candidate.value("version", ""), current.value("version", ""));
  if (cmp != 0) return cmp > 0;
  return candidate.value("registered_ms", int64_t{}) > current.value("registered_ms", int64_t{});
}

std::filesystem::path SimulationModelRegistry::package_dir(const std::string& id,
                                                           const std::string& version) const {
  return storage_dir_ / id / version;
}

std::filesystem::path SimulationModelRegistry::snapshot_package(
    const std::filesystem::path& asset_root, const std::filesystem::path& pkg_dir,
    uint64_t max_bytes) const {
  constexpr uint64_t kMaxFiles = 50000;
  // Narrow-with-asset_root/raise-max_asset_bytes guidance lives in copy_directory_tree's
  // own error message now (shared with export_scene's asset bundling).
  simulation::copy_directory_tree(asset_root, pkg_dir, max_bytes, kMaxFiles);
  return pkg_dir;
}

std::filesystem::path SimulationModelRegistry::convert_urdf(
    const std::filesystem::path& source, const std::filesystem::path& output) const {
  std::filesystem::create_directories(output.parent_path());
  char error[2048] = {};
  std::lock_guard xml_lock{simulation_mujoco_xml_mutex()};
  mjModel* model = mj_loadXML(source.string().c_str(), nullptr, error, sizeof(error));
  if (!model) throw std::invalid_argument(error[0] ? error : "failed to load URDF");
  const int saved = mj_saveLastXML(output.string().c_str(), model, error, sizeof(error));
  mj_deleteModel(model);
  mj_freeLastXML();
  if (!saved) throw std::runtime_error(error[0] ? error : "failed to convert URDF to MJCF");
  return std::filesystem::weakly_canonical(output);
}

std::filesystem::path SimulationModelRegistry::effective_abs(const nlohmann::json& entry) const {
  if (entry.value("packaged", false))
    return std::filesystem::weakly_canonical(storage_dir_ / entry.value("effective_rel", ""));
  return std::filesystem::path{entry.value("effective_path", "")};
}

nlohmann::json SimulationModelRegistry::hydrate(const nlohmann::json& entry) const {
  nlohmann::json out = entry;
  if (entry.value("packaged", false)) {
    out["effective_path"] = effective_abs(entry).string();
    out["source_path"]
        = std::filesystem::weakly_canonical(storage_dir_ / entry.value("source_rel", "")).string();
  }
  return out;
}

nlohmann::json SimulationModelRegistry::integrity_status(const nlohmann::json& entry) const {
  const auto path = effective_abs(entry);
  if (!std::filesystem::exists(path))
    return {{"ok", false}, {"reason", "effective model file is missing"}, {"path", path.string()}};

  try {
    // For packaged assets verify the whole snapshot (covers meshes/textures);
    // otherwise fall back to the effective model file digest.
    const std::string package_expected = entry.value("package_sha256", "");
    if (entry.value("packaged", false) && !package_expected.empty()) {
      const auto pkg = package_dir(entry.value("id", ""), entry.value("version", ""));

      // Cheap stat-only pre-check: if the package's file count/total bytes/newest
      // mtime match what was recorded at registration, nothing plausibly changed --
      // skip re-reading and re-hashing every file in the package. Only fall back to
      // the full sha256_directory (which every info()/verify() call used to pay for
      // unconditionally) when the fingerprint disagrees.
      if (entry.contains("package_file_count")) {
        const simulation::DirectoryFingerprint expected_fp{
            entry.value("package_file_count", uint64_t{0}),
            entry.value("package_total_bytes", uint64_t{0}),
            entry.value("package_mtime_tick", int64_t{0})};
        if (simulation::directory_fingerprint(pkg) == expected_fp) {
          return {{"ok", true}, {"reason", "package unchanged (fingerprint match)"}};
        }
      }

      const std::string actual = sha256_directory(pkg);
      if (actual == package_expected) return {{"ok", true}, {"reason", "package digest matches"}};
      return {{"ok", false},
              {"reason", "package digest mismatch (an asset changed since registration)"},
              {"scope", "package"},
              {"expected", package_expected},
              {"actual", actual}};
    }

    const std::string expected = entry.value("content_sha256", "");
    if (expected.empty()) return {{"ok", true}, {"reason", "no digest recorded"}};

    // Same cheap pre-check for the single-file (reference-mode) case.
    std::error_code ec;
    const auto live_bytes = static_cast<uint64_t>(std::filesystem::file_size(path, ec));
    if (!ec && entry.value("content_bytes", uint64_t{0}) == live_bytes
        && entry.value("content_mtime_tick", int64_t{0}) == file_mtime_tick(path)) {
      return {{"ok", true}, {"reason", "file unchanged (size/mtime match)"}};
    }

    const std::string actual = sha256_file(path);
    if (actual == expected) return {{"ok", true}, {"reason", "digest matches"}};
    return {{"ok", false},
            {"reason", "content digest mismatch (file changed since registration)"},
            {"scope", "model-file"},
            {"expected", expected},
            {"actual", actual}};
  } catch (const std::exception& e) {
    return {{"ok", false}, {"reason", e.what()}};
  }
}

nlohmann::json SimulationModelRegistry::resolve_locked(const std::string& id,
                                                       const std::string& version) const {
  if (!version.empty()) {
    auto it = entries_.find(key(id, version));
    if (it == entries_.end())
      throw std::out_of_range("model version not found: " + key(id, version));
    return it->second;
  }

  const nlohmann::json* latest = nullptr;
  for (const auto& [entry_key, entry] : entries_) {
    (void)entry_key;
    if (entry.value("id", "") != id) continue;
    if (!latest || is_newer_version(entry, *latest)) latest = &entry;
  }
  if (!latest) throw std::out_of_range("model not found: " + id);
  return *latest;
}

void SimulationModelRegistry::load_manifest() {
  std::lock_guard lock{mutex_};
  std::ifstream input(manifest_path_);
  if (!input) return;

  nlohmann::json manifest;
  try {
    manifest = nlohmann::json::parse(input);
  } catch (const std::exception&) {
    // Never take the plugin down over a corrupt manifest: back it up and start
    // from an empty registry.
    std::error_code ec;
    const auto backup = manifest_path_.string() + ".corrupt-" + std::to_string(now_ms());
    std::filesystem::rename(manifest_path_, backup, ec);
    return;
  }

  const nlohmann::json* models = nullptr;
  if (manifest.is_array()) {
    models = &manifest;  // legacy schema: a bare array of entries
  } else if (manifest.is_object() && manifest.contains("models") && manifest["models"].is_array()) {
    models = &manifest["models"];
  } else {
    return;  // unrecognised shape; treat as empty rather than throwing
  }

  for (const auto& entry : *models) {
    if (!entry.is_object()) continue;
    const std::string id = entry.value("id", "");
    const std::string version = entry.value("version", "");
    if (id.empty() || version.empty()) continue;
    entries_[key(id, version)] = entry;
  }
}

nlohmann::json SimulationModelRegistry::snapshot_manifest_locked() const {
  nlohmann::json models = nlohmann::json::array();
  for (const auto& [entry_key, entry] : entries_) {
    (void)entry_key;
    models.push_back(entry);
  }
  return {{"schema", 2}, {"models", std::move(models)}};
}

void SimulationModelRegistry::persist_manifest(nlohmann::json manifest, uint64_t generation) const {
  std::lock_guard io_lock{manifest_io_mutex_};
  if (generation < last_written_generation_) return;  // a newer snapshot already landed

  std::filesystem::create_directories(storage_dir_);
  const auto temporary = manifest_path_.string() + ".tmp";
  {
    std::ofstream output(temporary);
    if (!output) throw std::runtime_error("failed to write model registry manifest");
    output << manifest.dump(2) << '\n';
  }
  std::filesystem::rename(temporary, manifest_path_);
  last_written_generation_ = generation;
}
