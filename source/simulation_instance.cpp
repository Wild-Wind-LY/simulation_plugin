#include "simulation_instance.hpp"
#include "simulation_mujoco_utils.hpp"

#include <algorithm>
#include <chrono>
#include <stdexcept>
#include <unordered_map>
#include <utility>

namespace {

  std::string object_name(const mjModel* model, int object_type, int id) {
    const char* name = mj_id2name(model, object_type, id);
    return name ? std::string{name} : std::string{};
  }

  nlohmann::json numeric_array(const float* values, int count) {
    nlohmann::json out = nlohmann::json::array();
    if (!values || count <= 0) return out;

    for (int i = 0; i < count; ++i) {
      out.push_back(static_cast<double>(values[i]));
    }
    return out;
  }
  std::string geom_shape(int type) {
    switch (type) {
      case mjGEOM_PLANE:
        return "plane";
      case mjGEOM_SPHERE:
        return "sphere";
      case mjGEOM_CAPSULE:
        return "capsule";
      case mjGEOM_ELLIPSOID:
        return "ellipsoid";
      case mjGEOM_CYLINDER:
        return "cylinder";
      case mjGEOM_BOX:
        return "box";
      case mjGEOM_MESH:
        return "mesh";
      case mjGEOM_HFIELD:
        return "hfield";
      default:
        return "unsupported";
    }
  }
  nlohmann::json numeric_array(const mjtNum* values, int count) {
    nlohmann::json out = nlohmann::json::array();
    if (!values || count <= 0) return out;

    for (int i = 0; i < count; ++i) {
      out.push_back(static_cast<double>(values[i]));
    }
    return out;
  }

}  // namespace

SimulationInstance::CreateResult SimulationInstance::create(std::string id,
                                                            std::string model_path,
                                                            ModelPtr shared_model) {
  if (!shared_model) {
    char error[1024] = {};
    std::lock_guard xml_lock{simulation_mujoco_xml_mutex()};
    mjModel* loaded = mj_loadXML(model_path.c_str(), nullptr, error, sizeof(error));
    if (!loaded) {
      return {nullptr, error[0] ? error : "failed to load MuJoCo model"};
    }
    shared_model = ModelPtr(loaded, mj_deleteModel);
  }

  mjData* data = mj_makeData(shared_model.get());
  if (!data) return {nullptr, "failed to allocate MuJoCo data"};

  return {std::unique_ptr<SimulationInstance>(new SimulationInstance(
              std::move(id), std::move(model_path), std::move(shared_model), data)),
          {}};
}

SimulationInstance::SimulationInstance(std::string id, std::string model_path, ModelPtr model,
                                       mjData* data)
    : id_(std::move(id)),
      model_path_(std::move(model_path)),
      model_owner_(std::move(model)),
      model_(model_owner_.get()),
      data_(data) {}

SimulationInstance::~SimulationInstance() {
  request_worker_stop();
  if (data_) mj_deleteData(data_);
}

nlohmann::json SimulationInstance::configure(const nlohmann::json& data) {
  std::lock_guard lock{mutex_};
  if (!model_ || !data_) {
    status_ = Status::Error;
    return state_locked();
  }

  initial_config_ = nlohmann::json::object();
  scene_id_ = data.value("scene_id", scene_id_);
  compiled_model_id_ = data.value("compiled_model_id", compiled_model_id_);
  step_hz_ = clamp_rate(data.value("step_hz", step_hz_), step_hz_, 1.0, 5000.0);
  publish_hz_ = clamp_rate(data.value("publish_hz", publish_hz_), publish_hz_, 0.0, 200.0);
  if (data.contains("qpos")) {
    assign_array(data_->qpos, model_->nq, data.at("qpos"), "qpos");
    initial_config_["qpos"] = data.at("qpos");
  }
  if (data.contains("qvel")) {
    assign_array(data_->qvel, model_->nv, data.at("qvel"), "qvel");
    initial_config_["qvel"] = data.at("qvel");
  }
  if (data.contains("ctrl")) {
    assign_array(data_->ctrl, model_->nu, data.at("ctrl"), "ctrl");
    initial_config_["ctrl"] = data.at("ctrl");
  }
  return state_locked();
}

nlohmann::json SimulationInstance::start(double step_hz, double publish_hz, PublishFn publisher) {
  {
    std::lock_guard lock{mutex_};
    if (!model_ || !data_) {
      status_ = Status::Error;
      return state_locked();
    }

    step_hz_ = clamp_rate(step_hz, step_hz_, 1.0, 5000.0);
    publish_hz_ = clamp_rate(publish_hz, publish_hz_, 0.0, 200.0);
    status_ = Status::Running;
    stop_requested_ = false;

    if (!worker_.joinable()) {
      worker_ = std::thread([this, publisher = std::move(publisher)]() mutable {
        worker_loop(std::move(publisher));
      });
    }
  }

  cv_.notify_all();
  return state();
}

nlohmann::json SimulationInstance::pause() {
  std::lock_guard lock{mutex_};
  if (status_ == Status::Running) status_ = Status::Paused;
  return state_locked();
}

nlohmann::json SimulationInstance::stop() {
  request_worker_stop();

  std::lock_guard lock{mutex_};
  if (status_ != Status::Error) status_ = Status::Stopped;
  return state_locked();
}

nlohmann::json SimulationInstance::step(int count) {
  std::lock_guard lock{mutex_};
  if (!model_ || !data_) {
    status_ = Status::Error;
    return state_locked();
  }

  const int steps = std::max(1, count);
  for (int i = 0; i < steps; ++i) {
    mj_step(model_, data_);
  }
  step_count_ += static_cast<uint64_t>(steps);
  return state_locked();
}

nlohmann::json SimulationInstance::reset() {
  std::lock_guard lock{mutex_};
  if (!model_ || !data_) {
    status_ = Status::Error;
    return state_locked();
  }

  mj_resetData(model_, data_);
  if (initial_config_.contains("qpos")) {
    assign_array(data_->qpos, model_->nq, initial_config_.at("qpos"), "qpos");
  }
  if (initial_config_.contains("qvel")) {
    assign_array(data_->qvel, model_->nv, initial_config_.at("qvel"), "qvel");
  }
  if (initial_config_.contains("ctrl")) {
    assign_array(data_->ctrl, model_->nu, initial_config_.at("ctrl"), "ctrl");
  }
  step_count_ = 0;
  status_ = Status::Created;
  return state_locked();
}

nlohmann::json SimulationInstance::apply_runtime(const nlohmann::json& data) {
  std::lock_guard lock{mutex_};
  if (!model_ || !data_) {
    status_ = Status::Error;
    return state_locked();
  }

  const nlohmann::json& runtime
      = data.contains("defaults") && data["defaults"].is_object() ? data.at("defaults") : data;
  if (runtime.contains("step_hz")) {
    step_hz_ = clamp_rate(runtime.value("step_hz", step_hz_), step_hz_, 1.0, 5000.0);
  }
  if (runtime.contains("publish_hz")) {
    publish_hz_ = clamp_rate(runtime.value("publish_hz", publish_hz_), publish_hz_, 0.0, 200.0);
  }
  if (runtime.contains("qpos")) {
    assign_array(data_->qpos, model_->nq, runtime.at("qpos"), "qpos");
    initial_config_["qpos"] = runtime.at("qpos");
  }
  if (runtime.contains("qvel")) {
    assign_array(data_->qvel, model_->nv, runtime.at("qvel"), "qvel");
    initial_config_["qvel"] = runtime.at("qvel");
  }
  if (runtime.contains("ctrl")) {
    assign_array(data_->ctrl, model_->nu, runtime.at("ctrl"), "ctrl");
    initial_config_["ctrl"] = runtime.at("ctrl");
  }
  return state_locked();
}
nlohmann::json SimulationInstance::state() const {
  std::lock_guard lock{mutex_};
  return state_locked();
}

nlohmann::json SimulationInstance::metadata() const {
  std::lock_guard lock{mutex_};
  return metadata_locked();
}
nlohmann::json SimulationInstance::inspect_model() const {
  std::lock_guard lock{mutex_};
  auto out = metadata_locked();
  nlohmann::json warnings = nlohmann::json::array();
  if (!model_) {
    warnings.push_back("model is not loaded");
  } else {
    if (model_->nu == 0) warnings.push_back("model has no actuators; ctrl commands cannot move it");
    if (model_->njnt == 0) warnings.push_back("model has no joints");
    if (model_->nsensor == 0) warnings.push_back("model has no sensors");
  }
  out["controllable"] = model_ && model_->nu > 0;
  out["warnings"] = warnings;
  return out;
}

nlohmann::json SimulationInstance::joint_state() const {
  std::lock_guard lock{mutex_};
  return joint_state_locked();
}

nlohmann::json SimulationInstance::sensor_state() const {
  std::lock_guard lock{mutex_};
  return sensor_state_locked();
}

nlohmann::json SimulationInstance::visual_model(bool include_geometry) const {
  std::lock_guard lock{mutex_};
  return visual_model_locked(include_geometry);
}

nlohmann::json SimulationInstance::write_ctrl(const nlohmann::json& data) {
  std::lock_guard lock{mutex_};
  if (!model_ || !data_) {
    status_ = Status::Error;
    return state_locked();
  }
  if (data.contains("ctrl")) {
    assign_array(data_->ctrl, model_->nu, data.at("ctrl"), "ctrl");
    return state_locked();
  }
  if (model_->nu <= 0) throw std::invalid_argument("model has no actuators");

  if (!data.contains("values") || !data["values"].is_object()) {
    throw std::invalid_argument("missing 'ctrl' array or 'values' object");
  }

  for (auto it = data["values"].begin(); it != data["values"].end(); ++it) {
    int index = -1;
    try {
      size_t consumed = 0;
      const int parsed = std::stoi(it.key(), &consumed);
      if (consumed == it.key().size()) index = parsed;
    } catch (...) {
    }
    if (index < 0) index = mj_name2id(model_, mjOBJ_ACTUATOR, it.key().c_str());
    if (index < 0 || index >= model_->nu) {
      throw std::invalid_argument("actuator not found: " + it.key());
    }
    data_->ctrl[index] = it.value().get<double>();
  }

  return state_locked();
}
const char* SimulationInstance::status_text(Status status) noexcept {
  switch (status) {
    case Status::Created:
      return "created";
    case Status::Running:
      return "running";
    case Status::Paused:
      return "paused";
    case Status::Stopped:
      return "stopped";
    case Status::Error:
      return "error";
  }
  return "unknown";
}

double SimulationInstance::clamp_rate(double value, double fallback, double min, double max) {
  if (value <= 0.0 && min <= 0.0) return 0.0;
  if (value <= 0.0) return fallback;
  return std::clamp(value, min, max);
}

void SimulationInstance::assign_array(mjtNum* target, int target_size, const nlohmann::json& values,
                                      const char* field) {
  if (!values.is_array())
    throw std::invalid_argument(std::string("'") + field + "' must be an array");
  if (static_cast<int>(values.size()) > target_size) {
    throw std::invalid_argument(std::string("'") + field + "' length exceeds target size");
  }
  for (int i = 0; i < static_cast<int>(values.size()); ++i) {
    target[i] = values.at(i).get<double>();
  }
}

void SimulationInstance::worker_loop(PublishFn publisher) {
  using clock = std::chrono::steady_clock;
  auto last_publish = clock::now();

  while (true) {
    nlohmann::json snapshot;
    double step_hz = 100.0;
    double publish_hz = 10.0;

    {
      std::unique_lock lock{mutex_};
      cv_.wait_for(lock, std::chrono::milliseconds(100),
                   [this] { return stop_requested_ || status_ == Status::Running; });

      if (stop_requested_) break;
      if (status_ != Status::Running) continue;

      if (!model_ || !data_) {
        status_ = Status::Error;
        snapshot = state_locked();
      } else {
        mj_step(model_, data_);
        ++step_count_;
        step_hz = step_hz_;
        publish_hz = publish_hz_;

        const auto now = clock::now();
        const auto publish_interval = publish_hz > 0.0
                                          ? std::chrono::duration<double>(1.0 / publish_hz)
                                          : std::chrono::duration<double>::max();
        if (publish_hz > 0.0 && now - last_publish >= publish_interval) {
          snapshot = state_locked();
          last_publish = now;
        }
      }
    }

    if (!snapshot.is_null() && publisher) publisher(snapshot);

    const auto delay = std::chrono::duration<double>(1.0 / std::max(step_hz, 1.0));
    std::this_thread::sleep_for(delay);
  }
}

void SimulationInstance::request_worker_stop() {
  {
    std::lock_guard lock{mutex_};
    stop_requested_ = true;
  }
  cv_.notify_all();
  if (worker_.joinable()) worker_.join();
}

nlohmann::json SimulationInstance::state_locked() const {
  nlohmann::json out;
  out["id"] = id_;
  out["status"] = status_text(status_);
  out["model_path"] = model_path_;
  if (!scene_id_.empty()) out["scene_id"] = scene_id_;
  if (!compiled_model_id_.empty()) out["compiled_model_id"] = compiled_model_id_;
  out["step_count"] = step_count_;
  out["step_hz"] = step_hz_;
  out["publish_hz"] = publish_hz_;

  if (!model_ || !data_) {
    out["time"] = 0.0;
    return out;
  }

  out["time"] = static_cast<double>(data_->time);
  out["model"] = {
      {"nq", model_->nq},
      {"nv", model_->nv},
      {"nu", model_->nu},
      {"nsensordata", model_->nsensordata},
  };
  out["qpos"] = numeric_array(data_->qpos, model_->nq);
  out["qvel"] = numeric_array(data_->qvel, model_->nv);
  out["ctrl"] = numeric_array(data_->ctrl, model_->nu);
  out["sensordata"] = numeric_array(data_->sensordata, model_->nsensordata);
  return out;
}
nlohmann::json SimulationInstance::metadata_locked() const {
  nlohmann::json out;
  out["id"] = id_;
  out["model_path"] = model_path_;
  if (!scene_id_.empty()) out["scene_id"] = scene_id_;
  if (!compiled_model_id_.empty()) out["compiled_model_id"] = compiled_model_id_;

  if (!model_) return out;

  out["sizes"] = {
      {"nq", model_->nq},
      {"nv", model_->nv},
      {"nu", model_->nu},
      {"njnt", model_->njnt},
      {"nbody", model_->nbody},
      {"nsensor", model_->nsensor},
      {"nsensordata", model_->nsensordata},
  };

  out["joints"] = nlohmann::json::array();
  for (int i = 0; i < model_->njnt; ++i) {
    out["joints"].push_back({
        {"id", i},
        {"name", object_name(model_, mjOBJ_JOINT, i)},
        {"type", model_->jnt_type[i]},
        {"qpos_adr", model_->jnt_qposadr[i]},
        {"dof_adr", model_->jnt_dofadr[i]},
    });
  }

  out["actuators"] = nlohmann::json::array();
  for (int i = 0; i < model_->nu; ++i) {
    nlohmann::json actuator = {
        {"id", i},
        {"name", object_name(model_, mjOBJ_ACTUATOR, i)},
        {"ctrl_index", i},
    };
    if (model_->actuator_ctrllimited[i]) {
      actuator["ctrlrange"]
          = {model_->actuator_ctrlrange[2 * i], model_->actuator_ctrlrange[2 * i + 1]};
    }
    out["actuators"].push_back(std::move(actuator));
  }

  out["sensors"] = nlohmann::json::array();
  for (int i = 0; i < model_->nsensor; ++i) {
    out["sensors"].push_back({
        {"id", i},
        {"name", object_name(model_, mjOBJ_SENSOR, i)},
        {"type", model_->sensor_type[i]},
        {"data_adr", model_->sensor_adr[i]},
        {"data_dim", model_->sensor_dim[i]},
    });
  }

  return out;
}

nlohmann::json SimulationInstance::visual_model_locked(bool include_geometry) const {
  nlohmann::json out = {
      {"id", id_},    {"model_path", model_path_},         {"backend", "mujoco"},
      {"units", "m"}, {"coordinate_system", "mujoco_xyz"}, {"items", nlohmann::json::array()},
  };
  if (!scene_id_.empty()) out["scene_id"] = scene_id_;
  if (!model_ || !data_) return out;

  mj_forward(model_, data_);
  out["geometry_included"] = include_geometry;
  out["status"] = status_text(status_);
  out["time"] = data_->time;
  out["step_count"] = step_count_;
  out["ncon"] = data_->ncon;
  out["ngeom"] = model_->ngeom;
  out["model_counts"] = {
      {"body", model_->nbody},
      {"joint", model_->njnt},
      {"dof", model_->nv},
      {"geom", model_->ngeom},
      {"site", model_->nsite},
      {"actuator", model_->nu},
      {"sensor", model_->nsensor},
      {"tendon", model_->ntendon},
      {"equality", model_->neq},
      {"contact_pair", model_->npair},
      {"contact_exclude", model_->nexclude},
      {"mesh", model_->nmesh},
      {"height_field", model_->nhfield},
      {"material", model_->nmat},
      {"texture", model_->ntex},
      {"camera", model_->ncam},
      {"light", model_->nlight},
      {"nq", model_->nq},
      {"nv", model_->nv},
      {"na", model_->na},
      {"nu", model_->nu},
  };
  for (int i = 0; i < model_->ngeom; ++i) {
    const int type = model_->geom_type[i];
    const int body_id = model_->geom_bodyid[i];
    nlohmann::json item = {
        {"id", object_name(model_, mjOBJ_GEOM, i).empty() ? "geom_" + std::to_string(i)
                                                          : object_name(model_, mjOBJ_GEOM, i)},
        {"geom_id", i},
        {"body_id", body_id},
        {"body", body_id >= 0 ? object_name(model_, mjOBJ_BODY, body_id) : std::string{}},
        {"source", "mujoco.model"},
        {"kind", "geom"},
        {"shape", geom_shape(type)},
        {"geom_type", type},
        {"pos", numeric_array(data_->geom_xpos + 3 * i, 3)},
        {"xmat", numeric_array(data_->geom_xmat + 9 * i, 9)},
        {"size", numeric_array(model_->geom_size + 3 * i, 3)},
        {"rgba", numeric_array(model_->geom_rgba + 4 * i, 4)},
    };
    const int material_id = model_->geom_matid[i];
    item["material_id"] = material_id;
    if (material_id >= 0 && material_id < model_->nmat) {
      item["rgba"] = numeric_array(model_->mat_rgba + 4 * material_id, 4);
    }
    if (include_geometry && type == mjGEOM_MESH) {
      const int mesh_id = model_->geom_dataid[i];
      item["mesh_id"] = mesh_id;
      if (mesh_id >= 0 && mesh_id < model_->nmesh) {
        const int vertex_adr = model_->mesh_vertadr[mesh_id];
        const int vertex_count = model_->mesh_vertnum[mesh_id];
        const int face_adr = model_->mesh_faceadr[mesh_id];
        const int face_count = model_->mesh_facenum[mesh_id];
        item["vertices"] = numeric_array(model_->mesh_vert + 3 * vertex_adr,
                                         3 * vertex_count);
        nlohmann::json faces = nlohmann::json::array();
        for (int face = 0; face < face_count; ++face) {
          const int* indices = model_->mesh_face + 3 * (face_adr + face);
          faces.push_back({indices[0], indices[1], indices[2]});
        }
        item["faces"] = std::move(faces);
      }
    } else if (include_geometry && type == mjGEOM_HFIELD) {
      const int hfield_id = model_->geom_dataid[i];
      item["hfield_id"] = hfield_id;
      if (hfield_id >= 0 && hfield_id < model_->nhfield) {
        const int rows = model_->hfield_nrow[hfield_id];
        const int columns = model_->hfield_ncol[hfield_id];
        const int data_adr = model_->hfield_adr[hfield_id];
        item["hfield"] = {
            {"rows", rows},
            {"columns", columns},
            {"size", numeric_array(model_->hfield_size + 4 * hfield_id, 4)},
            {"data", numeric_array(model_->hfield_data + data_adr, rows * columns)},
        };
      }
    }
    out["items"].push_back(std::move(item));
  }
  return out;
}
nlohmann::json SimulationInstance::joint_state_locked() const {
  nlohmann::json out = {{"id", id_}, {"joints", nlohmann::json::array()}};
  if (!model_ || !data_) return out;

  for (int i = 0; i < model_->njnt; ++i) {
    const int qpos_adr = model_->jnt_qposadr[i];
    const int dof_adr = model_->jnt_dofadr[i];
    const int qpos_dim
        = i + 1 < model_->njnt ? model_->jnt_qposadr[i + 1] - qpos_adr : model_->nq - qpos_adr;
    const int dof_dim
        = i + 1 < model_->njnt ? model_->jnt_dofadr[i + 1] - dof_adr : model_->nv - dof_adr;
    out["joints"].push_back({
        {"id", i},
        {"name", object_name(model_, mjOBJ_JOINT, i)},
        {"type", model_->jnt_type[i]},
        {"qpos_adr", qpos_adr},
        {"dof_adr", dof_adr},
        {"qpos", numeric_array(data_->qpos + qpos_adr, qpos_dim)},
        {"qvel", numeric_array(data_->qvel + dof_adr, dof_dim)},
    });
  }
  return out;
}

nlohmann::json SimulationInstance::sensor_state_locked() const {
  nlohmann::json out = {{"id", id_}, {"sensors", nlohmann::json::array()}};
  if (!model_ || !data_) return out;

  for (int i = 0; i < model_->nsensor; ++i) {
    const int adr = model_->sensor_adr[i];
    const int dim = model_->sensor_dim[i];
    out["sensors"].push_back({
        {"id", i},
        {"name", object_name(model_, mjOBJ_SENSOR, i)},
        {"type", model_->sensor_type[i]},
        {"data_adr", adr},
        {"data_dim", dim},
        {"data", numeric_array(data_->sensordata + adr, dim)},
    });
  }
  return out;
}
