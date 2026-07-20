#include "simulation_instance.hpp"

#include <algorithm>
#include <chrono>
#include <stdexcept>
#include <unordered_map>
#include <utility>
#include <vector>

#include "simulation_json_utils.hpp"
#include "simulation_mujoco_utils.hpp"
#include "simulation_visual.hpp"

namespace {

  // Shared by write_ctrl/write_qpos: accept either a numeric index or an object
  // name for `key`; returns -1 if it doesn't resolve to a valid index < limit.
  int resolve_index(const mjModel* model, int object_type, const std::string& key, int limit) {
    int index = -1;
    try {
      size_t consumed = 0;
      const int parsed = std::stoi(key, &consumed);
      if (consumed == key.size()) index = parsed;
    } catch (...) {
    }
    if (index < 0) index = mj_name2id(model, object_type, key.c_str());
    if (index < 0 || index >= limit) return -1;
    return index;
  }

}  // namespace

SimulationInstance::CreateResult SimulationInstance::create(std::string id, std::string model_path,
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

  scene_id_ = data.value("scene_id", scene_id_);
  compiled_model_id_ = data.value("compiled_model_id", compiled_model_id_);
  step_hz_ = clamp_rate(data.value("step_hz", step_hz_), step_hz_, 1.0, 5000.0);
  publish_hz_ = clamp_rate(data.value("publish_hz", publish_hz_), publish_hz_, 0.0, 200.0);
  // Merge into initial_config_ (like apply_runtime does) rather than wiping it: a later
  // configure() call that only touches e.g. scene_id must not drop a previously-set qpos.
  if (data.contains("qpos")) {
    assign_array(data_->qpos, model_->nq, data.at("qpos"), "qpos");
    initial_config_["qpos"] = data.at("qpos");
    needs_forward_ = true;
  }
  if (data.contains("qvel")) {
    assign_array(data_->qvel, model_->nv, data.at("qvel"), "qvel");
    initial_config_["qvel"] = data.at("qvel");
    needs_forward_ = true;
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

nlohmann::json SimulationInstance::step(int count, bool include_state) {
  const int steps = std::max(1, count);
  // Release the lock every kMaxStepsPerLock steps so a large batch (up to 100000)
  // doesn't make state()/stop()/write_ctrl() block for the whole call.
  constexpr int kMaxStepsPerLock = 256;
  int done = 0;
  while (done < steps) {
    std::lock_guard lock{mutex_};
    if (!model_ || !data_) {
      status_ = Status::Error;
      return state_locked();
    }
    if (stop_requested_) break;
    const int batch = std::min(kMaxStepsPerLock, steps - done);
    for (int i = 0; i < batch; ++i) {
      mj_step(model_, data_);
    }
    step_count_ += static_cast<uint64_t>(batch);
    done += batch;
  }

  std::lock_guard lock{mutex_};
  return state_locked(include_state);
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
  needs_forward_ = true;
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
    needs_forward_ = true;
  }
  if (runtime.contains("qvel")) {
    assign_array(data_->qvel, model_->nv, runtime.at("qvel"), "qvel");
    initial_config_["qvel"] = runtime.at("qvel");
    needs_forward_ = true;
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
    const int index = resolve_index(model_, mjOBJ_ACTUATOR, it.key(), model_->nu);
    if (index < 0) throw std::invalid_argument("actuator not found: " + it.key());
    data_->ctrl[index] = it.value().get<double>();
  }

  return state_locked();
}

nlohmann::json SimulationInstance::write_qpos(const nlohmann::json& data) {
  std::lock_guard lock{mutex_};
  if (!model_ || !data_) {
    status_ = Status::Error;
    return state_locked();
  }
  if (model_->njnt <= 0) throw std::invalid_argument("model has no joints");

  if (!data.contains("values") || !data["values"].is_object()) {
    throw std::invalid_argument("missing 'values' object (joint name/id -> position)");
  }

  for (auto it = data["values"].begin(); it != data["values"].end(); ++it) {
    const int index = resolve_index(model_, mjOBJ_JOINT, it.key(), model_->njnt);
    if (index < 0) throw std::invalid_argument("joint not found: " + it.key());
    const int type = model_->jnt_type[index];
    if (type != mjJNT_HINGE && type != mjJNT_SLIDE) {
      throw std::invalid_argument("joint is not scalar (hinge/slide): " + it.key());
    }
    double value = it.value().get<double>();
    if (model_->jnt_limited[index]) {
      value = std::clamp(value, model_->jnt_range[2 * index], model_->jnt_range[2 * index + 1]);
    }
    data_->qpos[model_->jnt_qposadr[index]] = value;
    // 拖关节是摆位操作：清掉该自由度残余速度，避免松手后继续漂移
    data_->qvel[model_->jnt_dofadr[index]] = 0.0;
  }

  // visual_model_locked() does the forward lazily on next access (needed for
  // the pose to show correctly even while paused, without paying for it here
  // if nothing ever queries the visual model before the next step/write).
  needs_forward_ = true;
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
    double step_hz = 100.0;
    double publish_hz = 10.0;
    bool errored = false;
    bool want_publish = false;
    // Populated under the lock (cheap raw copy) only on a publish tick; the
    // (more expensive) JSON construction happens after releasing the lock below,
    // so a synchronous RPC against this instance (state()/write_ctrl()/stop())
    // only ever waits on this thread's mj_step, not also on building/allocating
    // the snapshot JSON.
    std::string id_copy, status_copy, model_path_copy, scene_id_copy, compiled_model_id_copy;
    uint64_t step_count_copy = 0;
    double time_copy = 0.0;
    int nq = 0, nv = 0, nu = 0, nsensordata = 0;
    std::vector<mjtNum> qpos_copy, qvel_copy, ctrl_copy, sensordata_copy;

    {
      std::unique_lock lock{mutex_};
      cv_.wait_for(lock, std::chrono::milliseconds(100),
                   [this] { return stop_requested_ || status_ == Status::Running; });

      if (stop_requested_) break;
      if (status_ != Status::Running) continue;

      if (!model_ || !data_) {
        status_ = Status::Error;
        errored = true;
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
          want_publish = true;
          last_publish = now;
          id_copy = id_;
          status_copy = status_text(status_);
          model_path_copy = model_path_;
          scene_id_copy = scene_id_;
          compiled_model_id_copy = compiled_model_id_;
          step_count_copy = step_count_;
          time_copy = static_cast<double>(data_->time);
          nq = model_->nq;
          nv = model_->nv;
          nu = model_->nu;
          nsensordata = model_->nsensordata;
          qpos_copy.assign(data_->qpos, data_->qpos + nq);
          qvel_copy.assign(data_->qvel, data_->qvel + nv);
          ctrl_copy.assign(data_->ctrl, data_->ctrl + nu);
          sensordata_copy.assign(data_->sensordata, data_->sensordata + nsensordata);
        }
      }
    }

    nlohmann::json snapshot;
    if (errored) {
      snapshot = state();  // rare path; a fresh lock here is fine
    } else if (want_publish) {
      snapshot["id"] = id_copy;
      snapshot["status"] = status_copy;
      snapshot["model_path"] = model_path_copy;
      if (!scene_id_copy.empty()) snapshot["scene_id"] = scene_id_copy;
      if (!compiled_model_id_copy.empty()) snapshot["compiled_model_id"] = compiled_model_id_copy;
      snapshot["step_count"] = step_count_copy;
      snapshot["step_hz"] = step_hz;
      snapshot["publish_hz"] = publish_hz;
      snapshot["time"] = time_copy;
      snapshot["model"] = {{"nq", nq}, {"nv", nv}, {"nu", nu}, {"nsensordata", nsensordata}};
      snapshot["qpos"] = numeric_array(qpos_copy.data(), nq);
      snapshot["qvel"] = numeric_array(qvel_copy.data(), nv);
      snapshot["ctrl"] = numeric_array(ctrl_copy.data(), nu);
      snapshot["sensordata"] = numeric_array(sensordata_copy.data(), nsensordata);
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

nlohmann::json SimulationInstance::state_locked(bool include_arrays) const {
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
  if (include_arrays) {
    out["qpos"] = numeric_array(data_->qpos, model_->nq);
    out["qvel"] = numeric_array(data_->qvel, model_->nv);
    out["ctrl"] = numeric_array(data_->ctrl, model_->nu);
    out["sensordata"] = numeric_array(data_->sensordata, model_->nsensordata);
  }
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
    nlohmann::json joint = {
        {"id", i},
        {"name", object_name(model_, mjOBJ_JOINT, i)},
        {"type", model_->jnt_type[i]},
        {"body", object_name(model_, mjOBJ_BODY, model_->jnt_bodyid[i])},
        {"qpos_adr", model_->jnt_qposadr[i]},
        {"dof_adr", model_->jnt_dofadr[i]},
        {"limited", static_cast<bool>(model_->jnt_limited[i])},
    };
    if (model_->jnt_limited[i]) {
      joint["range"] = {model_->jnt_range[2 * i], model_->jnt_range[2 * i + 1]};
    }
    out["joints"].push_back(std::move(joint));
  }

  // body 树（含父链），前端用于把点选的 link 映射到最近的驱动关节
  out["bodies"] = nlohmann::json::array();
  for (int i = 0; i < model_->nbody; ++i) {
    out["bodies"].push_back({
        {"id", i},
        {"name", object_name(model_, mjOBJ_BODY, i)},
        {"parent_id", model_->body_parentid[i]},
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

  // mj_step already leaves body/geom world poses consistent, so only re-forward
  // when something touched qpos/qvel directly (configure/apply_runtime/reset/write_qpos)
  // since the last time this was computed.
  if (needs_forward_) {
    mj_forward(model_, data_);
    needs_forward_ = false;
  }
  out.update(simulation_visual_json(model_, data_, include_geometry));
  out["status"] = status_text(status_);
  out["time"] = data_->time;
  out["step_count"] = step_count_;
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
