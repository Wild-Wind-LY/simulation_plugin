#pragma once

#include <mujoco/mujoco.h>

#include <condition_variable>
#include <cstdint>
#include <functional>
#include <memory>
#include <mutex>
#include <nlohmann/json.hpp>
#include <string>
#include <thread>

class SimulationInstance {
public:
  using ModelPtr = std::shared_ptr<mjModel>;

  enum class Status {
    Created,
    Running,
    Paused,
    Stopped,
    Error,
  };

  using PublishFn = std::function<void(const nlohmann::json&)>;

  struct CreateResult {
    std::unique_ptr<SimulationInstance> instance;
    std::string error;
  };

  static CreateResult create(std::string id, std::string model_path, ModelPtr shared_model = {});

  SimulationInstance(const SimulationInstance&) = delete;
  SimulationInstance& operator=(const SimulationInstance&) = delete;
  ~SimulationInstance();

  const std::string& id() const noexcept { return id_; }

  nlohmann::json configure(const nlohmann::json& data);
  nlohmann::json start(double step_hz, double publish_hz, PublishFn publisher);
  nlohmann::json pause();
  nlohmann::json stop();
  nlohmann::json step(int count, bool include_state = true);
  nlohmann::json reset();
  nlohmann::json apply_runtime(const nlohmann::json& data);
  nlohmann::json state() const;
  nlohmann::json metadata() const;
  nlohmann::json inspect_model() const;
  nlohmann::json joint_state() const;
  nlohmann::json sensor_state() const;
  nlohmann::json visual_model(bool include_geometry = true) const;
  nlohmann::json write_ctrl(const nlohmann::json& data);
  nlohmann::json write_qpos(const nlohmann::json& data);
  nlohmann::json write_equality(const nlohmann::json& data);

private:
  SimulationInstance(std::string id, std::string model_path, ModelPtr model, mjData* data);

  static const char* status_text(Status status) noexcept;
  static double clamp_rate(double value, double fallback, double min, double max);
  static void assign_array(mjtNum* target, int target_size, const nlohmann::json& values,
                           const char* field);

  void worker_loop(PublishFn publisher);
  void request_worker_stop();
  nlohmann::json state_locked(bool include_arrays = true) const;
  nlohmann::json metadata_locked() const;
  nlohmann::json joint_state_locked() const;
  nlohmann::json sensor_state_locked() const;
  nlohmann::json visual_model_locked(bool include_geometry) const;

  std::string id_;
  std::string model_path_;
  std::string scene_id_;
  std::string compiled_model_id_;
  nlohmann::json initial_config_;
  ModelPtr model_owner_;
  mjModel* model_{nullptr};
  mjData* data_{nullptr};
  Status status_{Status::Created};
  uint64_t step_count_{0};
  double step_hz_{100.0};
  double publish_hz_{10.0};
  bool stop_requested_{false};
  // Set whenever qpos/qvel is written directly (configure/apply_runtime/reset/write_qpos);
  // mj_step already leaves poses consistent, so visual_model_locked() only re-forwards
  // when this is set, instead of on every single call.
  mutable bool needs_forward_{true};
  mutable std::mutex mutex_;
  std::condition_variable cv_;
  std::thread worker_;
};
