#pragma once

#include <exception>
#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>
#include <unordered_set>

#include "plugin_core/sdk/plugin_logx.hpp"
#include "plugin_core/sdk/plugin_sdk.hpp"
#include "simulation_compiler.hpp"
#include "simulation_instance_manager.hpp"
#include "simulation_model_registry.hpp"
#include "simulation_model_validator.hpp"
#include "simulation_record_manager.hpp"
#include "simulation_scene_manager.hpp"
#include "simulation_task_manager.hpp"

class SimulationPlug final : public PluginBase {
public:
  explicit SimulationPlug() = default;
  ~SimulationPlug() override = default;

private:
  bool on_init(IPluginContext& ctx) noexcept override;
  bool on_start() noexcept override;
  void on_stop() noexcept override;
  void on_unload() noexcept override;

  JsonRpcRouter::RouteMap build_routes();
  PluginHttpResponse handle_http_rpc(const PluginHttpRequest& req);
  PluginHttpResponse handle_model_upload(const PluginHttpRequest& req);
  void handle_ws_message(const char* session_id, const void* data, size_t size,
                         PluginWsMessageType type);
  void handle_ws_open(const char* session_id);
  void handle_ws_close(const char* session_id);

  JsonRpcResult handle_scene_load(const nlohmann::json& data);
  JsonRpcResult handle_scene_create(const nlohmann::json& data);
  JsonRpcResult handle_scene_save(const nlohmann::json& data);
  JsonRpcResult handle_scene_unload(const nlohmann::json& data);
  JsonRpcResult handle_scene_update(const nlohmann::json& data);
  JsonRpcResult handle_scene_apply(const nlohmann::json& data);
  JsonRpcResult handle_scene_list(const nlohmann::json& data);
  JsonRpcResult handle_scene_info(const nlohmann::json& data);
  JsonRpcResult handle_scene_validate(const nlohmann::json& data);
  JsonRpcResult handle_scene_diff(const nlohmann::json& data);
  JsonRpcResult handle_scene_compile(const nlohmann::json& data);
  JsonRpcResult handle_scene_export(const nlohmann::json& data);
  JsonRpcResult handle_visual_scene(const nlohmann::json& data);
  JsonRpcResult handle_visual_model(const nlohmann::json& data);
  JsonRpcResult handle_instance_create(const nlohmann::json& data);
  JsonRpcResult handle_instance_recreate_from_scene(const nlohmann::json& data);
  JsonRpcResult handle_instance_start(const nlohmann::json& data);
  JsonRpcResult handle_instance_pause(const nlohmann::json& data);
  JsonRpcResult handle_instance_stop(const nlohmann::json& data);
  JsonRpcResult handle_instance_step(const nlohmann::json& data);
  JsonRpcResult handle_instance_reset(const nlohmann::json& data);
  JsonRpcResult handle_instance_apply_runtime(const nlohmann::json& data);
  JsonRpcResult handle_instance_state(const nlohmann::json& data);
  JsonRpcResult handle_instance_metadata(const nlohmann::json& data);
  JsonRpcResult handle_instance_list(const nlohmann::json& data);
  JsonRpcResult handle_instance_destroy(const nlohmann::json& data);
  JsonRpcResult handle_model_register(const nlohmann::json& data);
  JsonRpcResult handle_model_list(const nlohmann::json& data);
  JsonRpcResult handle_model_info(const nlohmann::json& data);
  JsonRpcResult handle_model_remove(const nlohmann::json& data);
  JsonRpcResult handle_model_cache_prune(const nlohmann::json& data);
  JsonRpcResult handle_model_verify(const nlohmann::json& data);
  JsonRpcResult handle_model_validate(const nlohmann::json& data);
  JsonRpcResult handle_model_inspect(const nlohmann::json& data);
  JsonRpcResult handle_model_visual(const nlohmann::json& data);
  JsonRpcResult handle_control_joint_state(const nlohmann::json& data);
  JsonRpcResult handle_control_sensor_state(const nlohmann::json& data);
  JsonRpcResult handle_control_write_ctrl(const nlohmann::json& data);
  JsonRpcResult handle_task_create(const nlohmann::json& data);
  JsonRpcResult handle_task_remove(const nlohmann::json& data);
  JsonRpcResult handle_task_list(const nlohmann::json& data);
  JsonRpcResult handle_task_info(const nlohmann::json& data);
  JsonRpcResult handle_task_run(const nlohmann::json& data);
  JsonRpcResult handle_record_start(const nlohmann::json& data);
  JsonRpcResult handle_record_stop(const nlohmann::json& data);
  JsonRpcResult handle_record_list(const nlohmann::json& data);
  JsonRpcResult handle_record_info(const nlohmann::json& data);
  JsonRpcResult handle_record_remove(const nlohmann::json& data);
  JsonRpcResult route_error(const char* action, const std::exception& e) const;
  nlohmann::json resolve_scene_models(const nlohmann::json& scene) const;
  void publish_state(const nlohmann::json& state) noexcept;

private:
  struct WsSubscription {
    std::unordered_set<std::string> instances;  // 含 "*" 表示订阅全部实例
    bool visual{false};                         // 推送时附带 geom 位姿（3D 预览用）
  };

  const std::string TAG = "仿真插件";
  std::shared_ptr<const JsonRpcRouter::RouteMap> routes_;
  std::mutex ws_mutex_;
  std::unordered_map<std::string, WsSubscription> ws_subscriptions_;
  SimulationSceneManager scenes_;
  SimulationModelRegistry model_registry_;
  SimulationCompiler compiler_;
  SimulationModelValidator model_validator_;
  SimulationInstanceManager instances_;
  SimulationTaskManager tasks_;
  SimulationRecordManager records_;
};