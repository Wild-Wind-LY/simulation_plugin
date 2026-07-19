#include "simulation_plug.hpp"

#include <algorithm>
#include <chrono>
#include <exception>
#include <filesystem>
#include <fstream>
#include <string_view>
#include <utility>
#include <vector>

#include "simulation_paths.hpp"
#include "simulation_visual.hpp"

bool SimulationPlug::on_init(IPluginContext& ctx) noexcept {
  // 设置插件全局日志
  LOGX_GLOBAL_CLASS(PLUGIN_LOG_NAME)::Construct();
  G_LOG()->set(logger());

  LOG_INFO("[{}] Initialized!", TAG);
  return true;
}

bool SimulationPlug::on_start() noexcept {
  routes_ = std::make_shared<const JsonRpcRouter::RouteMap>(build_routes());

  // 路由声明交给 Host：方法检查、404/405、CORS、per-route 指标都由 HTTP Gateway 完成
  PluginHttpEndpointOptions http_options;
  http_options.methods = {"POST"};
  http_options.routes.reserve(routes_->size() + 1);
  for (const auto& entry : *routes_) {
    http_options.routes.push_back(PluginHttpRoute{"/" + entry.first, {"POST"}});
  }
  // multipart 模型上传（浏览器直传模型 + mesh，注册进资产库）
  http_options.routes.push_back(PluginHttpRoute{"/model.upload", {"POST"}});
  http_options.max_body_size = 256 * 1024 * 1024;
  // JSON 请求超过 8 MiB 会落盘并被 handler 拒绝（同旧行为）；大 body 只允许 multipart 上传
  http_options.max_memory_body_size = 8 * 1024 * 1024;
  http_options.max_memory_part_size = 8 * 1024 * 1024;
  http_options.upload_timeout_ms = 120000;
  http_options.timeout_ms = 30000;
  http_options.max_concurrency = 16;
  // visual.model 全量几何（mesh 顶点/面片 JSON）会超过 Host 默认 16 MiB 的响应体上限
  http_options.max_response_body_size = 256 * 1024 * 1024;
  // 大响应给足发送预算（默认 30s）；并发下载沿用 Host 默认 4 路
  http_options.download_timeout_ms = 60000;
  if (!register_http_endpoint(
          "simulation", [this](const PluginHttpRequest& req) { return handle_http_rpc(req); },
          std::move(http_options))) {
    LOG_ERROR("[{}] Failed to register simulation http endpoint", TAG);
    return false;
  }

  // 实时状态推送：ws://<host>/backend/plugin-ws/simulation
  PluginWsEndpointOptions ws_options;
  ws_options.max_receive_message_size = 64 * 1024;
  ws_options.max_send_message_size = 8 * 1024 * 1024;
  ws_options.max_sessions = 32;
  ws_options.max_send_queue_messages = 32;
  ws_options.max_send_queue_bytes = 8 * 1024 * 1024;
  if (!register_ws_endpoint(
          "simulation",
          [this](const char* session_id, const void* data, size_t size, PluginWsMessageType type) {
            handle_ws_message(session_id, data, size, type);
          },
          [this](const char* session_id) { handle_ws_open(session_id); },
          [this](const char* session_id) { handle_ws_close(session_id); }, ws_options)) {
    LOG_ERROR("[{}] Failed to register simulation ws endpoint", TAG);
    return false;
  }

  LOG_INFO(
      "[{}] http: POST /backend/plugin-http/simulation/<module> ({} routes) | ws: "
      "/backend/plugin-ws/simulation",
      TAG, routes_->size());

  LOG_INFO("[{}] Started!", TAG);
  return true;
}

JsonRpcRouter::RouteMap SimulationPlug::build_routes() {
  return {
      {"scene.load", [this](const nlohmann::json& data) { return handle_scene_load(data); }},
      {"scene.create", [this](const nlohmann::json& data) { return handle_scene_create(data); }},
      {"scene.save", [this](const nlohmann::json& data) { return handle_scene_save(data); }},
      {"scene.unload", [this](const nlohmann::json& data) { return handle_scene_unload(data); }},
      {"scene.update", [this](const nlohmann::json& data) { return handle_scene_update(data); }},
      {"scene.apply", [this](const nlohmann::json& data) { return handle_scene_apply(data); }},
      {"scene.list", [this](const nlohmann::json& data) { return handle_scene_list(data); }},
      {"scene.info", [this](const nlohmann::json& data) { return handle_scene_info(data); }},
      {"scene.validate",
       [this](const nlohmann::json& data) { return handle_scene_validate(data); }},
      {"scene.diff", [this](const nlohmann::json& data) { return handle_scene_diff(data); }},
      {"scene.compile", [this](const nlohmann::json& data) { return handle_scene_compile(data); }},
      {"scene.export", [this](const nlohmann::json& data) { return handle_scene_export(data); }},
      {"visual.scene", [this](const nlohmann::json& data) { return handle_visual_scene(data); }},
      {"visual.model", [this](const nlohmann::json& data) { return handle_visual_model(data); }},
      {"instance.create",
       [this](const nlohmann::json& data) { return handle_instance_create(data); }},
      {"instance.recreate_from_scene",
       [this](const nlohmann::json& data) { return handle_instance_recreate_from_scene(data); }},
      {"instance.start",
       [this](const nlohmann::json& data) { return handle_instance_start(data); }},
      {"instance.pause",
       [this](const nlohmann::json& data) { return handle_instance_pause(data); }},
      {"instance.stop", [this](const nlohmann::json& data) { return handle_instance_stop(data); }},
      {"instance.step", [this](const nlohmann::json& data) { return handle_instance_step(data); }},
      {"instance.reset",
       [this](const nlohmann::json& data) { return handle_instance_reset(data); }},
      {"instance.apply_runtime",
       [this](const nlohmann::json& data) { return handle_instance_apply_runtime(data); }},
      {"instance.state",
       [this](const nlohmann::json& data) { return handle_instance_state(data); }},
      {"instance.metadata",
       [this](const nlohmann::json& data) { return handle_instance_metadata(data); }},
      {"instance.destroy",
       [this](const nlohmann::json& data) { return handle_instance_destroy(data); }},
      {"instance.list", [this](const nlohmann::json& data) { return handle_instance_list(data); }},
      {"task.create", [this](const nlohmann::json& data) { return handle_task_create(data); }},
      {"task.remove", [this](const nlohmann::json& data) { return handle_task_remove(data); }},
      {"task.list", [this](const nlohmann::json& data) { return handle_task_list(data); }},
      {"task.info", [this](const nlohmann::json& data) { return handle_task_info(data); }},
      {"task.run", [this](const nlohmann::json& data) { return handle_task_run(data); }},
      {"record.start", [this](const nlohmann::json& data) { return handle_record_start(data); }},
      {"record.stop", [this](const nlohmann::json& data) { return handle_record_stop(data); }},
      {"record.list", [this](const nlohmann::json& data) { return handle_record_list(data); }},
      {"record.info", [this](const nlohmann::json& data) { return handle_record_info(data); }},
      {"record.remove", [this](const nlohmann::json& data) { return handle_record_remove(data); }},
      {"model.register",
       [this](const nlohmann::json& data) { return handle_model_register(data); }},
      {"model.list", [this](const nlohmann::json& data) { return handle_model_list(data); }},
      {"model.info", [this](const nlohmann::json& data) { return handle_model_info(data); }},
      {"model.remove", [this](const nlohmann::json& data) { return handle_model_remove(data); }},
      {"model.cache_prune",
       [this](const nlohmann::json& data) { return handle_model_cache_prune(data); }},
      {"model.verify", [this](const nlohmann::json& data) { return handle_model_verify(data); }},
      {"model.validate",
       [this](const nlohmann::json& data) { return handle_model_validate(data); }},
      {"model.inspect", [this](const nlohmann::json& data) { return handle_model_inspect(data); }},
      {"model.visual", [this](const nlohmann::json& data) { return handle_model_visual(data); }},
      {"control.joint_state",
       [this](const nlohmann::json& data) { return handle_control_joint_state(data); }},
      {"control.sensor_state",
       [this](const nlohmann::json& data) { return handle_control_sensor_state(data); }},
      {"control.write_ctrl",
       [this](const nlohmann::json& data) { return handle_control_write_ctrl(data); }},
      {"control.write_qpos",
       [this](const nlohmann::json& data) { return handle_control_write_qpos(data); }},

  };
}

void SimulationPlug::on_stop() noexcept {
  records_.stop_all();
  instances_.stop_all();
  {
    std::lock_guard<std::mutex> lock(ws_mutex_);
    ws_subscriptions_.clear();
  }
  LOG_INFO("[{}] Stopped!", TAG);
}

void SimulationPlug::on_unload() noexcept {
  LOG_INFO("[{}] onUnload called!", TAG);

  records_.stop_all();
  instances_.stop_all();
  LOGX_GLOBAL_CLASS(PLUGIN_LOG_NAME)::Destruct();
}

JsonRpcResult SimulationPlug::handle_scene_load(const nlohmann::json& data) {
  try {
    return JsonRpcResult::ok("scene loaded", scenes_.load(data));
  } catch (const std::exception& e) {
    return route_error("scene.load", e);
  }
}

JsonRpcResult SimulationPlug::handle_scene_create(const nlohmann::json& data) {
  try {
    return JsonRpcResult::ok("scene created", scenes_.create(data));
  } catch (const std::exception& e) {
    return route_error("scene.create", e);
  }
}

JsonRpcResult SimulationPlug::handle_scene_save(const nlohmann::json& data) {
  try {
    return JsonRpcResult::ok("scene saved", scenes_.save(data));
  } catch (const std::exception& e) {
    return route_error("scene.save", e);
  }
}

JsonRpcResult SimulationPlug::handle_scene_unload(const nlohmann::json& data) {
  try {
    return JsonRpcResult::ok("scene unloaded", scenes_.unload(data));
  } catch (const std::exception& e) {
    return route_error("scene.unload", e);
  }
}

JsonRpcResult SimulationPlug::handle_scene_update(const nlohmann::json& data) {
  try {
    return JsonRpcResult::ok("scene updated", scenes_.update(data));
  } catch (const std::exception& e) {
    return route_error("scene.update", e);
  }
}

JsonRpcResult SimulationPlug::handle_scene_apply(const nlohmann::json& data) {
  try {
    const std::string scene_id = data.value("scene_id", data.value("id", ""));
    const std::string instance_id = data.value("instance_id", data.value("id", ""));
    if (scene_id.empty()) throw std::invalid_argument("missing 'scene_id' or 'id'");
    if (instance_id.empty()) throw std::invalid_argument("missing 'instance_id' or 'id'");
    if (!data.contains("new_scene") || !data["new_scene"].is_object()) {
      throw std::invalid_argument("scene.apply missing 'new_scene'");
    }

    auto old_scene = scenes_.info({{"id", scene_id}});
    auto new_scene = old_scene;
    for (auto item = data.at("new_scene").begin(); item != data.at("new_scene").end(); ++item) {
      new_scene[item.key()] = item.value();
    }
    new_scene["id"] = scene_id;

    auto resolved_new_scene = resolve_scene_models(new_scene);
    auto validation = compiler_.validate_scene(resolved_new_scene);
    if (!validation.value("valid", false)) {
      return JsonRpcResult::ok("scene apply validation failed",
                               {{"applied", false}, {"validation", validation}});
    }

    auto diff = compiler_.diff_scenes(old_scene, new_scene);
    auto updated_scene = scenes_.update(new_scene);
    nlohmann::json result = {
        {"scene_id", scene_id},
        {"instance_id", instance_id},
        {"diff", diff},
        {"scene", updated_scene},
    };

    if (diff.value("requires_recreate", false)) {
      auto compiled = compiler_.compile_scene(resolve_scene_models(updated_scene));
      auto config = compiler_.build_instance_config({{"id", instance_id}, {"scene_id", scene_id}},
                                                    compiled);
      auto model = compiler_.get_compiled_model(compiled.value("compiled_model_id", ""));
      auto recreated = instances_.recreate(config, std::move(model));
      publish_state(recreated["state"]);
      result["applied"] = true;
      result["mode"] = "recreate";
      result["instance"] = recreated;
      return JsonRpcResult::ok("scene applied with recreate", std::move(result));
    }

    nlohmann::json runtime = updated_scene.value("defaults", nlohmann::json::object());
    runtime["id"] = instance_id;
    auto state = instances_.apply_runtime(runtime);
    publish_state(state);
    result["applied"] = true;
    result["mode"] = "runtime";
    result["state"] = state;
    return JsonRpcResult::ok("scene applied at runtime", std::move(result));
  } catch (const std::exception& e) {
    return route_error("scene.apply", e);
  }
}
JsonRpcResult SimulationPlug::handle_scene_list(const nlohmann::json& data) {
  try {
    (void)data;
    return JsonRpcResult::ok("scene list", scenes_.list());
  } catch (const std::exception& e) {
    return route_error("scene.list", e);
  }
}

JsonRpcResult SimulationPlug::handle_scene_info(const nlohmann::json& data) {
  try {
    return JsonRpcResult::ok("scene info", scenes_.info(data));
  } catch (const std::exception& e) {
    return route_error("scene.info", e);
  }
}

JsonRpcResult SimulationPlug::handle_scene_validate(const nlohmann::json& data) {
  try {
    nlohmann::json scene = data;
    if (data.contains("id") && data["id"].is_string()) scene = scenes_.info(data);
    return JsonRpcResult::ok("scene validation",
                             compiler_.validate_scene(resolve_scene_models(scene)));
  } catch (const std::exception& e) {
    return route_error("scene.validate", e);
  }
}

JsonRpcResult SimulationPlug::handle_scene_diff(const nlohmann::json& data) {
  try {
    nlohmann::json old_scene;
    nlohmann::json new_scene;
    if (data.contains("old_scene") && data.contains("new_scene")) {
      old_scene = data.at("old_scene");
      new_scene = data.at("new_scene");
    } else {
      const std::string from = data.value("from", data.value("old_id", ""));
      const std::string to = data.value("to", data.value("new_id", ""));
      if (from.empty() || to.empty()) {
        throw std::invalid_argument("scene.diff requires old_scene/new_scene or from/to ids");
      }
      old_scene = scenes_.info({{"id", from}});
      new_scene = scenes_.info({{"id", to}});
    }
    return JsonRpcResult::ok("scene diff", compiler_.diff_scenes(old_scene, new_scene));
  } catch (const std::exception& e) {
    return route_error("scene.diff", e);
  }
}
JsonRpcResult SimulationPlug::handle_scene_compile(const nlohmann::json& data) {
  try {
    const auto scene = scenes_.info(data);
    auto result = compiler_.compile_scene(resolve_scene_models(scene));
    // include_visual: 无需创建实例即可拿到编译几何，编辑器用它渲染真实模型。
    // 临时 mjData 只做一次 mj_forward 取世界位姿，用完即弃。
    if (data.value("include_visual", false)) {
      const bool include_geometry = data.value("include_geometry", true);
      if (auto model
          = compiler_.get_compiled_model(result.value("compiled_model_id", std::string{}))) {
        if (mjData* tmp = mj_makeData(model.get())) {
          // 预览应用场景 defaults.qpos，让初始关节角与实例创建后一致（宽容截断，静默跳过非法值）
          const nlohmann::json defaults = scene.value("defaults", nlohmann::json::object());
          const nlohmann::json qpos = defaults.value("qpos", nlohmann::json::array());
          if (qpos.is_array() && !qpos.empty()) {
            const int count = std::min<int>(model->nq, static_cast<int>(qpos.size()));
            for (int i = 0; i < count; ++i) {
              if (qpos[i].is_number()) tmp->qpos[i] = qpos[i].get<double>();
            }
          }
          mj_forward(model.get(), tmp);
          result["visual"] = simulation_visual_json(model.get(), tmp, include_geometry);
          mj_deleteData(tmp);
        }
      }
    }
    return JsonRpcResult::ok("scene compiled", std::move(result));
  } catch (const std::exception& e) {
    return route_error("scene.compile", e);
  }
}

JsonRpcResult SimulationPlug::handle_scene_export(const nlohmann::json& data) {
  try {
    const auto scene = scenes_.info(data);
    return JsonRpcResult::ok("scene exported", compiler_.export_scene(resolve_scene_models(scene),
                                                                      data.value("out_dir", ""),
                                                                      data.value("flatten", true)));
  } catch (const std::exception& e) {
    return route_error("scene.export", e);
  }
}

JsonRpcResult SimulationPlug::handle_visual_scene(const nlohmann::json& data) {
  try {
    nlohmann::json scene;
    if (data.contains("scene") && data["scene"].is_object()) {
      scene = data["scene"];
    } else {
      nlohmann::json lookup = data;
      if (!lookup.contains("id") && lookup.contains("scene_id")) lookup["id"] = lookup["scene_id"];
      scene = scenes_.info(lookup);
    }
    return JsonRpcResult::ok("visual scene",
                             compiler_.build_visual_scene(resolve_scene_models(scene)));
  } catch (const std::exception& e) {
    return route_error("visual.scene", e);
  }
}

JsonRpcResult SimulationPlug::handle_visual_model(const nlohmann::json& data) {
  try {
    return JsonRpcResult::ok("visual model", instances_.visual_model(data));
  } catch (const std::exception& e) {
    return route_error("visual.model", e);
  }
}

JsonRpcResult SimulationPlug::handle_instance_create(const nlohmann::json& data) {
  try {
    auto config = data;
    SimulationCompiler::ModelPtr shared_model;
    const std::string scene_id = data.value("scene_id", "");
    if (!scene_id.empty()) {
      auto scene = scenes_.info({{"id", scene_id}});
      auto compiled = compiler_.compile_scene(resolve_scene_models(scene));
      config = compiler_.build_instance_config(data, compiled);
      shared_model = compiler_.get_compiled_model(compiled.value("compiled_model_id", ""));
    } else {
      auto [compiled_model_id, model] = compiler_.get_or_load_model(data.value("model_path", ""));
      config["compiled_model_id"] = compiled_model_id;
      shared_model = std::move(model);
    }
    auto state = instances_.create(config, std::move(shared_model));
    publish_state(state);
    return JsonRpcResult::ok("instance created", std::move(state));
  } catch (const std::exception& e) {
    return route_error("instance.create", e);
  }
}

JsonRpcResult SimulationPlug::handle_instance_recreate_from_scene(const nlohmann::json& data) {
  try {
    const std::string scene_id = data.value("scene_id", "");
    if (scene_id.empty()) throw std::invalid_argument("missing 'scene_id'");
    auto scene = scenes_.info({{"id", scene_id}});
    auto compiled = compiler_.compile_scene(resolve_scene_models(scene));
    auto config = compiler_.build_instance_config(data, compiled);
    auto model = compiler_.get_compiled_model(compiled.value("compiled_model_id", ""));
    auto result = instances_.recreate(config, std::move(model));
    publish_state(result["state"]);
    if (data.value("start", false)) {
      auto state
          = instances_.start({{"id", data.value("id", "")},
                              {"step_hz", config.value("step_hz", 100.0)},
                              {"publish_hz", config.value("publish_hz", 10.0)}},
                             [this](const nlohmann::json& snapshot) { publish_state(snapshot); });
      result["state"] = state;
      publish_state(state);
    }
    return JsonRpcResult::ok("instance recreated from scene", std::move(result));
  } catch (const std::exception& e) {
    return route_error("instance.recreate_from_scene", e);
  }
}
JsonRpcResult SimulationPlug::handle_instance_start(const nlohmann::json& data) {
  try {
    auto state = instances_.start(
        data, [this](const nlohmann::json& snapshot) { publish_state(snapshot); });
    publish_state(state);
    return JsonRpcResult::ok("instance started", std::move(state));
  } catch (const std::exception& e) {
    return route_error("instance.start", e);
  }
}

JsonRpcResult SimulationPlug::handle_instance_pause(const nlohmann::json& data) {
  try {
    auto state = instances_.pause(data);
    publish_state(state);
    return JsonRpcResult::ok("instance paused", std::move(state));
  } catch (const std::exception& e) {
    return route_error("instance.pause", e);
  }
}

JsonRpcResult SimulationPlug::handle_instance_stop(const nlohmann::json& data) {
  try {
    auto state = instances_.stop(data);
    publish_state(state);
    return JsonRpcResult::ok("instance stopped", std::move(state));
  } catch (const std::exception& e) {
    return route_error("instance.stop", e);
  }
}

JsonRpcResult SimulationPlug::handle_instance_step(const nlohmann::json& data) {
  try {
    auto state = instances_.step(data);
    publish_state(state);
    return JsonRpcResult::ok("instance stepped", std::move(state));
  } catch (const std::exception& e) {
    return route_error("instance.step", e);
  }
}

JsonRpcResult SimulationPlug::handle_instance_reset(const nlohmann::json& data) {
  try {
    auto state = instances_.reset(data);
    publish_state(state);
    return JsonRpcResult::ok("instance reset", std::move(state));
  } catch (const std::exception& e) {
    return route_error("instance.reset", e);
  }
}

JsonRpcResult SimulationPlug::handle_instance_apply_runtime(const nlohmann::json& data) {
  try {
    auto state = instances_.apply_runtime(data);
    publish_state(state);
    return JsonRpcResult::ok("instance runtime applied", std::move(state));
  } catch (const std::exception& e) {
    return route_error("instance.apply_runtime", e);
  }
}
JsonRpcResult SimulationPlug::handle_instance_state(const nlohmann::json& data) {
  try {
    auto state = instances_.state(data);
    return JsonRpcResult::ok("instance state", std::move(state));
  } catch (const std::exception& e) {
    return route_error("instance.state", e);
  }
}

JsonRpcResult SimulationPlug::handle_instance_metadata(const nlohmann::json& data) {
  try {
    return JsonRpcResult::ok("instance metadata", instances_.metadata(data));
  } catch (const std::exception& e) {
    return route_error("instance.metadata", e);
  }
}
JsonRpcResult SimulationPlug::handle_instance_list(const nlohmann::json& data) {
  try {
    (void)data;
    return JsonRpcResult::ok("instance list", instances_.list());
  } catch (const std::exception& e) {
    return route_error("instance.list", e);
  }
}

JsonRpcResult SimulationPlug::handle_instance_destroy(const nlohmann::json& data) {
  try {
    auto result = instances_.destroy(data);
    nlohmann::json event = {
        {"id", result.value("id", "")},
        {"status", "destroyed"},
        {"destroyed", true},
    };
    publish_state(event);
    return JsonRpcResult::ok("instance destroyed", std::move(result));
  } catch (const std::exception& e) {
    return route_error("instance.destroy", e);
  }
}

JsonRpcResult SimulationPlug::handle_model_register(const nlohmann::json& data) {
  try {
    return JsonRpcResult::ok("model registered", model_registry_.register_model(data));
  } catch (const std::exception& e) {
    return route_error("model.register", e);
  }
}

JsonRpcResult SimulationPlug::handle_model_list(const nlohmann::json& data) {
  try {
    (void)data;
    return JsonRpcResult::ok("model list", model_registry_.list());
  } catch (const std::exception& e) {
    return route_error("model.list", e);
  }
}

JsonRpcResult SimulationPlug::handle_model_info(const nlohmann::json& data) {
  try {
    return JsonRpcResult::ok("model info", model_registry_.info(data));
  } catch (const std::exception& e) {
    return route_error("model.info", e);
  }
}

JsonRpcResult SimulationPlug::handle_model_visual(const nlohmann::json& data) {
  try {
    // 独立编译注册资产（不需要场景/实例），返回默认位形（qpos0）的几何。
    // 编辑器用它生成资产库缩略图和首次拖拽的即时预览。
    const auto info = model_registry_.info(data);
    const std::string path = info.value("effective_path", "");
    if (path.empty()) throw std::invalid_argument("model has no effective file");
    auto [cache_model_id, model] = compiler_.get_or_load_model(path);
    if (!model) throw std::runtime_error("failed to load model: " + path);
    nlohmann::json result = {
        {"id", info.value("id", "")},
        {"version", info.value("version", "")},
        {"format", info.value("format", "")},
        {"cache_model_id", cache_model_id},
    };
    if (mjData* tmp = mj_makeData(model.get())) {
      mj_forward(model.get(), tmp);
      result.update(simulation_visual_json(model.get(), tmp, data.value("include_geometry", true)));
      mj_deleteData(tmp);
    }
    return JsonRpcResult::ok("model visual", std::move(result));
  } catch (const std::exception& e) {
    return route_error("model.visual", e);
  }
}

JsonRpcResult SimulationPlug::handle_model_remove(const nlohmann::json& data) {
  try {
    const std::string id = data.value("id", data.value("model_id", ""));
    const std::string version = data.value("version", "");
    std::string latest_version;
    try {
      latest_version = model_registry_.info({{"id", id}}).value("version", "");
    } catch (const std::exception&) {
      // No registered versions to resolve a latest from; leave it empty so the
      // reference check stays conservative.
    }
    if (scenes_.references_model(id, version, latest_version))
      throw std::invalid_argument("model version is referenced by a loaded scene");
    return JsonRpcResult::ok("model removed", model_registry_.remove(data));
  } catch (const std::exception& e) {
    return route_error("model.remove", e);
  }
}

JsonRpcResult SimulationPlug::handle_model_cache_prune(const nlohmann::json& data) {
  try {
    (void)data;
    return JsonRpcResult::ok("model cache pruned", {{"removed", compiler_.prune_unused_models()}});
  } catch (const std::exception& e) {
    return route_error("model.cache_prune", e);
  }
}

JsonRpcResult SimulationPlug::handle_model_verify(const nlohmann::json& data) {
  try {
    return JsonRpcResult::ok("model verified", model_registry_.verify(data));
  } catch (const std::exception& e) {
    return route_error("model.verify", e);
  }
}

JsonRpcResult SimulationPlug::handle_model_validate(const nlohmann::json& data) {
  try {
    return JsonRpcResult::ok("model validation", model_validator_.validate(data));
  } catch (const std::exception& e) {
    return route_error("model.validate", e);
  }
}
JsonRpcResult SimulationPlug::handle_model_inspect(const nlohmann::json& data) {
  try {
    return JsonRpcResult::ok("model inspection", instances_.inspect_model(data));
  } catch (const std::exception& e) {
    return route_error("model.inspect", e);
  }
}

JsonRpcResult SimulationPlug::handle_control_joint_state(const nlohmann::json& data) {
  try {
    return JsonRpcResult::ok("joint state", instances_.joint_state(data));
  } catch (const std::exception& e) {
    return route_error("control.joint_state", e);
  }
}

JsonRpcResult SimulationPlug::handle_control_sensor_state(const nlohmann::json& data) {
  try {
    return JsonRpcResult::ok("sensor state", instances_.sensor_state(data));
  } catch (const std::exception& e) {
    return route_error("control.sensor_state", e);
  }
}

JsonRpcResult SimulationPlug::handle_control_write_ctrl(const nlohmann::json& data) {
  try {
    auto state = instances_.write_ctrl(data);
    publish_state(state);
    return JsonRpcResult::ok("control written", std::move(state));
  } catch (const std::exception& e) {
    return route_error("control.write_ctrl", e);
  }
}

JsonRpcResult SimulationPlug::handle_control_write_qpos(const nlohmann::json& data) {
  try {
    auto state = instances_.write_qpos(data);
    publish_state(state);
    return JsonRpcResult::ok("qpos written", std::move(state));
  } catch (const std::exception& e) {
    return route_error("control.write_qpos", e);
  }
}
JsonRpcResult SimulationPlug::handle_task_create(const nlohmann::json& data) {
  try {
    return JsonRpcResult::ok("task created", tasks_.create(data));
  } catch (const std::exception& e) {
    return route_error("task.create", e);
  }
}

JsonRpcResult SimulationPlug::handle_task_remove(const nlohmann::json& data) {
  try {
    return JsonRpcResult::ok("task removed", tasks_.remove(data));
  } catch (const std::exception& e) {
    return route_error("task.remove", e);
  }
}

JsonRpcResult SimulationPlug::handle_task_list(const nlohmann::json& data) {
  try {
    (void)data;
    return JsonRpcResult::ok("task list", tasks_.list());
  } catch (const std::exception& e) {
    return route_error("task.list", e);
  }
}

JsonRpcResult SimulationPlug::handle_task_info(const nlohmann::json& data) {
  try {
    return JsonRpcResult::ok("task info", tasks_.info(data));
  } catch (const std::exception& e) {
    return route_error("task.info", e);
  }
}

JsonRpcResult SimulationPlug::handle_task_run(const nlohmann::json& data) {
  try {
    return JsonRpcResult::ok("task done", tasks_.run(data, instances_));
  } catch (const std::exception& e) {
    return route_error("task.run", e);
  }
}
JsonRpcResult SimulationPlug::handle_record_start(const nlohmann::json& data) {
  try {
    return JsonRpcResult::ok("record started", records_.start(data, instances_));
  } catch (const std::exception& e) {
    return route_error("record.start", e);
  }
}

JsonRpcResult SimulationPlug::handle_record_stop(const nlohmann::json& data) {
  try {
    return JsonRpcResult::ok("record stopped", records_.stop(data));
  } catch (const std::exception& e) {
    return route_error("record.stop", e);
  }
}

JsonRpcResult SimulationPlug::handle_record_list(const nlohmann::json& data) {
  try {
    (void)data;
    return JsonRpcResult::ok("record list", records_.list());
  } catch (const std::exception& e) {
    return route_error("record.list", e);
  }
}

JsonRpcResult SimulationPlug::handle_record_info(const nlohmann::json& data) {
  try {
    return JsonRpcResult::ok("record info", records_.info(data));
  } catch (const std::exception& e) {
    return route_error("record.info", e);
  }
}

JsonRpcResult SimulationPlug::handle_record_remove(const nlohmann::json& data) {
  try {
    return JsonRpcResult::ok("record removed", records_.remove(data));
  } catch (const std::exception& e) {
    return route_error("record.remove", e);
  }
}
JsonRpcResult SimulationPlug::route_error(const char* action, const std::exception& e) const {
  LOG_ERROR("[{}] {} failed: {}", TAG, action, e.what());
  return JsonRpcResult::error(e.what(), -1, 400);
}

namespace {
  PluginHttpResponse to_http_response(const JsonRpcResult& result) {
    PluginHttpResponse res;
    res.status_code = result.http_status;
    res.content_type = "application/json; charset=utf-8";
    nlohmann::json out{{"code", result.code}, {"message", result.message}};
    if (!result.data.is_null()) out["data"] = result.data;
    const auto text = out.dump();
    res.body.assign(text.begin(), text.end());
    return res;
  }
}  // namespace

PluginHttpResponse SimulationPlug::handle_http_rpc(const PluginHttpRequest& req) {
  try {
    // Host 已超时(504)或客户端已断开时不再执行重活（scene.compile 等）
    if (req.stop_token.stop_requested()) {
      return to_http_response(JsonRpcResult::error("request canceled", -1, 503));
    }

    // Host 已按声明的 routes 匹配：route_pattern 形如 "/scene.load"，未命中根本到不了这里
    std::string module = req.route_pattern;
    if (!module.empty() && module.front() == '/') module.erase(0, 1);

    // multipart 上传单独处理（大文件走 parts，不受 JSON body 限制）
    if (module == "model.upload") return handle_model_upload(req);

    if (req.body_spooled_to_file()) {
      return to_http_response(JsonRpcResult::error("request body too large", -1, 413));
    }

    const auto route = routes_->find(module);
    if (route == routes_->end()) {
      return to_http_response(JsonRpcResult::error("Handler not found: " + module, -1, 404));
    }

    nlohmann::json data = nlohmann::json::object();
    if (!req.text().empty()) {
      data = nlohmann::json::parse(req.text());
      if (!data.is_object()) {
        return to_http_response(
            JsonRpcResult::error("Request body must be a json object", -1, 400));
      }
    }

    return to_http_response(route->second(data));
  } catch (const nlohmann::json::parse_error& e) {
    return to_http_response(
        JsonRpcResult::error(std::string("Invalid json: ") + e.what(), -1, 400));
  } catch (const std::exception& e) {
    return to_http_response(JsonRpcResult::error(std::string("Http RPC error: ") + e.what()));
  } catch (...) {
    return to_http_response(JsonRpcResult::error("Http RPC unknown error"));
  }
}

namespace {
  // 上传文件名允许携带相对子目录（mesh 引用），但绝不允许逃出上传目录
  std::filesystem::path sanitize_upload_rel_path(std::string name) {
    std::replace(name.begin(), name.end(), '\\', '/');
    std::filesystem::path out;
    for (const auto& part : std::filesystem::path(name)) {
      const auto text = part.string();
      if (text.empty() || text == "." || text == ".." || text == "/") continue;
      if (text.find(':') != std::string::npos) continue;  // 丢弃 Windows 盘符段
      out /= text;
    }
    if (out.empty() || out.is_absolute())
      throw std::invalid_argument("invalid upload filename: " + name);
    return out;
  }

  std::string safe_dir_token(const std::string& text) {
    std::string out;
    for (const char ch : text) {
      const bool ok = (ch >= 'a' && ch <= 'z') || (ch >= 'A' && ch <= 'Z')
                      || (ch >= '0' && ch <= '9') || ch == '.' || ch == '_' || ch == '-';
      out.push_back(ok ? ch : '_');
    }
    return out.empty() ? std::string("x") : out;
  }

  bool looks_like_model_file(const std::filesystem::path& rel) {
    const auto ext = rel.extension().string();
    return ext == ".xml" || ext == ".urdf" || ext == ".mjcf" || ext == ".XML" || ext == ".URDF";
  }
}  // namespace

PluginHttpResponse SimulationPlug::handle_model_upload(const PluginHttpRequest& req) {
  namespace fs = std::filesystem;
  fs::path upload_dir;
  try {
    if (!req.multipart()) {
      return to_http_response(
          JsonRpcResult::error("model.upload requires multipart/form-data", -1, 400));
    }

    const auto field_text = [&](const char* name) -> std::string {
      const auto* part = req.part(name);
      if (!part || part->body_spooled_to_file()) return {};
      return std::string(part->text());
    };

    const std::string id = field_text("id");
    std::string version = field_text("version");
    if (version.empty()) version = "1";
    if (id.empty()) {
      return to_http_response(JsonRpcResult::error("missing 'id' form field", -1, 400));
    }

    const auto now_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                            std::chrono::system_clock::now().time_since_epoch())
                            .count();
    upload_dir
        = simulation_data_dir("tmp/uploads", "model_uploads")
          / (safe_dir_token(id) + "_" + safe_dir_token(version) + "_" + std::to_string(now_ms));
    fs::create_directories(upload_dir);

    // 落盘所有文件 part，保留相对子目录结构（meshdir/include 引用可继续解析）
    fs::path model_file;
    fs::path first_file;
    size_t file_count = 0;
    for (const auto& part : req.parts) {
      if (part.filename.empty()) continue;
      const auto rel = sanitize_upload_rel_path(part.filename);
      const auto dest = upload_dir / rel;
      fs::create_directories(dest.parent_path());

      if (part.body_spooled_to_file()) {
        fs::copy_file(part.body_file_path, dest, fs::copy_options::overwrite_existing);
      } else {
        std::ofstream out(dest, std::ios::binary | std::ios::trunc);
        if (!out) throw std::runtime_error("failed to write upload file: " + dest.string());
        out.write(reinterpret_cast<const char*>(part.body.data()),
                  static_cast<std::streamsize>(part.body.size()));
        if (!out.good()) throw std::runtime_error("failed to write upload file: " + dest.string());
      }
      ++file_count;
      if (first_file.empty()) first_file = dest;
      if (model_file.empty() && (part.name == "model" || looks_like_model_file(rel))) {
        model_file = dest;
      }
    }
    if (file_count == 0) {
      return to_http_response(JsonRpcResult::error("no file parts in upload", -1, 400));
    }
    if (model_file.empty()) model_file = first_file;

    nlohmann::json payload{
        {"id", id},
        {"version", version},
        {"path", model_file.string()},
        {"asset_root", upload_dir.string()},
        {"copy_assets", true},
        {"replace", field_text("replace") == "true"},
    };
    if (!field_text("require_actuators").empty()) {
      payload["require_actuators"] = field_text("require_actuators") == "true";
    }

    // 注册即快照进资产库，随后清理临时上传目录
    auto result = handle_model_register(payload);
    std::error_code ec;
    fs::remove_all(upload_dir, ec);
    return to_http_response(result);
  } catch (const std::exception& e) {
    if (!upload_dir.empty()) {
      std::error_code ec;
      fs::remove_all(upload_dir, ec);
    }
    LOG_ERROR("[{}] model.upload failed: {}", TAG, e.what());
    return to_http_response(JsonRpcResult::error(e.what(), -1, 400));
  } catch (...) {
    if (!upload_dir.empty()) {
      std::error_code ec;
      fs::remove_all(upload_dir, ec);
    }
    return to_http_response(JsonRpcResult::error("model.upload unknown error"));
  }
}

void SimulationPlug::handle_ws_open(const char* session_id) {
  if (!session_id) return;
  const nlohmann::json hello{
      {"type", "hello"},
      {"endpoint", "simulation"},
      {"usage",
       R"(send {"action":"subscribe","instances":["<id>"],"visual":true}; ["*"] subscribes all; visual attaches geom poses)"}};
  send_ws_text_checked(session_id, hello.dump());
}

void SimulationPlug::handle_ws_close(const char* session_id) {
  if (!session_id) return;
  std::lock_guard<std::mutex> lock(ws_mutex_);
  ws_subscriptions_.erase(session_id);
}

void SimulationPlug::handle_ws_message(const char* session_id, const void* data, size_t size,
                                       PluginWsMessageType type) {
  if (!session_id || type != PluginWsMessageType::Text) return;

  const auto reply = [&](const nlohmann::json& j) { send_ws_text_checked(session_id, j.dump()); };

  nlohmann::json msg;
  try {
    msg = nlohmann::json::parse(std::string_view(static_cast<const char*>(data), size));
  } catch (const std::exception& e) {
    reply({{"type", "error"}, {"message", std::string("Invalid json: ") + e.what()}});
    return;
  }

  try {
    const std::string action = msg.value("action", "");
    constexpr size_t kMaxSubscriptions = 64;
    constexpr size_t kMaxInstanceIdSize = 128;

    std::unordered_set<std::string> instances;
    if (msg.contains("instances")) {
      if (!msg["instances"].is_array() || msg["instances"].size() > kMaxSubscriptions) {
        reply({{"type", "error"}, {"message", "instances must be an array of at most 64 IDs"}});
        return;
      }
      for (const auto& item : msg["instances"]) {
        if (!item.is_string()) {
          reply({{"type", "error"}, {"message", "instance IDs must be strings"}});
          return;
        }
        const auto id = item.get<std::string>();
        if (id.empty() || id.size() > kMaxInstanceIdSize) {
          reply({{"type", "error"}, {"message", "instance ID length must be 1..128"}});
          return;
        }
        instances.insert(id);
      }
    } else if (msg.contains("instance")) {
      if (!msg["instance"].is_string()) {
        reply({{"type", "error"}, {"message", "instance must be a string"}});
        return;
      }
      const auto id = msg["instance"].get<std::string>();
      if (id.empty() || id.size() > kMaxInstanceIdSize) {
        reply({{"type", "error"}, {"message", "instance ID length must be 1..128"}});
        return;
      }
      instances.insert(id);
    }

    if (action == "subscribe") {
      if (instances.empty()) {
        reply({{"type", "error"}, {"message", "subscribe requires instances"}});
        return;
      }
      if (msg.contains("visual") && !msg["visual"].is_boolean()) {
        reply({{"type", "error"}, {"message", "visual must be a boolean"}});
        return;
      }
      bool visual = false;
      bool limit_exceeded = false;
      {
        std::lock_guard<std::mutex> lock(ws_mutex_);
        auto next = ws_subscriptions_[session_id];
        if (instances.count("*") != 0) {
          next.instances = {"*"};
        } else if (next.instances.count("*") == 0) {
          next.instances.insert(instances.begin(), instances.end());
        }
        if (next.instances.size() > kMaxSubscriptions) {
          limit_exceeded = true;
        } else {
          if (msg.contains("visual")) next.visual = msg.value("visual", false);
          visual = next.visual;
          ws_subscriptions_[session_id] = std::move(next);
        }
      }
      if (limit_exceeded) {
        reply({{"type", "error"}, {"message", "subscription limit exceeded"}});
        return;
      }
      reply(
          {{"type", "ack"}, {"action", "subscribe"}, {"instances", instances}, {"visual", visual}});
      return;
    }

    if (action == "unsubscribe") {
      if (instances.empty()) {
        reply({{"type", "error"}, {"message", "unsubscribe requires instances"}});
        return;
      }
      {
        std::lock_guard<std::mutex> lock(ws_mutex_);
        const auto it = ws_subscriptions_.find(session_id);
        if (it != ws_subscriptions_.end()) {
          if (instances.count("*") != 0) {
            ws_subscriptions_.erase(it);
          } else {
            for (const auto& id : instances) it->second.instances.erase(id);
            if (it->second.instances.empty()) ws_subscriptions_.erase(it);
          }
        }
      }
      reply({{"type", "ack"}, {"action", "unsubscribe"}, {"instances", instances}});
      return;
    }

    if (action == "ping") {
      reply({{"type", "pong"}});
      return;
    }

    reply({{"type", "error"}, {"message", "Unknown action: " + action}});
  } catch (const std::exception& e) {
    reply({{"type", "error"}, {"message", e.what()}});
  }
}

nlohmann::json SimulationPlug::resolve_scene_models(const nlohmann::json& scene) const {
  return model_registry_.resolve_scene(scene);
}

void SimulationPlug::publish_state(const nlohmann::json& state) noexcept {
  try {
    const std::string id = state.value("id", "");
    if (id.empty()) return;

    const std::string topic = "simulation.instance." + id + ".state";
    const std::string payload = state.dump();
    emit_text(topic.c_str(), payload);

    std::vector<std::string> plain_targets;
    std::vector<std::string> visual_targets;
    {
      std::lock_guard<std::mutex> lock(ws_mutex_);
      for (const auto& [session_id, sub] : ws_subscriptions_) {
        if (sub.instances.count("*") == 0 && sub.instances.count(id) == 0) continue;
        (sub.visual ? visual_targets : plain_targets).push_back(session_id);
      }
    }
    if (plain_targets.empty() && visual_targets.empty()) return;

    nlohmann::json event{{"type", "state"}, {"topic", topic}, {"data", state}};
    if (!plain_targets.empty()) {
      const std::string text = event.dump();
      for (const auto& session_id : plain_targets)
        send_ws_latest_text_checked(session_id.c_str(), topic.c_str(), text);
    }
    if (!visual_targets.empty()) {
      // 附带 geom 世界位姿（transforms-only），浏览器端按 geom_id 增量更新 3D 预览。
      // 实例刚销毁等竞态下取不到位姿时退化为纯状态推送。
      try {
        if (!state.value("destroyed", false)) {
          event["visual"] = instances_.visual_model({{"id", id}, {"include_geometry", false}});
        }
      } catch (...) {
      }
      const std::string text = event.dump();
      for (const auto& session_id : visual_targets)
        send_ws_latest_text_checked(session_id.c_str(), topic.c_str(), text);
    }
  } catch (const std::exception& e) {
    LOG_ERROR("[{}] publish_state failed: {}", TAG, e.what());
  } catch (...) {
    LOG_ERROR("[{}] publish_state unknown failure", TAG);
  }
}

bool SimulationPlug::send_ws_text_checked(const char* session_id, std::string_view text) noexcept {
  const auto result = ws_send_text_result(session_id, text);
  if (result == PluginWsSendResult::Queued) return true;
  note_ws_send_failure("send_text", session_id, result, text.size());
  return false;
}

bool SimulationPlug::send_ws_latest_text_checked(const char* session_id, const char* topic,
                                                 std::string_view text) noexcept {
  const auto result = ws_send_latest_text_result(session_id, topic, text);
  if (result == PluginWsSendResult::Queued) return true;
  note_ws_send_failure("send_latest_text", session_id, result, text.size(), topic);
  return false;
}

void SimulationPlug::note_ws_send_failure(const char* operation, const char* session_id,
                                          PluginWsSendResult result, size_t bytes,
                                          const char* topic) noexcept {
  if (session_id
      && (result == PluginWsSendResult::SessionClosed
          || result == PluginWsSendResult::SessionNotFound)) {
    std::lock_guard<std::mutex> lock(ws_mutex_);
    ws_subscriptions_.erase(session_id);
  }

  const uint64_t failures = ws_send_failures_.fetch_add(1, std::memory_order_relaxed) + 1;
  // 1,2,4,8... 次时记录，既能看到持续失败，又避免高频状态发送刷日志。
  if ((failures & (failures - 1)) == 0) {
    if (topic && topic[0] != '\0') {
      LOG_WARN("[{}] WS {} failed: result={}, session={}, topic={}, bytes={}, total_failures={}",
               TAG, operation ? operation : "send", plugin_ws_send_result_name(result),
               session_id ? session_id : "", topic, bytes, failures);
    } else {
      LOG_WARN("[{}] WS {} failed: result={}, session={}, bytes={}, total_failures={}", TAG,
               operation ? operation : "send", plugin_ws_send_result_name(result),
               session_id ? session_id : "", bytes, failures);
    }
  }
}

PLUGIN_DECLARE(SimulationPlug, "SimulationPlugin", "1.0", "liuyan", "Simulation Plugin")
