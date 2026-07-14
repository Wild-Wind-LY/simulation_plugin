#include "simulation_plug.hpp"

#include <exception>
#include <utility>

/*
- Scene Layer：只描述“有什么”，例如 obstacle、sensor�?- Compile Layer：负责把 obstacle/senso
翻译�?MJCF�?- Simulation Layer：完全不用知�?obstacle/sensor 是怎么生成的，只加�?compiled XML�?- Task
Layer：还没做，后面负责程序、轨迹、机器人动作�?*/
bool SimulationPlug::on_init(IPluginContext& ctx) noexcept {
  // 设置插件全局日志
  LOGX_GLOBAL_CLASS(PLUGIN_LOG_NAME)::Construct();
  G_LOG()->set(logger());

  LOG_INFO("[{}] Initialized!", TAG);
  return true;
}

bool SimulationPlug::on_start() noexcept {
  const bool registered = register_json_service(
      "simulation.http",
      {
          {"scene.load", [this](const nlohmann::json& data) { return handle_scene_load(data); }},
          {"scene.unload",
           [this](const nlohmann::json& data) { return handle_scene_unload(data); }},
          {"scene.update",
           [this](const nlohmann::json& data) { return handle_scene_update(data); }},
          {"scene.apply", [this](const nlohmann::json& data) { return handle_scene_apply(data); }},
          {"scene.list", [this](const nlohmann::json& data) { return handle_scene_list(data); }},
          {"scene.info", [this](const nlohmann::json& data) { return handle_scene_info(data); }},
          {"scene.validate",
           [this](const nlohmann::json& data) { return handle_scene_validate(data); }},
          {"scene.diff", [this](const nlohmann::json& data) { return handle_scene_diff(data); }},
          {"scene.compile",
           [this](const nlohmann::json& data) { return handle_scene_compile(data); }},
          {"scene.export",
           [this](const nlohmann::json& data) { return handle_scene_export(data); }},
          {"visual.scene",
           [this](const nlohmann::json& data) { return handle_visual_scene(data); }},
          {"visual.model",
           [this](const nlohmann::json& data) { return handle_visual_model(data); }},
          {"instance.create",
           [this](const nlohmann::json& data) { return handle_instance_create(data); }},
          {"instance.recreate_from_scene",
           [this](const nlohmann::json& data) {
             return handle_instance_recreate_from_scene(data);
           }},
          {"instance.start",
           [this](const nlohmann::json& data) { return handle_instance_start(data); }},
          {"instance.pause",
           [this](const nlohmann::json& data) { return handle_instance_pause(data); }},
          {"instance.stop",
           [this](const nlohmann::json& data) { return handle_instance_stop(data); }},
          {"instance.step",
           [this](const nlohmann::json& data) { return handle_instance_step(data); }},
          {"instance.reset",
           [this](const nlohmann::json& data) { return handle_instance_reset(data); }},
          {"instance.apply_runtime",
           [this](const nlohmann::json& data) { return handle_instance_apply_runtime(data); }},
          {"instance.state",
           [this](const nlohmann::json& data) { return handle_instance_state(data); }},
          {"instance.metadata",
           [this](const nlohmann::json& data) { return handle_instance_metadata(data); }},
          {"instance.list",
           [this](const nlohmann::json& data) { return handle_instance_list(data); }},
          {"task.create", [this](const nlohmann::json& data) { return handle_task_create(data); }},
          {"task.remove", [this](const nlohmann::json& data) { return handle_task_remove(data); }},
          {"task.list", [this](const nlohmann::json& data) { return handle_task_list(data); }},
          {"task.info", [this](const nlohmann::json& data) { return handle_task_info(data); }},
          {"task.run", [this](const nlohmann::json& data) { return handle_task_run(data); }},
          {"record.start",
           [this](const nlohmann::json& data) { return handle_record_start(data); }},
          {"record.stop", [this](const nlohmann::json& data) { return handle_record_stop(data); }},
          {"record.list", [this](const nlohmann::json& data) { return handle_record_list(data); }},
          {"record.info", [this](const nlohmann::json& data) { return handle_record_info(data); }},
          {"record.remove",
           [this](const nlohmann::json& data) { return handle_record_remove(data); }},
          {"model.register",
           [this](const nlohmann::json& data) { return handle_model_register(data); }},
          {"model.list", [this](const nlohmann::json& data) { return handle_model_list(data); }},
          {"model.info", [this](const nlohmann::json& data) { return handle_model_info(data); }},
          {"model.remove",
           [this](const nlohmann::json& data) { return handle_model_remove(data); }},
          {"model.cache_prune",
           [this](const nlohmann::json& data) { return handle_model_cache_prune(data); }},
          {"model.verify",
           [this](const nlohmann::json& data) { return handle_model_verify(data); }},
          {"model.validate",
           [this](const nlohmann::json& data) { return handle_model_validate(data); }},
          {"model.inspect",
           [this](const nlohmann::json& data) { return handle_model_inspect(data); }},
          {"control.joint_state",
           [this](const nlohmann::json& data) { return handle_control_joint_state(data); }},
          {"control.sensor_state",
           [this](const nlohmann::json& data) { return handle_control_sensor_state(data); }},
          {"control.write_ctrl",
           [this](const nlohmann::json& data) { return handle_control_write_ctrl(data); }},
          {"instance.destroy",
           [this](const nlohmann::json& data) { return handle_instance_destroy(data); }},
      });

  if (!registered) {
    LOG_ERROR("[{}] Failed to register simulation.http", TAG);
    return false;
  }

  LOG_INFO("[{}] simulation.http is ready", TAG);
  LOG_INFO("[{}] Started!", TAG);
  return true;
}

void SimulationPlug::on_stop() noexcept {
  records_.stop_all();
  instances_.stop_all();
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
    return JsonRpcResult::ok("scene validation", compiler_.validate_scene(resolve_scene_models(scene)));
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
    return JsonRpcResult::ok("scene compiled", compiler_.compile_scene(resolve_scene_models(scene)));
  } catch (const std::exception& e) {
    return route_error("scene.compile", e);
  }
}

JsonRpcResult SimulationPlug::handle_scene_export(const nlohmann::json& data) {
  try {
    const auto scene = scenes_.info(data);
    return JsonRpcResult::ok("scene exported",
                             compiler_.export_scene(resolve_scene_models(scene),
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
    return JsonRpcResult::ok("visual scene", compiler_.build_visual_scene(resolve_scene_models(scene)));
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
    return JsonRpcResult::ok("model cache pruned",
                             {{"removed", compiler_.prune_unused_models()}});
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
  } catch (const std::exception& e) {
    LOG_ERROR("[{}] publish_state failed: {}", TAG, e.what());
  } catch (...) {
    LOG_ERROR("[{}] publish_state unknown failure", TAG);
  }
}

PLUGIN_DECLARE(SimulationPlug, "SimulationPlugin", "1.0", "liuyan", "Simulation Plugin")
