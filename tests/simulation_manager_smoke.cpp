#include <chrono>
#include <filesystem>
#include <iostream>
#include <nlohmann/json.hpp>
#include <system_error>
#include <thread>

#include "simulation_compiler.hpp"
#include "simulation_instance_manager.hpp"
#include "simulation_model_registry.hpp"
#include "simulation_model_validator.hpp"
#include "simulation_paths.hpp"
#include "simulation_record_manager.hpp"
#include "simulation_scene_manager.hpp"
#include "simulation_task_manager.hpp"
#include "simulation_visual.hpp"

int main() {
  try {
    SimulationSceneManager scenes;
    auto scene = scenes.load(
        {{"path", "/home/liuyan/rpc_gateway_plugins/simulation_plugin/tests/simple_scene.json"}});
    if (scene.value("id", "") != "falling_box_scene") {
      std::cerr << "scene load failed\n";
      return 1;
    }
    if (scenes.list().empty()
        || scenes.info({{"id", "falling_box_scene"}}).value("name", "") != "Falling Box Demo") {
      std::cerr << "scene list/info failed\n";
      return 1;
    }
    // scene.list 磁盘发现：保存到默认目录 -> 卸载 -> list 以 loaded:false 列出 -> 按 path 重载
    {
      const auto saved = scenes.save(
          {{"id", "falling_box_scene"},
           {"path", (simulation_data_dir("scenes", "scenes") / "__smoke_list__.json").string()}});
      const std::string saved_path = saved.value("path", "");
      scenes.unload({{"id", "falling_box_scene"}});
      bool found_on_disk = false;
      for (const auto& entry : scenes.list()) {
        if (entry.value("id", "") == "falling_box_scene" && !entry.value("loaded", true)
            && entry.value("path", "") == saved_path) {
          found_on_disk = true;
          break;
        }
      }
      if (!found_on_disk) {
        std::cerr << "scene list disk discovery failed\n";
        return 1;
      }
      auto reloaded = scenes.load({{"path", saved_path}});
      bool loaded_in_list = false;
      for (const auto& entry : scenes.list()) {
        if (entry.value("id", "") == "falling_box_scene" && entry.value("loaded", false)) {
          loaded_in_list = true;
          break;
        }
      }
      if (reloaded.value("id", "") != "falling_box_scene" || !loaded_in_list) {
        std::cerr << "scene list reload failed\n";
        return 1;
      }
      std::error_code cleanup_ec;
      std::filesystem::remove(saved_path, cleanup_ec);
    }
    scene = scenes.update(scene);
    if (!scenes.references_model("workpiece", "1") || scenes.references_model("workpiece", "2")) {
      std::cerr << "scene model reference tracking failed\n";
      return 1;
    }

    SimulationInstanceManager manager;
    int published = 0;
    auto publisher = [&published](const nlohmann::json&) { ++published; };

    const auto registry_dir
        = std::filesystem::temp_directory_path() / "simulation_plugin_registry_smoke";
    std::filesystem::remove_all(registry_dir);
    SimulationModelRegistry registry(registry_dir);
    auto registered_mjcf = registry.register_model({
        {"id", "workpiece"},
        {"version", "1"},
        {"path", "/home/liuyan/rpc_gateway_plugins/simulation_plugin/tests/attached_workpiece.xml"},
    });
    auto registered_urdf = registry.register_model({
        {"id", "simple_arm"},
        {"version", "1"},
        {"path", "/home/liuyan/rpc_gateway_plugins/simulation_plugin/tests/simple_arm.urdf"},
    });
    registry.register_model({
        {"id", "falling_box"},
        {"version", "1"},
        {"path", "/home/liuyan/rpc_gateway_plugins/simulation_plugin/tests/falling_box.xml"},
    });
    if (registered_mjcf.value("format", "") != "mjcf"
        || registered_urdf.value("format", "") != "urdf"
        || !std::filesystem::exists(registered_urdf.value("effective_path", ""))) {
      std::cerr << "model registry registration failed\n";
      return 1;
    }
    registry.register_model({
        {"id", "workpiece"},
        {"version", "2"},
        {"path", "/home/liuyan/rpc_gateway_plugins/simulation_plugin/tests/attached_workpiece.xml"},
    });
    if (registry.info({{"id", "workpiece"}}).value("version", "") != "2"
        || !registry.remove({{"id", "workpiece"}, {"version", "2"}}).value("removed", false)) {
      std::cerr << "model registry version lifecycle failed\n";
      return 1;
    }
    SimulationModelRegistry reloaded_registry(registry_dir);
    auto registry_scene = nlohmann::json{
        {"id", "registry_scene"},
        {"models", nlohmann::json::array({
                       {{"id", "arm_1"}, {"model_id", "simple_arm"}, {"version", "1"}},
                   })},
        {"defaults", nlohmann::json::object()},
    };
    auto resolved_registry_scene = reloaded_registry.resolve_scene(registry_scene);
    if (resolved_registry_scene["models"][0].value("source", "").empty()
        || reloaded_registry.list().size() != 3) {
      std::cerr << "model registry persistence/resolve failed\n";
      return 1;
    }

    const std::string debug_model_path
        = "/home/liuyan/rpc_gateway_plugins/simulation_plugin/tests/simple_model.xml";
    SimulationModelValidator model_validator;
    auto model_validation = model_validator.validate({{"model_path", debug_model_path}});
    if (!model_validation.value("valid", false) || model_validation.value("controllable", true)) {
      std::cerr << "model validation failed\n";
      return 1;
    }
    auto missing_model_validation
        = model_validator.validate({{"model_path", "/tmp/simulation_plugin_missing_model.xml"}});
    if (missing_model_validation.value("valid", true)
        || missing_model_validation["errors"].empty()) {
      std::cerr << "missing model validation failed\n";
      return 1;
    }

    SimulationCompiler compiler;
    auto [direct_id, direct_model] = compiler.get_or_load_model(debug_model_path);
    auto [direct_id_again, direct_model_again] = compiler.get_or_load_model(debug_model_path);
    if (direct_id != direct_id_again || direct_model != direct_model_again) {
      std::cerr << "direct model cache failed\n";
      return 1;
    }
    direct_model.reset();
    direct_model_again.reset();
    if (compiler.prune_unused_models() != 1) {
      std::cerr << "model cache prune failed\n";
      return 1;
    }
    auto registry_compiled = compiler.compile_scene(resolved_registry_scene);
    auto registry_model
        = compiler.get_compiled_model(registry_compiled.value("compiled_model_id", ""));
    if (!registry_model || registry_model->nbody < 2) {
      std::cerr << "registered URDF scene compile failed\n";
      return 1;
    }
    // model.visual 路径：注册资产独立编译 -> 默认位形几何（资产库缩略图/首次拖拽预览）
    {
      const auto asset_info = reloaded_registry.info({{"id", "workpiece"}});
      const std::string effective = asset_info.value("effective_path", "");
      auto [asset_cache_id, asset_model] = compiler.get_or_load_model(effective);
      if (effective.empty() || !asset_model) {
        std::cerr << "asset visual: standalone load failed\n";
        return 1;
      }
      mjData* asset_data = mj_makeData(asset_model.get());
      if (!asset_data) {
        std::cerr << "asset visual: mjData allocation failed\n";
        return 1;
      }
      mj_forward(asset_model.get(), asset_data);
      auto asset_visual = simulation_visual_json(asset_model.get(), asset_data, true);
      mj_deleteData(asset_data);
      if (asset_visual["items"].empty()
          || asset_visual["model_counts"].value("geom", 0) != asset_model->ngeom) {
        std::cerr << "asset visual: extraction failed\n";
        return 1;
      }
    }
    // 场景模型只允许注册资产引用：编译/校验前都必须经过注册表解析
    auto resolved_scene = reloaded_registry.resolve_scene(scene);
    auto validation = compiler.validate_scene(resolved_scene);
    if (!validation.value("valid", false)) {
      std::cerr << "scene validation failed\n";
      return 1;
    }
    auto unresolved_validation = compiler.validate_scene(scene);
    if (unresolved_validation.value("valid", true)) {
      std::cerr << "unresolved scene should fail validation\n";
      return 1;
    }
    auto runtime_scene = scene;
    runtime_scene["defaults"]["step_hz"] = 240;
    auto runtime_diff = compiler.diff_scenes(scene, runtime_scene);
    if (!runtime_diff.value("runtime_changed", false)
        || runtime_diff.value("requires_recreate", true)) {
      std::cerr << "runtime diff failed\n";
      return 1;
    }
    auto structural_scene = scene;
    structural_scene["objects"].push_back({{"id", "extra_box"}, {"type", "obstacle.box"}});
    auto structural_diff = compiler.diff_scenes(scene, structural_scene);
    if (!structural_diff.value("requires_recreate", false)) {
      std::cerr << "structural diff failed\n";
      return 1;
    }
    auto compiled = compiler.compile_scene(resolved_scene);
    if (!compiled.value("generated", false)
        || !compiled["mapping"]["bodies"].contains("obstacle_box_1")) {
      std::cerr << "compiled obstacle missing\n";
      return 1;
    }
    if (!compiled["mapping"]["models"].contains("workpiece_asset")) {
      std::cerr << "compiled model mapping missing\n";
      return 1;
    }
    if (!compiled["mapping"]["sensors"].contains("box_force_sensor")
        || !compiled["mapping"]["sensors"].contains("box_torque_sensor")) {
      std::cerr << "compiled sensor missing\n";
      return 1;
    }
    if (compiled.value("backend", "") != "mujoco"
        || compiled.value("compiled_model_id", "").empty()) {
      std::cerr << "compile failed\n";
      return 1;
    }
    auto shared_model = compiler.get_compiled_model(compiled.value("compiled_model_id", ""));
    auto compiled_again = compiler.compile_scene(resolved_scene);
    auto shared_model_again
        = compiler.get_compiled_model(compiled_again.value("compiled_model_id", ""));
    if (shared_model != shared_model_again || shared_model->nq < 14 || shared_model->nexclude < 1
        || mj_name2id(shared_model.get(), mjOBJ_BODY, "workpiece_asset_workpiece") < 0) {
      std::cerr << "compiled model cache/attachment failed\n";
      return 1;
    }
    // scene.compile include_visual 路径：无实例时用临时 mjData 提取编译几何
    {
      mjData* preview_data = mj_makeData(shared_model.get());
      if (!preview_data) {
        std::cerr << "preview mjData allocation failed\n";
        return 1;
      }
      mj_forward(shared_model.get(), preview_data);
      auto preview = simulation_visual_json(shared_model.get(), preview_data, true);
      mj_deleteData(preview_data);
      if (!preview.value("geometry_included", false)
          || preview["items"].size() != static_cast<size_t>(shared_model->ngeom)
          || preview["model_counts"].value("geom", 0) != shared_model->ngeom) {
        std::cerr << "scene visual preview failed\n";
        return 1;
      }
      bool found_attached_geom = false;
      for (const auto& item : preview["items"]) {
        if (item.value("body", "").rfind("workpiece_asset_", 0) == 0) {
          found_attached_geom = true;
          break;
        }
      }
      if (!found_attached_geom) {
        std::cerr << "scene visual preview missing attached model geometry\n";
        return 1;
      }
    }
    auto composed_scene = resolved_scene;
    composed_scene["id"] = "asset_only_scene";
    composed_scene["objects"] = nlohmann::json::array();
    composed_scene["sensors"] = nlohmann::json::array();
    composed_scene["contacts"] = nlohmann::json::object();
    auto composed_validation = compiler.validate_scene(composed_scene);
    auto composed_compiled = compiler.compile_scene(composed_scene);
    auto composed_model
        = compiler.get_compiled_model(composed_compiled.value("compiled_model_id", ""));
    if (!composed_validation.value("valid", false) || composed_model->nq != 14) {
      std::cerr << "asset-only scene compile failed\n";
      return 1;
    }

    auto visual = compiler.build_visual_scene(resolved_scene);
    if (visual.value("coordinate_system", "") != "mujoco_xyz"
        || visual.value("items", nlohmann::json::array()).size() < 4) {
      std::cerr << "visual scene builder failed\n";
      return 1;
    }
    auto config = compiler.build_instance_config(
        {{"id", "smoke"}, {"scene_id", "falling_box_scene"}}, compiled);
    auto created = manager.create(config, shared_model);
    if (created.value("id", "") != "smoke"
        || created.value("scene_id", "") != "falling_box_scene") {
      std::cerr << "unexpected scene instance\n";
      return 1;
    }
    auto second_config = config;
    second_config["id"] = "smoke-2";
    auto second_created = manager.create(second_config, shared_model);
    if (second_created.value("compiled_model_id", "") != created.value("compiled_model_id", "")
        || second_created.value("id", "") != "smoke-2") {
      std::cerr << "shared model instance failed\n";
      return 1;
    }
    auto metadata = manager.metadata({{"id", "smoke"}});
    if (!metadata.contains("sensors") || metadata["sensors"].size() < 2) {
      std::cerr << "metadata sensors missing\n";
      return 1;
    }
    auto visual_model = manager.visual_model({{"id", "smoke"}});
    if (visual_model.value("backend", "") != "mujoco"
        || !visual_model.value("geometry_included", false)
        || visual_model.value("model_counts", nlohmann::json::object()).value("geom", 0) < 2
        || !visual_model.contains("ncon")
        || visual_model.value("items", nlohmann::json::array()).empty()) {
      std::cerr << "visual model failed\n";
      return 1;
    }
    bool exported_mesh = false;
    for (const auto& item : visual_model["items"]) {
      if (item.value("shape", "") == "mesh"
          && item.value("vertices", nlohmann::json::array()).size() >= 12
          && item.value("faces", nlohmann::json::array()).size() >= 4) {
        exported_mesh = true;
      }
    }
    if (!exported_mesh) {
      std::cerr << "compiled mesh geometry missing\n";
      return 1;
    }
    auto visual_pose = manager.visual_model({{"id", "smoke"}, {"include_geometry", false}});
    if (visual_pose.value("geometry_included", true)
        || visual_pose["items"].front().contains("vertices")
        || !visual_pose["items"].front().contains("xmat")) {
      std::cerr << "lightweight visual pose failed\n";
      return 1;
    }
    auto inspection = manager.inspect_model({{"id", "smoke"}});
    if (inspection.value("controllable", true) || inspection["warnings"].empty()) {
      std::cerr << "model inspection failed\n";
      return 1;
    }
    auto joint_state = manager.joint_state({{"id", "smoke"}});
    if (!joint_state.contains("joints") || joint_state["joints"].empty()) {
      std::cerr << "joint state failed\n";
      return 1;
    }
    auto sensor_state = manager.sensor_state({{"id", "smoke"}});
    if (!sensor_state.contains("sensors") || sensor_state["sensors"].size() < 2) {
      std::cerr << "sensor state failed\n";
      return 1;
    }
    bool write_ctrl_failed = false;
    try {
      (void)manager.write_ctrl({{"id", "smoke"}, {"values", {{"0", 0.0}}}});
    } catch (const std::exception&) {
      write_ctrl_failed = true;
    }
    if (!write_ctrl_failed) {
      std::cerr << "write ctrl should fail for model without actuators\n";
      return 1;
    }
    if (created["model"].value("nsensordata", 0) < 6) {
      std::cerr << "sensor data missing\n";
      return 1;
    }
    if (created.value("step_hz", 0.0) != 120.0 || created.value("publish_hz", 0.0) != 12.0) {
      std::cerr << "scene defaults not applied\n";
      return 1;
    }

    auto updated_scene = scenes.update(
        {{"id", "falling_box_scene"},
         {"defaults",
          {{"step_hz", 180}, {"publish_hz", 18}, {"ctrl", scene["defaults"]["ctrl"]}}}});
    auto apply_diff = compiler.diff_scenes(scene, updated_scene);
    if (!apply_diff.value("can_apply_runtime", false)) {
      std::cerr << "scene update runtime diff failed\n";
      return 1;
    }
    auto runtime_state
        = manager.apply_runtime({{"id", "smoke"}, {"defaults", updated_scene["defaults"]}});
    if (runtime_state.value("step_hz", 0.0) != 180.0
        || runtime_state.value("publish_hz", 0.0) != 18.0) {
      std::cerr << "runtime apply failed\n";
      return 1;
    }

    auto stepped = manager.step({{"id", "smoke"}, {"count", 10}});
    if (stepped.value("time", 0.0) <= 0.0 || stepped.value("step_count", 0) != 10) {
      std::cerr << "manual step did not advance\n";
      return 1;
    }

    auto started
        = manager.start({{"id", "smoke"}, {"step_hz", 200.0}, {"publish_hz", 50.0}}, publisher);
    if (started.value("status", "") != "running") {
      std::cerr << "start failed\n";
      return 1;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(40));

    auto running = manager.state({{"id", "smoke"}});
    if (running.value("step_count", 0) <= stepped.value("step_count", 0)) {
      std::cerr << "auto step did not advance\n";
      return 1;
    }

    auto paused = manager.pause({{"id", "smoke"}});
    if (paused.value("status", "") != "paused") {
      std::cerr << "pause failed\n";
      return 1;
    }

    auto ctrl = manager.write_ctrl({{"id", "smoke"}, {"ctrl", nlohmann::json::array()}});
    if (!ctrl.contains("ctrl")) {
      std::cerr << "ctrl update failed\n";
      return 1;
    }

    auto listed = manager.list();
    if (!listed.is_array() || listed.empty()) {
      std::cerr << "list failed\n";
      return 1;
    }

    SimulationRecordManager records;
    auto record_started = records.start(
        {{"record_id", "smoke_record"}, {"instance_id", "smoke"}, {"sample_hz", 100.0}}, manager);
    if (record_started.value("status", "") != "running") {
      std::cerr << "record start failed\n";
      return 1;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(35));
    auto record_stopped = records.stop({{"record_id", "smoke_record"}});
    if (record_stopped.value("sample_count", 0) < 1
        || record_stopped.value("status", "") != "stopped") {
      std::cerr << "record stop failed\n";
      return 1;
    }
    if (records.info({{"record_id", "smoke_record"}}).value("sample_count", 0) < 1
        || records.list().empty()) {
      std::cerr << "record info/list failed\n";
      return 1;
    }
    if (!records.remove({{"record_id", "smoke_record"}}).value("removed", false)) {
      std::cerr << "record remove failed\n";
      return 1;
    }

    SimulationTaskManager tasks;
    auto task = tasks.create({
        {"id", "smoke_task"},
        {"instance_id", "smoke"},
        {"steps", nlohmann::json::array({
                      {{"op", "step"}, {"count", 2}},
                      {{"op", "set_ctrl"}, {"ctrl", nlohmann::json::array()}},
                      {{"op", "reset"}},
                  })},
    });
    if (task.value("status", "") != "created" || tasks.list().empty()) {
      std::cerr << "task create/list failed\n";
      return 1;
    }
    auto task_result = tasks.run({{"id", "smoke_task"}}, manager);
    if (task_result.value("status", "") != "done" || task_result["results"].size() != 3) {
      std::cerr << "task run failed\n";
      return 1;
    }
    if (tasks.info({{"id", "smoke_task"}}).value("status", "") != "done") {
      std::cerr << "task info failed\n";
      return 1;
    }
    if (!tasks.remove({{"id", "smoke_task"}}).value("removed", false)) {
      std::cerr << "task remove failed\n";
      return 1;
    }

    auto structural_updated_scene
        = scenes.update({{"id", "falling_box_scene"}, {"objects", structural_scene["objects"]}});
    auto structural_compiled
        = compiler.compile_scene(reloaded_registry.resolve_scene(structural_updated_scene));
    auto structural_config = compiler.build_instance_config(
        {{"id", "smoke"}, {"scene_id", "falling_box_scene"}}, structural_compiled);
    auto structural_model
        = compiler.get_compiled_model(structural_compiled.value("compiled_model_id", ""));
    auto structural_recreated = manager.recreate(structural_config, structural_model);
    if (!structural_recreated.value("recreated", false)
        || structural_recreated["state"].value("id", "") != "smoke") {
      std::cerr << "structural recreate failed\n";
      return 1;
    }

    auto recreated = manager.recreate(config, shared_model);
    if (!recreated.value("recreated", false) || recreated["state"].value("id", "") != "smoke") {
      std::cerr << "recreate failed\n";
      return 1;
    }

    auto stopped = manager.stop({{"id", "smoke"}});
    if (stopped.value("status", "") != "stopped") {
      std::cerr << "stop failed\n";
      return 1;
    }

    auto second_destroyed = manager.destroy({{"id", "smoke-2"}});
    if (!second_destroyed.value("destroyed", false)) {
      std::cerr << "second destroy failed\n";
      return 1;
    }

    auto destroyed = manager.destroy({{"id", "smoke"}});
    if (!destroyed.value("destroyed", false)) {
      std::cerr << "destroy failed\n";
      return 1;
    }
  } catch (const std::exception& e) {
    std::cerr << e.what() << "\n";
    return 1;
  }

  std::filesystem::remove_all(std::filesystem::temp_directory_path()
                              / "simulation_plugin_registry_smoke");
  return 0;
}
