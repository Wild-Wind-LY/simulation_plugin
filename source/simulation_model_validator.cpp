#include "simulation_model_validator.hpp"
#include "simulation_mujoco_utils.hpp"

#include <mujoco/mujoco.h>

#include <algorithm>
#include <cctype>
#include <filesystem>

namespace {

  std::string object_name(const mjModel* model, int object_type, int id) {
    const char* name = mj_id2name(model, object_type, id);
    return name ? std::string{name} : std::string{};
  }

  std::filesystem::path normalize_path(const std::string& path_text) {
    std::filesystem::path path{path_text};
    if (path.is_relative()) path = std::filesystem::current_path() / path;
    return std::filesystem::weakly_canonical(path);
  }

}  // namespace

nlohmann::json SimulationModelValidator::validate(const nlohmann::json& data) const {
  nlohmann::json errors = nlohmann::json::array();
  nlohmann::json warnings = nlohmann::json::array();

  const std::string path_text = data.value("model_path", "");
  nlohmann::json out = {
      {"valid", false},          {"controllable", false},
      {"model_path", path_text}, {"format", simulation_detect_model_format(path_text)},
      {"errors", errors},        {"warnings", warnings},
  };

  if (path_text.empty()) {
    out["errors"].push_back("missing 'model_path'");
    return out;
  }

  std::filesystem::path model_path;
  try {
    model_path = normalize_path(path_text);
    out["model_path"] = model_path.string();
  } catch (const std::exception& e) {
    out["errors"].push_back(e.what());
    return out;
  }

  if (!std::filesystem::exists(model_path)) {
    out["errors"].push_back("model_path does not exist");
    return out;
  }

  char error[2048] = {};
  std::lock_guard xml_lock{simulation_mujoco_xml_mutex()};
  mjModel* model = mj_loadXML(model_path.string().c_str(), nullptr, error, sizeof(error));
  if (!model) {
    out["errors"].push_back(error[0] ? error : "failed to load model");
    return out;
  }

  out["valid"] = true;
  out["controllable"] = model->nu > 0;
  out["sizes"] = {
      {"nq", model->nq},
      {"nv", model->nv},
      {"nu", model->nu},
      {"njnt", model->njnt},
      {"nbody", model->nbody},
      {"nsensor", model->nsensor},
      {"nsensordata", model->nsensordata},
  };

  if (model->nu == 0)
    out["warnings"].push_back("model has no actuators; ctrl commands cannot move it");
  if (model->njnt == 0) out["warnings"].push_back("model has no joints");
  if (model->nsensor == 0) out["warnings"].push_back("model has no sensors");
  if (out.value("format", "") == "urdf" && model->nu == 0) {
    out["warnings"].push_back("URDF loaded, but no MuJoCo actuators were created");
  }

  out["joints"] = nlohmann::json::array();
  for (int i = 0; i < model->njnt; ++i) {
    out["joints"].push_back({
        {"id", i},
        {"name", object_name(model, mjOBJ_JOINT, i)},
        {"type", model->jnt_type[i]},
        {"qpos_adr", model->jnt_qposadr[i]},
        {"dof_adr", model->jnt_dofadr[i]},
    });
  }

  out["actuators"] = nlohmann::json::array();
  for (int i = 0; i < model->nu; ++i) {
    out["actuators"].push_back({
        {"id", i},
        {"name", object_name(model, mjOBJ_ACTUATOR, i)},
        {"ctrl_index", i},
    });
  }

  out["sensors"] = nlohmann::json::array();
  for (int i = 0; i < model->nsensor; ++i) {
    out["sensors"].push_back({
        {"id", i},
        {"name", object_name(model, mjOBJ_SENSOR, i)},
        {"type", model->sensor_type[i]},
        {"data_adr", model->sensor_adr[i]},
        {"data_dim", model->sensor_dim[i]},
    });
  }

  mj_deleteModel(model);
  return out;
}
