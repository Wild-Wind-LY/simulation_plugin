#pragma once

#include <mujoco/mujoco.h>

#include <nlohmann/json.hpp>
#include <string>

// Small helpers shared by the instance state/metadata JSON (simulation_instance.cpp)
// and the visual/geometry JSON (simulation_visual.cpp) — kept in one place so a fix
// (e.g. the reserve() below) or a new geom type only needs to be applied once.

inline std::string object_name(const mjModel* model, int object_type, int id) {
  const char* name = mj_id2name(model, object_type, id);
  return name ? std::string{name} : std::string{};
}

inline nlohmann::json numeric_array(const float* values, int count) {
  nlohmann::json out = nlohmann::json::array();
  if (!values || count <= 0) return out;
  out.get_ref<nlohmann::json::array_t&>().reserve(static_cast<size_t>(count));
  for (int i = 0; i < count; ++i) {
    out.push_back(static_cast<double>(values[i]));
  }
  return out;
}

inline nlohmann::json numeric_array(const mjtNum* values, int count) {
  nlohmann::json out = nlohmann::json::array();
  if (!values || count <= 0) return out;
  out.get_ref<nlohmann::json::array_t&>().reserve(static_cast<size_t>(count));
  for (int i = 0; i < count; ++i) {
    out.push_back(static_cast<double>(values[i]));
  }
  return out;
}

inline std::string geom_shape(int type) {
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
