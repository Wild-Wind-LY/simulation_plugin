#include "simulation_visual.hpp"

#include <algorithm>
#include <set>
#include <string>
#include <vector>

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

  nlohmann::json numeric_array(const mjtNum* values, int count) {
    nlohmann::json out = nlohmann::json::array();
    if (!values || count <= 0) return out;

    for (int i = 0; i < count; ++i) {
      out.push_back(static_cast<double>(values[i]));
    }
    return out;
  }

  std::string base64_encode(const std::string& in) {
    static const char* chars = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
    std::string out;
    out.reserve(((in.size() + 2) / 3) * 4);
    int val = 0;
    int valb = -6;
    for (unsigned char c : in) {
      val = (val << 8) + c;
      valb += 8;
      while (valb >= 0) {
        out.push_back(chars[(val >> valb) & 0x3F]);
        valb -= 6;
      }
    }
    if (valb > -6) out.push_back(chars[((val << 8) >> (valb + 8)) & 0x3F]);
    while (out.size() % 4) out.push_back('=');
    return out;
  }

  // geom 的 2D 底色纹理 id（RGB 角色优先，其次 RGBA），非 2D（cube/skybox）返回 -1
  int material_2d_texture(const mjModel* model, int material_id) {
    if (material_id < 0 || material_id >= model->nmat) return -1;
    int tex = model->mat_texid[material_id * mjNTEXROLE + mjTEXROLE_RGB];
    if (tex < 0) tex = model->mat_texid[material_id * mjNTEXROLE + mjTEXROLE_RGBA];
    if (tex < 0 || tex >= model->ntex) return -1;
    return model->tex_type[tex] == mjTEXTURE_2D ? tex : -1;
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

}  // namespace

nlohmann::json simulation_visual_json(const mjModel* model, const mjData* data,
                                      bool include_geometry) {
  nlohmann::json out = {
      {"backend", "mujoco"},
      {"units", "m"},
      {"coordinate_system", "mujoco_xyz"},
      {"items", nlohmann::json::array()},
  };
  if (!model || !data) return out;

  out["geometry_included"] = include_geometry;
  out["ncon"] = data->ncon;
  out["ngeom"] = model->ngeom;
  out["model_counts"] = {
      {"body", model->nbody},
      {"joint", model->njnt},
      {"dof", model->nv},
      {"geom", model->ngeom},
      {"site", model->nsite},
      {"actuator", model->nu},
      {"sensor", model->nsensor},
      {"tendon", model->ntendon},
      {"equality", model->neq},
      {"contact_pair", model->npair},
      {"contact_exclude", model->nexclude},
      {"mesh", model->nmesh},
      {"height_field", model->nhfield},
      {"material", model->nmat},
      {"texture", model->ntex},
      {"camera", model->ncam},
      {"light", model->nlight},
      {"nq", model->nq},
      {"nv", model->nv},
      {"na", model->na},
      {"nu", model->nu},
  };
  // 具名 site 与世界坐标：前端用于传感器 site 下拉和把传感器标记画在真实位置。
  // 未命名 site 无法被 MJCF 传感器引用，直接跳过。
  out["sites"] = nlohmann::json::array();
  for (int i = 0; i < model->nsite; ++i) {
    const auto name = object_name(model, mjOBJ_SITE, i);
    if (name.empty()) continue;
    const int body_id = model->site_bodyid[i];
    out["sites"].push_back({
        {"site_id", i},
        {"id", name},
        {"body", body_id >= 0 ? object_name(model, mjOBJ_BODY, body_id) : std::string{}},
        {"pos", numeric_array(data->site_xpos + 3 * i, 3)},
    });
  }
  std::set<int> used_textures;
  for (int i = 0; i < model->ngeom; ++i) {
    const int type = model->geom_type[i];
    const int body_id = model->geom_bodyid[i];
    nlohmann::json item = {
        {"id", object_name(model, mjOBJ_GEOM, i).empty() ? "geom_" + std::to_string(i)
                                                         : object_name(model, mjOBJ_GEOM, i)},
        {"geom_id", i},
        {"body_id", body_id},
        {"body", body_id >= 0 ? object_name(model, mjOBJ_BODY, body_id) : std::string{}},
        {"source", "mujoco.model"},
        {"kind", "geom"},
        {"shape", geom_shape(type)},
        {"geom_type", type},
        {"pos", numeric_array(data->geom_xpos + 3 * i, 3)},
        {"xmat", numeric_array(data->geom_xmat + 9 * i, 9)},
        {"size", numeric_array(model->geom_size + 3 * i, 3)},
        {"rgba", numeric_array(model->geom_rgba + 4 * i, 4)},
    };
    const int material_id = model->geom_matid[i];
    item["material_id"] = material_id;
    if (material_id >= 0 && material_id < model->nmat) {
      item["rgba"] = numeric_array(model->mat_rgba + 4 * material_id, 4);
    }
    int texture_id = -1;
    if (include_geometry) {
      texture_id = material_2d_texture(model, material_id);
      if (texture_id >= 0) {
        item["texture_id"] = texture_id;
        item["texrepeat"] = numeric_array(model->mat_texrepeat + 2 * material_id, 2);
        item["texuniform"] = static_cast<bool>(model->mat_texuniform[material_id]);
        used_textures.insert(texture_id);
      }
    }
    if (include_geometry && type == mjGEOM_MESH) {
      const int mesh_id = model->geom_dataid[i];
      item["mesh_id"] = mesh_id;
      if (mesh_id >= 0 && mesh_id < model->nmesh) {
        const int vertex_adr = model->mesh_vertadr[mesh_id];
        const int vertex_count = model->mesh_vertnum[mesh_id];
        const int face_adr = model->mesh_faceadr[mesh_id];
        const int face_count = model->mesh_facenum[mesh_id];
        item["vertices"] = numeric_array(model->mesh_vert + 3 * vertex_adr, 3 * vertex_count);
        nlohmann::json faces = nlohmann::json::array();
        for (int face = 0; face < face_count; ++face) {
          const int* indices = model->mesh_face + 3 * (face_adr + face);
          faces.push_back({indices[0], indices[1], indices[2]});
        }
        item["faces"] = std::move(faces);
        // 有贴图且 mesh 带 texcoord：给出按顶点对齐的 UV（拓扑不一致时按面角近似回填）
        if (texture_id >= 0 && model->mesh_texcoordadr[mesh_id] >= 0) {
          const int texcoord_adr = model->mesh_texcoordadr[mesh_id];
          const int texcoord_num = model->mesh_texcoordnum[mesh_id];
          std::vector<float> uvs(static_cast<size_t>(vertex_count) * 2, 0.0f);
          bool have_uvs = false;
          if (texcoord_num == vertex_count) {
            for (int v = 0; v < vertex_count; ++v) {
              uvs[2 * v] = model->mesh_texcoord[2 * (texcoord_adr + v)];
              uvs[2 * v + 1] = model->mesh_texcoord[2 * (texcoord_adr + v) + 1];
            }
            have_uvs = true;
          } else if (model->mesh_facetexcoord) {
            for (int face = 0; face < face_count; ++face) {
              const int* face_verts = model->mesh_face + 3 * (face_adr + face);
              const int* face_tc = model->mesh_facetexcoord + 3 * (face_adr + face);
              for (int corner = 0; corner < 3; ++corner) {
                const int v = face_verts[corner];
                const int t = face_tc[corner];
                if (v >= 0 && v < vertex_count && t >= 0 && t < texcoord_num) {
                  uvs[2 * v] = model->mesh_texcoord[2 * (texcoord_adr + t)];
                  uvs[2 * v + 1] = model->mesh_texcoord[2 * (texcoord_adr + t) + 1];
                  have_uvs = true;
                }
              }
            }
          }
          if (have_uvs) {
            nlohmann::json uv_json = nlohmann::json::array();
            for (float v : uvs) uv_json.push_back(static_cast<double>(v));
            item["uvs"] = std::move(uv_json);
          }
        }
      }
    } else if (include_geometry && type == mjGEOM_HFIELD) {
      const int hfield_id = model->geom_dataid[i];
      item["hfield_id"] = hfield_id;
      if (hfield_id >= 0 && hfield_id < model->nhfield) {
        const int rows = model->hfield_nrow[hfield_id];
        const int columns = model->hfield_ncol[hfield_id];
        const int data_adr = model->hfield_adr[hfield_id];
        item["hfield"] = {
            {"rows", rows},
            {"columns", columns},
            {"size", numeric_array(model->hfield_size + 4 * hfield_id, 4)},
            {"data", numeric_array(model->hfield_data + data_adr, rows * columns)},
        };
      }
    }
    out["items"].push_back(std::move(item));
  }
  // 被引用的 2D 纹理：最长边降采样到 512 以内后按原始通道 base64 传输
  if (!used_textures.empty()) {
    nlohmann::json textures = nlohmann::json::array();
    for (int t : used_textures) {
      const int width = model->tex_width[t];
      const int height = model->tex_height[t];
      const int channels = model->tex_nchannel[t];
      if (width <= 0 || height <= 0 || channels <= 0 || channels > 4) continue;
      const mjtByte* src = model->tex_data + model->tex_adr[t];
      constexpr int kMaxSide = 512;
      const int stride = std::max(1, (std::max(width, height) + kMaxSide - 1) / kMaxSide);
      const int out_width = std::max(1, width / stride);
      const int out_height = std::max(1, height / stride);
      std::string raw;
      raw.reserve(static_cast<size_t>(out_width) * out_height * channels);
      for (int row = 0; row < out_height; ++row) {
        for (int col = 0; col < out_width; ++col) {
          const mjtByte* pixel
              = src
                + (static_cast<size_t>(row) * stride * width + static_cast<size_t>(col) * stride)
                      * channels;
          raw.append(reinterpret_cast<const char*>(pixel), channels);
        }
      }
      textures.push_back({
          {"id", t},
          {"width", out_width},
          {"height", out_height},
          {"nchannel", channels},
          {"data_base64", base64_encode(raw)},
      });
    }
    if (!textures.empty()) out["textures"] = std::move(textures);
  }
  return out;
}
