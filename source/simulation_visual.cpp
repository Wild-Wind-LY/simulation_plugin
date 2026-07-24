#include "simulation_visual.hpp"

#include <algorithm>
#include <map>
#include <set>
#include <string>
#include <tuple>
#include <vector>

#include "simulation_json_utils.hpp"

namespace {

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

  // Base64-encodes a numeric buffer's raw bytes directly, instead of letting
  // nlohmann::json serialize it as a text array of numbers. A JSON number
  // array is much bigger than the underlying binary data -- float values in
  // particular round-trip through `double` and often print 15-17 significant
  // digits (e.g. "0.10000000149011612" for a plain 0.1f), and every value
  // still carries comma/digit overhead on top of that. Base64 of the raw
  // bytes is a fixed ~4/3 the binary size regardless of value "ugliness",
  // which is what actually keeps large meshes under the gateway's 8 MiB
  // response-body limit (dedup alone brought the vertex *count* back down,
  // but each remaining number was still costing far more as text than as
  // binary).
  template <typename T> std::string base64_encode_binary(const std::vector<T>& values) {
    return base64_encode(
        std::string(reinterpret_cast<const char*>(values.data()), values.size() * sizeof(T)));
  }

  // geom 的 2D 底色纹理 id（RGB 角色优先，其次 RGBA），非 2D（cube/skybox）返回 -1
  int material_2d_texture(const mjModel* model, int material_id) {
    if (material_id < 0 || material_id >= model->nmat) return -1;
    int tex = model->mat_texid[material_id * mjNTEXROLE + mjTEXROLE_RGB];
    if (tex < 0) tex = model->mat_texid[material_id * mjNTEXROLE + mjTEXROLE_RGBA];
    if (tex < 0 || tex >= model->ntex) return -1;
    return model->tex_type[tex] == mjTEXTURE_2D ? tex : -1;
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
    std::string geom_name = object_name(model, mjOBJ_GEOM, i);
    if (geom_name.empty()) geom_name = "geom_" + std::to_string(i);
    nlohmann::json item = {
        {"id", geom_name},
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
        // MuJoCo's own visualizer (`simulate`) lets a user toggle rendering per
        // geom group (0-5) -- most notably to hide URDF <collision> geoms
        // (group 0) while keeping <visual> geoms (group 1) visible. Previously
        // never sent, so this web preview had no way to tell the two apart and
        // always rendered both stacked on top of each other.
        {"group", model->geom_group[i]},
    };
    const int material_id = model->geom_matid[i];
    item["material_id"] = material_id;
    if (material_id >= 0 && material_id < model->nmat) {
      item["rgba"] = numeric_array(model->mat_rgba + 4 * material_id, 4);
      // Compiled by MuJoCo from the material's specular/shininess/reflectance/
      // emission attributes (compile_asset_declaration passes all four through
      // faithfully) -- previously never sent, so the web preview could only
      // ever flat-shade geoms by rgba while `simulate` renders these properly.
      item["specular"] = model->mat_specular[material_id];
      item["shininess"] = model->mat_shininess[material_id];
      item["reflectance"] = model->mat_reflectance[material_id];
      item["emission"] = model->mat_emission[material_id];
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
        const int face_adr = model->mesh_faceadr[mesh_id];
        const int face_count = model->mesh_facenum[mesh_id];
        const int normal_adr = model->mesh_normaladr[mesh_id];
        const int texcoord_adr = model->mesh_texcoordadr[mesh_id];  // -1: this mesh has no UVs
        const bool have_uv = texture_id >= 0 && texcoord_adr >= 0 && model->mesh_facetexcoord;

        // Deduplicate by (position, normal, uv) index triple rather than
        // blindly emitting one vertex per face-corner: MuJoCo's own
        // mesh_facenormal/mesh_facetexcoord let two faces that touch the same
        // *position* reference two different normals/UVs (hard edges, UV
        // seams), which a shared-index BufferGeometry can't represent with a
        // single normal per position -- but most of a typical mesh is smooth
        // (every face touching a position agrees on its normal), so most
        // corners collapse back down to one shared vertex just like before;
        // only genuine hard edges/seams cost an extra vertex. A prior version
        // of this export unconditionally emitted 3 fresh vertices per face
        // (no dedup at all), which roughly tripled payload size and started
        // tripping the gateway's 8 MiB response-body limit on non-trivial
        // meshes -- this keeps the same correctness fix at close to the
        // original size.
        std::vector<float> positions, normals, uvs;
        std::vector<int> indices;
        indices.reserve(static_cast<size_t>(face_count) * 3);
        std::map<std::tuple<int, int, int>, int> dedup;

        for (int face = 0; face < face_count; ++face) {
          const int* verts = model->mesh_face + 3 * (face_adr + face);
          const int* fnorm = model->mesh_facenormal + 3 * (face_adr + face);
          const int* ftex = have_uv ? model->mesh_facetexcoord + 3 * (face_adr + face) : nullptr;
          for (int corner = 0; corner < 3; ++corner) {
            const int pidx = verts[corner];
            const int nidx = fnorm[corner];
            const int tidx = have_uv ? ftex[corner] : -1;
            const auto key = std::make_tuple(pidx, nidx, tidx);
            const auto it = dedup.find(key);
            int compact;
            if (it != dedup.end()) {
              compact = it->second;
            } else {
              compact = static_cast<int>(positions.size() / 3);
              dedup.emplace(key, compact);
              const int v = vertex_adr + pidx;
              positions.push_back(model->mesh_vert[3 * v]);
              positions.push_back(model->mesh_vert[3 * v + 1]);
              positions.push_back(model->mesh_vert[3 * v + 2]);
              const int n = normal_adr + nidx;
              normals.push_back(model->mesh_normal[3 * n]);
              normals.push_back(model->mesh_normal[3 * n + 1]);
              normals.push_back(model->mesh_normal[3 * n + 2]);
              if (have_uv) {
                const int t = texcoord_adr + tidx;
                uvs.push_back(model->mesh_texcoord[2 * t]);
                uvs.push_back(model->mesh_texcoord[2 * t + 1]);
              }
            }
            indices.push_back(compact);
          }
        }
        // *_b64 = base64 of the raw float32/int32 bytes (little-endian, matching
        // every realistic browser target) -- see base64_encode_binary's comment.
        item["positions_b64"] = base64_encode_binary(positions);
        item["normals_b64"] = base64_encode_binary(normals);
        if (have_uv) item["uvs_b64"] = base64_encode_binary(uvs);
        item["indices_b64"] = base64_encode_binary(indices);
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
