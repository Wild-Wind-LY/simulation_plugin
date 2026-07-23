#include "simulation_compiler.hpp"

#include <algorithm>
#include <cctype>
#include <fstream>
#include <functional>
#include <iomanip>
#include <optional>
#include <set>
#include <sstream>
#include <stdexcept>
#include <system_error>
#include <utility>
#include <vector>

#include "simulation_hash.hpp"
#include "simulation_mujoco_utils.hpp"
#include "simulation_paths.hpp"

namespace {

  // ------------------------- generic xml/value helpers -------------------------
  // Stateless (no instance data touched), so these live as free functions
  // rather than SimulationCompiler members -- every new MJCF element type below
  // reuses them without needing its own header declaration.

  std::string xml_escape(const std::string& value) {
    std::string out;
    out.reserve(value.size());
    for (const char c : value) {
      switch (c) {
        case '&':
          out += "&amp;";
          break;
        case '"':
          out += "&quot;";
          break;
        case '<':
          out += "&lt;";
          break;
        case '>':
          out += "&gt;";
          break;
        default:
          out.push_back(c);
          break;
      }
    }
    return out;
  }

  std::string numeric_list(const nlohmann::json& values, const std::string& fallback,
                           int expected_size) {
    if (!values.is_array() || values.empty()) return fallback;
    if (expected_size > 0 && static_cast<int>(values.size()) != expected_size) {
      throw std::invalid_argument("numeric list size mismatch");
    }

    std::ostringstream out;
    for (size_t i = 0; i < values.size(); ++i) {
      if (i > 0) out << ' ';
      out << values.at(i).get<double>();
    }
    return out.str();
  }

  // `class="..."` attribute string (with leading space) for an element that
  // opted into a defaultClasses[] entry, or empty if it didn't. Shared by
  // every instance compiler (joint/geom/site/camera/actuator) that accepts a
  // `class` field; actual cross-reference validity is checked in validate_scene.
  std::string class_attr(const nlohmann::json& obj) {
    const std::string cls = obj.value("class", "");
    if (cls.empty()) return {};
    return " class=\"" + xml_escape(cls) + "\"";
  }

  // ---------------- small attribute-emission helpers ----------------
  // Shared by compiler/size/statistic/visual/actuator, which are largely long
  // lists of "if present and of the right JSON kind, write it" attributes with
  // no cross-field logic -- these collapse each such line to one call.
  void emit_num_attr(std::ostringstream& out, const nlohmann::json& obj, const char* key) {
    if (obj.contains(key) && obj[key].is_number())
      out << " " << key << "=\"" << obj[key].get<double>() << "\"";
  }
  void emit_int_attr(std::ostringstream& out, const nlohmann::json& obj, const char* key) {
    if (obj.contains(key) && obj[key].is_number_integer())
      out << " " << key << "=\"" << obj[key].get<int>() << "\"";
  }
  void emit_bool_attr(std::ostringstream& out, const nlohmann::json& obj, const char* key) {
    if (obj.contains(key) && obj[key].is_boolean())
      out << " " << key << "=\"" << (obj[key].get<bool>() ? "true" : "false") << "\"";
  }
  void emit_str_attr(std::ostringstream& out, const nlohmann::json& obj, const char* key) {
    if (obj.contains(key) && obj[key].is_string())
      out << " " << key << "=\"" << xml_escape(obj[key].get<std::string>()) << "\"";
  }
  void emit_vec_attr(std::ostringstream& out, const nlohmann::json& obj, const char* key,
                     int count) {
    if (obj.contains(key)) out << " " << key << "=\"" << numeric_list(obj[key], "", count) << "\"";
  }

  nlohmann::json numeric_array(const nlohmann::json& values, std::initializer_list<double> fallback,
                               int expected_size) {
    if (values.is_array() && !values.empty()) {
      if (expected_size > 0 && static_cast<int>(values.size()) != expected_size) {
        throw std::invalid_argument("numeric array size mismatch");
      }
      nlohmann::json out = nlohmann::json::array();
      for (const auto& value : values) out.push_back(value.get<double>());
      return out;
    }

    nlohmann::json out = nlohmann::json::array();
    for (const double value : fallback) out.push_back(value);
    return out;
  }

  std::filesystem::path resolve_scene_path(const nlohmann::json& scene,
                                           const std::string& path_text) {
    if (path_text.empty()) throw std::invalid_argument("missing source path");

    std::filesystem::path path{path_text};
    if (path.is_relative()) {
      const std::filesystem::path scene_path{scene.value("path", "")};
      const auto base_dir
          = scene_path.empty() ? std::filesystem::current_path() : scene_path.parent_path();
      path = base_dir / path;
    }
    return std::filesystem::weakly_canonical(path);
  }

  void add_issue(nlohmann::json& issues, const std::string& path, const std::string& message) {
    issues.push_back({{"path", path}, {"message", message}});
  }

  // ---------------- sensor type registry ----------------
  // Every MuJoCo sensor reads from one of a handful of reference "shapes",
  // confirmed against MuJoCo's own XML reader (xml_native_reader.cc's sensor
  // section, ~line 4020-4380). Adding a sensor type is a one-line table entry
  // as long as it fits an existing shape; only a genuinely new reference shape
  // needs a new case in compile_sensor_object()/validate_sensor_fields().
  //
  // Deliberately not covered: `contact` (its own multi-criteria data/reduce/num
  // mini-language, not a simple reference), `tactile`/`user` (these read from
  // an `mjcb_sensor` callback the scene JSON has no way to supply). `plugin`
  // *is* covered (kPlugin, sensor.plugin below) now that scene.plugins[] gives
  // the JSON a way to declare an <extension> instance to reference.
  enum class SensorShape {
    kSimple,         // one attribute, name == ref_kind: site|joint|tendon|actuator|body
    kFrame,          // objtype+objname (body/xbody/geom/site/camera), optional reftype+refname
    kCamProjection,  // site + camera
    kInsideSite,     // site + objtype/objname
    kGeomPair,       // (geom1|body1) + (geom2|body2)
    kNone,           // no reference at all
    kPlugin,         // plugin_instance (+ optional objtype/objname), see sensor.plugin below
  };

  struct SensorSpec {
    const char* mjcf_tag;
    SensorShape shape;
    const char* ref_kind = "";        // kSimple only
    bool frame_ref_optional = false;  // kFrame only: whether reftype/refname is accepted
  };

  const std::unordered_map<std::string, SensorSpec>& sensor_registry() {
    static const std::unordered_map<std::string, SensorSpec> table = {
        // site-attached
        {"sensor.force", {"force", SensorShape::kSimple, "site"}},
        {"sensor.torque", {"torque", SensorShape::kSimple, "site"}},
        {"sensor.touch", {"touch", SensorShape::kSimple, "site"}},
        {"sensor.accelerometer", {"accelerometer", SensorShape::kSimple, "site"}},
        {"sensor.velocimeter", {"velocimeter", SensorShape::kSimple, "site"}},
        {"sensor.gyro", {"gyro", SensorShape::kSimple, "site"}},
        {"sensor.magnetometer", {"magnetometer", SensorShape::kSimple, "site"}},
        {"sensor.rangefinder", {"rangefinder", SensorShape::kSimple, "site"}},
        // joint-attached
        {"sensor.jointpos", {"jointpos", SensorShape::kSimple, "joint"}},
        {"sensor.jointvel", {"jointvel", SensorShape::kSimple, "joint"}},
        {"sensor.jointactuatorfrc", {"jointactuatorfrc", SensorShape::kSimple, "joint"}},
        {"sensor.ballquat", {"ballquat", SensorShape::kSimple, "joint"}},
        {"sensor.ballangvel", {"ballangvel", SensorShape::kSimple, "joint"}},
        {"sensor.jointlimitpos", {"jointlimitpos", SensorShape::kSimple, "joint"}},
        {"sensor.jointlimitvel", {"jointlimitvel", SensorShape::kSimple, "joint"}},
        {"sensor.jointlimitfrc", {"jointlimitfrc", SensorShape::kSimple, "joint"}},
        // tendon-attached
        {"sensor.tendonpos", {"tendonpos", SensorShape::kSimple, "tendon"}},
        {"sensor.tendonvel", {"tendonvel", SensorShape::kSimple, "tendon"}},
        {"sensor.tendonactuatorfrc", {"tendonactuatorfrc", SensorShape::kSimple, "tendon"}},
        {"sensor.tendonlimitpos", {"tendonlimitpos", SensorShape::kSimple, "tendon"}},
        {"sensor.tendonlimitvel", {"tendonlimitvel", SensorShape::kSimple, "tendon"}},
        {"sensor.tendonlimitfrc", {"tendonlimitfrc", SensorShape::kSimple, "tendon"}},
        // actuator-attached
        {"sensor.actuatorpos", {"actuatorpos", SensorShape::kSimple, "actuator"}},
        {"sensor.actuatorvel", {"actuatorvel", SensorShape::kSimple, "actuator"}},
        {"sensor.actuatorfrc", {"actuatorfrc", SensorShape::kSimple, "actuator"}},
        // subtree, rooted at a body
        {"sensor.subtreecom", {"subtreecom", SensorShape::kSimple, "body"}},
        {"sensor.subtreelinvel", {"subtreelinvel", SensorShape::kSimple, "body"}},
        {"sensor.subtreeangmom", {"subtreeangmom", SensorShape::kSimple, "body"}},
        // frame family: generic objtype (body/xbody/geom/site/camera); framelinacc/
        // frameangacc are the two the reader does not accept an optional ref frame for.
        {"sensor.framepos", {"framepos", SensorShape::kFrame, "", true}},
        {"sensor.framequat", {"framequat", SensorShape::kFrame, "", true}},
        {"sensor.framexaxis", {"framexaxis", SensorShape::kFrame, "", true}},
        {"sensor.frameyaxis", {"frameyaxis", SensorShape::kFrame, "", true}},
        {"sensor.framezaxis", {"framezaxis", SensorShape::kFrame, "", true}},
        {"sensor.framelinvel", {"framelinvel", SensorShape::kFrame, "", true}},
        {"sensor.frameangvel", {"frameangvel", SensorShape::kFrame, "", true}},
        {"sensor.framelinacc", {"framelinacc", SensorShape::kFrame, "", false}},
        {"sensor.frameangacc", {"frameangacc", SensorShape::kFrame, "", false}},
        // special dual-reference shapes
        {"sensor.camprojection", {"camprojection", SensorShape::kCamProjection}},
        {"sensor.insidesite", {"insidesite", SensorShape::kInsideSite}},
        {"sensor.distance", {"distance", SensorShape::kGeomPair}},
        {"sensor.normal", {"normal", SensorShape::kGeomPair}},
        {"sensor.fromto", {"fromto", SensorShape::kGeomPair}},
        // no reference
        {"sensor.clock", {"clock", SensorShape::kNone}},
        {"sensor.e_potential", {"e_potential", SensorShape::kNone}},
        {"sensor.e_kinetic", {"e_kinetic", SensorShape::kNone}},
        // plugin-backed: references a scene.plugins[] instance (+ optional
        // objtype/objname, e.g. mujoco.sensor.touch_grid's target site).
        {"sensor.plugin", {"plugin", SensorShape::kPlugin}},
    };
    return table;
  }

  bool is_sensor_type(const std::string& type) { return sensor_registry().count(type) != 0; }

  // Shared by validate_scene()'s objects[]/sensors[] loops -- checks that a
  // sensor's reference fields (whichever ones its shape requires) are present
  // and well-formed. `path` is the JSON-pointer-ish path already built by the
  // caller for this sensor entry.
  void validate_sensor_fields(nlohmann::json& errors, const std::string& path,
                              const nlohmann::json& sensor, const std::string& type) {
    const auto& spec = sensor_registry().at(type);
    switch (spec.shape) {
      case SensorShape::kSimple:
        if (sensor.value(spec.ref_kind, "").empty())
          add_issue(errors, path + "." + spec.ref_kind,
                    std::string("sensor missing ") + spec.ref_kind);
        break;
      case SensorShape::kFrame: {
        static const std::set<std::string> kObjTypes = {"body", "xbody", "geom", "site", "camera"};
        const std::string objtype = sensor.value("objtype", "");
        if (!kObjTypes.count(objtype))
          add_issue(errors, path + ".objtype",
                    "objtype must be one of: body, xbody, geom, site, camera");
        if (sensor.value("objname", "").empty())
          add_issue(errors, path + ".objname", "sensor missing objname");
        if (spec.frame_ref_optional && !sensor.value("reftype", "").empty()) {
          if (!kObjTypes.count(sensor.value("reftype", "")))
            add_issue(errors, path + ".reftype",
                      "reftype must be one of: body, xbody, geom, site, camera");
          if (sensor.value("refname", "").empty())
            add_issue(errors, path + ".refname", "reftype given but refname is missing");
        }
        break;
      }
      case SensorShape::kCamProjection:
        if (sensor.value("site", "").empty())
          add_issue(errors, path + ".site", "sensor missing site");
        if (sensor.value("camera", "").empty())
          add_issue(errors, path + ".camera", "sensor missing camera");
        break;
      case SensorShape::kInsideSite:
        if (sensor.value("site", "").empty())
          add_issue(errors, path + ".site", "sensor missing site");
        if (sensor.value("objtype", "").empty())
          add_issue(errors, path + ".objtype", "sensor missing objtype");
        if (sensor.value("objname", "").empty())
          add_issue(errors, path + ".objname", "sensor missing objname");
        break;
      case SensorShape::kGeomPair: {
        const bool has_geom1 = !sensor.value("geom1", "").empty();
        const bool has_body1 = !sensor.value("body1", "").empty();
        const bool has_geom2 = !sensor.value("geom2", "").empty();
        const bool has_body2 = !sensor.value("body2", "").empty();
        if (has_geom1 == has_body1)
          add_issue(errors, path, "sensor requires exactly one of geom1/body1");
        if (has_geom2 == has_body2)
          add_issue(errors, path, "sensor requires exactly one of geom2/body2");
        break;
      }
      case SensorShape::kNone:
        break;
      case SensorShape::kPlugin:
        // `plugin_instance` cross-references scene.plugins[]; that requires
        // scene-level context this free function doesn't have, so validate_scene
        // checks it separately (see check_plugin_ref call sites). objtype/objname
        // are optional passthrough (not every sensor plugin needs a target object).
        break;
    }
  }

  // Body-name references a sensor entry's shape may carry, used by the
  // <replicate> cross-reference guard in validate_scene: a replicated body's
  // scene id can't be resolved to one MJCF body name post-compile, so nothing
  // is allowed to reference it by name, including these sensor fields.
  std::vector<std::string> sensor_body_refs(const nlohmann::json& sensor, const SensorSpec& spec) {
    std::vector<std::string> refs;
    switch (spec.shape) {
      case SensorShape::kSimple:
        if (std::string(spec.ref_kind) == "body") {
          const auto v = sensor.value("body", "");
          if (!v.empty()) refs.push_back(v);
        }
        break;
      case SensorShape::kFrame: {
        const std::string objtype = sensor.value("objtype", "");
        if (objtype == "body" || objtype == "xbody") {
          const auto v = sensor.value("objname", "");
          if (!v.empty()) refs.push_back(v);
        }
        const std::string reftype = sensor.value("reftype", "");
        if (reftype == "body" || reftype == "xbody") {
          const auto v = sensor.value("refname", "");
          if (!v.empty()) refs.push_back(v);
        }
        break;
      }
      case SensorShape::kGeomPair: {
        const auto b1 = sensor.value("body1", "");
        const auto b2 = sensor.value("body2", "");
        if (!b1.empty()) refs.push_back(b1);
        if (!b2.empty()) refs.push_back(b2);
        break;
      }
      default:
        break;
    }
    return refs;
  }

  // ---------------- MuJoCo plugin instances (scene.plugins[]) ----------------
  // Generic passthrough onto MJCF's <extension><plugin><instance><config/></...>
  // structure (confirmed against xml_native_reader.cc's Extension()/OnePlugin(),
  // ~line 2934-2964 and 2817-2827): this schema doesn't understand what any
  // given plugin's config keys mean, only that instances declared here exist
  // and are named uniquely -- geoms/actuators/sensors reference one by id via
  // `plugin_instance`, and this resolves that id to the ` plugin="..."
  // instance="..."` attribute pair MuJoCo's reader expects on the referencing
  // element. Callers rely on validate_scene having already confirmed the
  // reference exists (check_plugin_ref), so this throws only as a last resort.
  std::string plugin_ref_attrs(const nlohmann::json& scene, const std::string& instance_id) {
    for (const auto& plugin : scene.value("plugins", nlohmann::json::array())) {
      if (plugin.is_object() && plugin.value("id", "") == instance_id) {
        return " plugin=\"" + xml_escape(plugin.value("plugin", "")) + "\" instance=\""
               + xml_escape(instance_id) + "\"";
      }
    }
    throw std::invalid_argument("plugin_instance references unknown plugin: " + instance_id);
  }

  // ---------------- 环境全景：schema 归一化与取值映射 ----------------

  // ground: 缺省/true -> 空对象（默认地面）；false -> null（不生成）；对象 -> 原样
  nlohmann::json ground_config(const nlohmann::json& environment) {
    if (!environment.contains("ground")) return nlohmann::json::object();
    const auto& ground = environment["ground"];
    if (ground.is_boolean())
      return ground.get<bool>() ? nlohmann::json::object() : nlohmann::json();
    if (ground.is_object()) return ground;
    return nlohmann::json::object();
  }

  // lights 数组优先；否则退回旧式 light 布尔简写（true -> 一盏默认顶光）
  nlohmann::json scene_lights(const nlohmann::json& environment) {
    if (environment.contains("lights") && environment["lights"].is_array())
      return environment["lights"];
    if (environment.value("light", true)) {
      return nlohmann::json::array(
          {{{"pos", {0, 0, 3}}, {"dir", {0, 0, -1}}, {"directional", true}}});
    }
    return nlohmann::json::array();
  }

  bool skybox_enabled(const nlohmann::json& environment) {
    if (!environment.contains("skybox")) return false;
    const auto& skybox = environment["skybox"];
    return skybox.is_object() || (skybox.is_boolean() && skybox.get<bool>());
  }

  std::optional<std::string> canonical_integrator(std::string value) {
    std::transform(value.begin(), value.end(), value.begin(), ::tolower);
    if (value == "euler") return std::string{"Euler"};
    if (value == "rk4") return std::string{"RK4"};
    if (value == "implicit") return std::string{"implicit"};
    if (value == "implicitfast") return std::string{"implicitfast"};
    return std::nullopt;
  }

  std::optional<std::string> canonical_solver(std::string value) {
    std::transform(value.begin(), value.end(), value.begin(), ::tolower);
    if (value == "pgs") return std::string{"PGS"};
    if (value == "cg") return std::string{"CG"};
    if (value == "newton") return std::string{"Newton"};
    return std::nullopt;
  }

  std::optional<std::string> canonical_cone(std::string value) {
    std::transform(value.begin(), value.end(), value.begin(), ::tolower);
    if (value == "pyramidal") return std::string{"pyramidal"};
    if (value == "elliptic") return std::string{"elliptic"};
    return std::nullopt;
  }

  std::optional<std::string> canonical_jacobian(std::string value) {
    std::transform(value.begin(), value.end(), value.begin(), ::tolower);
    if (value == "dense") return std::string{"dense"};
    if (value == "sparse") return std::string{"sparse"};
    if (value == "auto") return std::string{"auto"};
    return std::nullopt;
  }

  // MJCF <option><flag .../></option> switch names (xml_native_writer.cc:1042-1071).
  // disable/enable share no names (each flag only makes sense in one direction),
  // so a single set covers both physics.flags.disable[] and .enable[].
  const std::set<std::string>& known_option_flags() {
    static const std::set<std::string> kFlags
        = {"constraint",   "equality",  "frictionloss", "limit",     "contact",
           "spring",       "damper",    "gravity",      "clampctrl", "warmstart",
           "filterparent", "actuation", "refsafe",      "sensor",    "midphase",
           "eulerdamp",    "autoreset", "nativeccd",    "island",    "override",
           "energy",       "fwdinv",    "invdiscrete",  "multiccd",  "sleep"};
    return kFlags;
  }

  // Runtime-only keys are excluded from the structural signature (and so from
  // cache invalidation / recreate-on-apply decisions); every other top-level
  // scene key counts as structural by default. This is a deliberate inversion
  // of "list the fields that matter" into "list the fields that don't" -- a
  // newly added top-level field (equality/tendon/actuator/keyframe/bodies/...)
  // is structural without anyone needing to remember to register it here.
  // `defaults` stays on the list even though schema_version>=2 scenes no
  // longer populate it as runtime state (see initial_state below): it's
  // reserved for a future MJCF <default>/class tree, which will also be
  // runtime-irrelevant to compilation-cache validity in the sense that class
  // definitions are structural on their own merits, not via this key.
  nlohmann::json structural_signature(const nlohmann::json& scene) {
    // `schema_version` is stamped by SimulationSceneManager's normalize_schema
    // and never read by any compile_*/generate_* function below, so it has zero
    // effect on compiled output; without this it counts as structural by the
    // "list what doesn't matter" inversion, and an apply patch that happens to
    // carry an explicit schema_version would force an unnecessary instance
    // recreate for a field that can't possibly have changed the compiled model.
    static const std::set<std::string> kRuntimeOnlyKeys
        = {"initial_state", "defaults", "path", "name", "schema_version"};
    nlohmann::json signature = nlohmann::json::object();
    if (!scene.is_object()) return signature;
    for (const auto& [key, value] : scene.items()) {
      if (kRuntimeOnlyKeys.count(key)) continue;
      signature[key] = value;
    }
    return signature;
  }

  // scene.defaults (legacy) / scene.initial_state (schema_version>=2) is the
  // instance's initial qpos/qvel/ctrl + step/publish rates -- never MJCF
  // structure, so every reader goes through this one fallback.
  nlohmann::json runtime_state(const nlohmann::json& scene) {
    return scene.contains("initial_state") ? scene.at("initial_state")
                                           : scene.value("defaults", nlohmann::json::object());
  }

  void append_change(nlohmann::json& changes, const std::string& path, const std::string& type) {
    changes.push_back({{"path", path}, {"type", type}});
  }

  // ---------------- primitive / sensor object compilation ----------------

  std::string compile_primitive_object(const nlohmann::json& object) {
    if (!object.is_object()) throw std::invalid_argument("scene object must be an object");

    const std::string id = object.value("id", "");
    const std::string type = object.value("type", "");
    if (id.empty()) throw std::invalid_argument("scene object missing 'id'");

    std::string geom_type;
    std::string size_default;
    int size_count = 0;
    bool is_mesh = false;
    bool is_hfield = false;
    if (type == "obstacle.box") {
      geom_type = "box";
      size_default = "0.1 0.1 0.1";
      size_count = 3;
    } else if (type == "obstacle.sphere") {
      geom_type = "sphere";
      size_default = "0.1";
      size_count = 1;
    } else if (type == "obstacle.cylinder") {
      geom_type = "cylinder";
      size_default = "0.1 0.1";
      size_count = 2;
    } else if (type == "obstacle.capsule") {
      geom_type = "capsule";
      size_default = "0.1 0.1";
      size_count = 2;
    } else if (type == "obstacle.ellipsoid") {
      geom_type = "ellipsoid";
      size_default = "0.1 0.1 0.1";
      size_count = 3;
    } else if (type == "obstacle.mesh") {
      geom_type = "mesh";
      is_mesh = true;
    } else if (type == "obstacle.hfield") {
      geom_type = "hfield";
      is_hfield = true;
    } else {
      throw std::invalid_argument("unsupported scene object type: " + type);
    }

    const std::string pos = numeric_list(object.value("pos", nlohmann::json::array()), "0 0 0", 3);
    const std::string quat
        = numeric_list(object.value("quat", nlohmann::json::array()), "1 0 0 0", 4);
    const double mass = object.value("mass", 0.0);
    const std::string rgba
        = numeric_list(object.value("rgba", nlohmann::json::array()), "0.7 0.2 0.2 1", 4);

    std::ostringstream xml;
    xml << "    <body name=\"" << xml_escape(id) << "\" pos=\"" << pos << "\" quat=\"" << quat
        << "\">\n";
    xml << "      <geom name=\"" << xml_escape(id + "_geom") << "\" type=\"" << geom_type << "\"";
    if (is_mesh) {
      // Like a bodies[].geoms[] mesh geom, extent comes from the referenced
      // <mesh> asset itself -- no `size` attribute.
      const std::string mesh_ref = object.value("mesh", "");
      if (mesh_ref.empty())
        throw std::invalid_argument("obstacle.mesh '" + id + "' missing 'mesh'");
      xml << " mesh=\"" << xml_escape(mesh_ref) << "\"";
    } else if (is_hfield) {
      const std::string hfield_ref = object.value("hfield", "");
      if (hfield_ref.empty())
        throw std::invalid_argument("obstacle.hfield '" + id + "' missing 'hfield'");
      xml << " hfield=\"" << xml_escape(hfield_ref) << "\"";
    } else {
      xml << " size=\""
          << numeric_list(object.value("size", nlohmann::json::array()), size_default, size_count)
          << "\"";
    }
    xml << " rgba=\"" << rgba << "\"";
    if (mass > 0.0) xml << " mass=\"" << mass << "\"";
    xml << "/>\n";
    xml << "    </body>\n";
    return xml.str();
  }

  std::string compile_sensor_object(const nlohmann::json& sensor, const nlohmann::json& scene) {
    if (!sensor.is_object()) throw std::invalid_argument("sensor object must be an object");
    const std::string id = sensor.value("id", "");
    const std::string type = sensor.value("type", "");
    if (id.empty()) throw std::invalid_argument("sensor missing 'id'");

    const auto it = sensor_registry().find(type);
    if (it == sensor_registry().end())
      throw std::invalid_argument("unsupported sensor type: " + type);
    const auto& spec = it->second;

    std::ostringstream xml;
    xml << "    <" << spec.mjcf_tag << " name=\"" << xml_escape(id) << "\"";

    switch (spec.shape) {
      case SensorShape::kSimple: {
        const std::string ref_value = sensor.value(spec.ref_kind, "");
        if (ref_value.empty())
          throw std::invalid_argument("sensor missing '" + std::string(spec.ref_kind) + "'");
        xml << " " << spec.ref_kind << "=\"" << xml_escape(ref_value) << "\"";
        break;
      }
      case SensorShape::kFrame: {
        const std::string objtype = sensor.value("objtype", "");
        const std::string objname = sensor.value("objname", "");
        if (objtype.empty() || objname.empty())
          throw std::invalid_argument("frame sensor requires 'objtype' and 'objname'");
        xml << " objtype=\"" << xml_escape(objtype) << "\" objname=\"" << xml_escape(objname)
            << "\"";
        if (spec.frame_ref_optional && !sensor.value("reftype", "").empty()) {
          xml << " reftype=\"" << xml_escape(sensor.value("reftype", "")) << "\" refname=\""
              << xml_escape(sensor.value("refname", "")) << "\"";
        }
        break;
      }
      case SensorShape::kCamProjection: {
        const std::string site = sensor.value("site", "");
        const std::string camera = sensor.value("camera", "");
        if (site.empty() || camera.empty())
          throw std::invalid_argument("camprojection sensor requires 'site' and 'camera'");
        xml << " site=\"" << xml_escape(site) << "\" camera=\"" << xml_escape(camera) << "\"";
        break;
      }
      case SensorShape::kInsideSite: {
        const std::string site = sensor.value("site", "");
        const std::string objtype = sensor.value("objtype", "");
        const std::string objname = sensor.value("objname", "");
        if (site.empty() || objtype.empty() || objname.empty())
          throw std::invalid_argument("insidesite sensor requires 'site', 'objtype' and 'objname'");
        xml << " site=\"" << xml_escape(site) << "\" objtype=\"" << xml_escape(objtype)
            << "\" objname=\"" << xml_escape(objname) << "\"";
        break;
      }
      case SensorShape::kGeomPair: {
        const std::string geom1 = sensor.value("geom1", "");
        const std::string body1 = sensor.value("body1", "");
        const std::string geom2 = sensor.value("geom2", "");
        const std::string body2 = sensor.value("body2", "");
        if (geom1.empty() == body1.empty())
          throw std::invalid_argument("sensor requires exactly one of geom1/body1");
        if (geom2.empty() == body2.empty())
          throw std::invalid_argument("sensor requires exactly one of geom2/body2");
        if (!geom1.empty())
          xml << " geom1=\"" << xml_escape(geom1) << "\"";
        else
          xml << " body1=\"" << xml_escape(body1) << "\"";
        if (!geom2.empty())
          xml << " geom2=\"" << xml_escape(geom2) << "\"";
        else
          xml << " body2=\"" << xml_escape(body2) << "\"";
        break;
      }
      case SensorShape::kNone:
        break;
      case SensorShape::kPlugin: {
        const std::string instance_id = sensor.value("plugin_instance", "");
        if (instance_id.empty())
          throw std::invalid_argument("sensor.plugin requires 'plugin_instance'");
        xml << plugin_ref_attrs(scene, instance_id);
        const std::string objtype = sensor.value("objtype", "");
        const std::string objname = sensor.value("objname", "");
        if (!objtype.empty() && !objname.empty())
          xml << " objtype=\"" << xml_escape(objtype) << "\" objname=\"" << xml_escape(objname)
              << "\"";
        break;
      }
    }
    xml << "/>\n";
    return xml.str();
  }

  // ---------------- worldbody: nested body/joint/geom/site/camera tree ----------------

  std::string compile_joint(const nlohmann::json& joint) {
    const std::string id = joint.value("id", "");
    if (id.empty()) throw std::invalid_argument("joint missing 'id'");
    const std::string type = joint.value("type", "hinge");
    static const std::set<std::string> kTypes = {"hinge", "slide", "ball", "free"};
    if (!kTypes.count(type)) throw std::invalid_argument("unsupported joint type: " + type);

    std::ostringstream xml;
    xml << "      <joint name=\"" << xml_escape(id) << "\" type=\"" << type << "\"";
    if ((type == "hinge" || type == "slide") && joint.contains("axis")) {
      // Only emitted when the instance sets it explicitly (unlike a hardcoded
      // fallback, which would always "win" over a class default's axis and
      // defeat class inheritance for this attribute -- MuJoCo's own built-in
      // default already is "0 0 1" when neither instance nor class supplies one).
      xml << " axis=\"" << numeric_list(joint["axis"], "0 0 1", 3) << "\"";
    }
    if (type == "hinge" || type == "slide") {
      if (joint.contains("range"))
        xml << " range=\"" << numeric_list(joint["range"], "", 2) << "\" limited=\"true\"";
    }
    if (joint.contains("pos")) xml << " pos=\"" << numeric_list(joint["pos"], "0 0 0", 3) << "\"";
    if (joint.contains("damping")) xml << " damping=\"" << joint["damping"].get<double>() << "\"";
    if (joint.contains("stiffness"))
      xml << " stiffness=\"" << joint["stiffness"].get<double>() << "\"";
    xml << class_attr(joint) << "/>\n";
    return xml.str();
  }

  // `ancestor_class_available`: true when some enclosing <body>'s `childclass`
  // (its own or an ancestor's, since MJCF resolves childclass through the
  // literal <body> nesting) applies to this geom even if the geom itself names
  // no `class`. Combined with the geom's own `class` field, this decides
  // whether the geom can resolve `type`/`size` some other way than our own
  // forced fallback. When available, an instance that omits `type`/`size` gets
  // neither attribute emitted at all, letting MuJoCo resolve them from the
  // class chain -- same reasoning as the rgba/axis fix: a forced fallback here
  // would always "win" over a class default and silently defeat the entire
  // point of class inheritance. A geom with no class in scope keeps the
  // historical forced "box"+matching-size behavior so pre-existing scenes that
  // never used classes are unaffected.
  std::string compile_geom(const nlohmann::json& geom, bool ancestor_class_available,
                           const nlohmann::json& scene) {
    const std::string id = geom.value("id", "");
    if (id.empty()) throw std::invalid_argument("geom missing 'id'");
    const bool has_type = geom.contains("type") && geom["type"].is_string();
    const bool class_available = ancestor_class_available || !geom.value("class", "").empty();
    if (!has_type && !class_available) {
      // Unclassed geom with no explicit type: nothing will ever supply one, so
      // keep the old forced default instead of leaving MuJoCo to fall back to
      // its own built-in type (sphere) with a degenerate zero size. Copy (not
      // rebuild) the geom so every other field -- present or absent -- passes
      // through untouched; only `type` is forced.
      nlohmann::json forced = geom;
      forced["type"] = "box";
      return compile_geom(forced, class_available, scene);
    }
    const std::string type = geom.value("type", "box");

    std::string size_default;
    int size_count = 0;
    const bool is_mesh = has_type && type == "mesh";
    const bool is_hfield = has_type && type == "hfield";
    const bool is_sdf = has_type && type == "sdf";
    if (has_type) {
      if (type == "box" || type == "ellipsoid") {
        size_default = "0.1 0.1 0.1";
        size_count = 3;
      } else if (type == "sphere") {
        size_default = "0.1";
        size_count = 1;
      } else if (type == "cylinder" || type == "capsule") {
        size_default = "0.1 0.1";
        size_count = 2;
      } else if (type == "plane") {
        // half-width X, half-width Y, rendering grid spacing (last one is
        // render-only, not physically meaningful).
        size_default = "1 1 0.1";
        size_count = 3;
      } else if (!is_mesh && !is_hfield && !is_sdf) {
        throw std::invalid_argument("unsupported geom type: " + type);
      }
    }
    // sdf geoms carry their shape via the referenced plugin instance, not a
    // size default/count -- validated below via `plugin_instance` instead.
    const std::string plugin_instance_id = geom.value("plugin_instance", "");
    if (is_sdf && plugin_instance_id.empty())
      throw std::invalid_argument("sdf geom '" + id + "' missing 'plugin_instance'");

    std::ostringstream xml;
    xml << "      <geom name=\"" << xml_escape(id) << "\"";
    if (has_type) xml << " type=\"" << type << "\"";
    if (is_mesh) {
      // MJCF derives a mesh geom's extent from the referenced <mesh> asset
      // itself (scaled via that asset's own `scale`), so no `size` attribute.
      const std::string mesh_ref = geom.value("mesh", "");
      if (mesh_ref.empty()) throw std::invalid_argument("mesh geom '" + id + "' missing 'mesh'");
      xml << " mesh=\"" << xml_escape(mesh_ref) << "\"";
    } else if (is_hfield) {
      // Like mesh, a hfield geom's extent comes from the referenced <hfield>
      // asset's own `size` (declared once in assets[]), so no `size` attribute here.
      const std::string hfield_ref = geom.value("hfield", "");
      if (hfield_ref.empty())
        throw std::invalid_argument("hfield geom '" + id + "' missing 'hfield'");
      xml << " hfield=\"" << xml_escape(hfield_ref) << "\"";
    } else if (is_sdf) {
      // An sdf geom's shape comes from a procedural <mesh> asset (declared
      // with `plugin_instance`, see compile_asset_declaration) referenced here
      // by name -- MuJoCo's own sdf examples (model/plugin/sdf/torus.xml) both
      // point the mesh asset *and* the geom itself at the same plugin instance.
      const std::string mesh_ref = geom.value("mesh", "");
      if (mesh_ref.empty())
        throw std::invalid_argument("sdf geom '" + id
                                    + "' missing 'mesh' (a procedural mesh asset)");
      xml << " mesh=\"" << xml_escape(mesh_ref) << "\"";
      if (geom.contains("size")) xml << " size=\"" << numeric_list(geom["size"], "", 0) << "\"";
    } else if (has_type) {
      xml << " size=\""
          << numeric_list(geom.value("size", nlohmann::json::array()), size_default, size_count)
          << "\"";
    } else if (geom.contains("size")) {
      // Type is left to the class chain to resolve, so we don't know the
      // required element count here either -- pass whatever was given straight
      // through rather than enforcing a count for a type we can't see.
      xml << " size=\"" << numeric_list(geom["size"], "", 0) << "\"";
    }
    if (geom.contains("pos")) xml << " pos=\"" << numeric_list(geom["pos"], "0 0 0", 3) << "\"";
    if (geom.contains("quat"))
      xml << " quat=\"" << numeric_list(geom["quat"], "1 0 0 0", 4) << "\"";
    const std::string material_ref = geom.value("material", "");
    if (!material_ref.empty()) {
      // A material supplies its own color; only override it if the caller set
      // an explicit rgba too (MuJoCo lets rgba win over the material's rgba).
      xml << " material=\"" << xml_escape(material_ref) << "\"";
      if (geom.contains("rgba"))
        xml << " rgba=\"" << numeric_list(geom["rgba"], "1 1 1 1", 4) << "\"";
    } else if (geom.contains("rgba")) {
      // Only emitted when set explicitly -- forcing a fallback here (as this
      // used to) would always "win" over a class's geom.rgba default and
      // silently defeat class inheritance for color, which is the single most
      // common thing default classes get used for. Absent both an instance
      // rgba and a class default, MuJoCo's own built-in default (a mid grey)
      // applies, same as any other omitted geom attribute.
      xml << " rgba=\"" << numeric_list(geom["rgba"], "0.7 0.2 0.2 1", 4) << "\"";
    }
    const double mass = geom.value("mass", 0.0);
    if (mass > 0.0) xml << " mass=\"" << mass << "\"";
    if (geom.contains("friction") && geom["friction"].is_number())
      xml << " friction=\"" << geom["friction"].get<double>() << " 0.005 0.0001\"";
    if (geom.contains("contype")) xml << " contype=\"" << geom["contype"].get<int>() << "\"";
    if (geom.contains("conaffinity"))
      xml << " conaffinity=\"" << geom["conaffinity"].get<int>() << "\"";
    if (geom.contains("condim")) xml << " condim=\"" << geom["condim"].get<int>() << "\"";
    xml << class_attr(geom);
    if (plugin_instance_id.empty()) {
      xml << "/>\n";
    } else {
      // <plugin> is a child element of <geom>, not an attribute (confirmed
      // against xml_native_reader.cc:1846), so a plugin-backed geom can't
      // self-close.
      xml << ">\n        <plugin" << plugin_ref_attrs(scene, plugin_instance_id) << "/>\n"
          << "      </geom>\n";
    }
    return xml.str();
  }

  // MJCF <site> only accepts this fixed shape set (confirmed against
  // xml_native_reader.cc's site attribute table); shared by compile_site,
  // default_site_attrs (defaultClasses[].site.type) and validate_scene so a
  // bogus/typo'd type is caught with a clear error instead of either being
  // silently written unescaped into an XML attribute or failing much later
  // inside mj_loadXML with an opaque message.
  const std::set<std::string>& site_shape_types() {
    static const std::set<std::string> kTypes
        = {"sphere", "box", "capsule", "cylinder", "ellipsoid"};
    return kTypes;
  }

  std::string compile_site(const nlohmann::json& site) {
    const std::string id = site.value("id", "");
    if (id.empty()) throw std::invalid_argument("site missing 'id'");
    const std::string type = site.value("type", "sphere");
    if (!site_shape_types().count(type))
      throw std::invalid_argument("unsupported site type: " + type);

    std::ostringstream xml;
    xml << "      <site name=\"" << xml_escape(id) << "\" type=\"" << xml_escape(type) << "\"";
    if (site.contains("pos")) xml << " pos=\"" << numeric_list(site["pos"], "0 0 0", 3) << "\"";
    if (site.contains("quat"))
      xml << " quat=\"" << numeric_list(site["quat"], "1 0 0 0", 4) << "\"";
    if (site.contains("size")) xml << " size=\"" << numeric_list(site["size"], "0.01", 0) << "\"";
    xml << class_attr(site) << "/>\n";
    return xml.str();
  }

  std::string compile_camera(const nlohmann::json& camera) {
    const std::string id = camera.value("id", "");
    if (id.empty()) throw std::invalid_argument("camera missing 'id'");

    std::ostringstream xml;
    xml << "      <camera name=\"" << xml_escape(id) << "\"";
    if (camera.contains("pos")) xml << " pos=\"" << numeric_list(camera["pos"], "0 0 0", 3) << "\"";
    if (camera.contains("quat"))
      xml << " quat=\"" << numeric_list(camera["quat"], "1 0 0 0", 4) << "\"";
    if (camera.contains("fovy")) xml << " fovy=\"" << camera["fovy"].get<double>() << "\"";
    xml << class_attr(camera) << "/>\n";
    return xml.str();
  }

  // bodies[] is a flat array with an optional `parent` id, not literal JSON
  // nesting -- build a parent->children index once (root bodies keyed by ""),
  // then recurse depth-first from the roots. Cycles/unknown parents are
  // rejected in validate_scene before this ever runs.
  // `ancestor_childclass_available`: whether some enclosing <body> (not this
  // one) already has a `childclass` in scope; combined with this body's own
  // `childclass`, threaded down to compile_geom so it knows whether omitting
  // `type`/`size` is safe to leave for class resolution.
  std::string compile_body_node(
      const nlohmann::json& body,
      const std::unordered_map<std::string, std::vector<const nlohmann::json*>>& children_of,
      bool ancestor_childclass_available, const nlohmann::json& scene) {
    const std::string id = body.value("id", "");
    const bool childclass_available
        = ancestor_childclass_available || !body.value("childclass", "").empty();
    std::ostringstream xml;
    xml << "    <body name=\"" << xml_escape(id) << "\" pos=\""
        << numeric_list(body.value("pos", nlohmann::json::array()), "0 0 0", 3) << "\" quat=\""
        << numeric_list(body.value("quat", nlohmann::json::array()), "1 0 0 0", 4) << "\"";
    if (!body.value("childclass", "").empty())
      xml << " childclass=\"" << xml_escape(body.value("childclass", "")) << "\"";
    xml << ">\n";
    for (const auto& joint : body.value("joints", nlohmann::json::array()))
      xml << compile_joint(joint);
    for (const auto& geom : body.value("geoms", nlohmann::json::array()))
      xml << compile_geom(geom, childclass_available, scene);
    for (const auto& site : body.value("sites", nlohmann::json::array())) xml << compile_site(site);
    for (const auto& camera : body.value("cameras", nlohmann::json::array()))
      xml << compile_camera(camera);
    if (const auto it = children_of.find(id); it != children_of.end()) {
      for (const auto* child : it->second)
        xml << compile_body_node(*child, children_of, childclass_available, scene);
    }
    xml << "    </body>\n";
    return xml.str();
  }

  // Wraps a root body's compiled XML in <replicate>, per its optional
  // `replicate: {count, offset, euler?, separate?}` (validate_scene restricts
  // this field to root bodies and forbids equality/contacts/sensors from
  // referencing anything inside the replicated subtree, since MuJoCo renames
  // each copy with a generated suffix that the scene JSON's flat id model has
  // no way to predict or address).
  std::string wrap_replicate(const nlohmann::json& body, std::string inner) {
    if (!body.contains("replicate")) return inner;
    const auto& rep = body["replicate"];
    std::ostringstream xml;
    xml << "    <replicate count=\"" << rep.value("count", 1) << "\" offset=\""
        << numeric_list(rep.value("offset", nlohmann::json::array()), "0 0 0", 3) << "\"";
    if (rep.contains("euler")) xml << " euler=\"" << numeric_list(rep["euler"], "0 0 0", 3) << "\"";
    if (rep.contains("separate"))
      xml << " separate=\"" << (rep.value("separate", false) ? "true" : "false") << "\"";
    xml << ">\n" << inner << "    </replicate>\n";
    return xml.str();
  }

  std::string compile_body_forest(const nlohmann::json& bodies, const nlohmann::json& scene) {
    if (!bodies.is_array() || bodies.empty()) return {};
    std::unordered_map<std::string, std::vector<const nlohmann::json*>> children_of;
    for (const auto& body : bodies) {
      if (!body.is_object() || body.value("id", "").empty()) continue;
      children_of[body.value("parent", "")].push_back(&body);
    }
    std::ostringstream xml;
    if (const auto it = children_of.find(std::string{}); it != children_of.end()) {
      for (const auto* root : it->second)
        xml << wrap_replicate(*root, compile_body_node(*root, children_of, false, scene));
    }
    return xml.str();
  }

  // ---------------- default/class inheritance ----------------
  // Mirrors mjXWriter::Default (xml_native_writer.cc): a <default class="id">
  // block whose children set default attribute values for that class, applied
  // to any instance carrying a matching `class="id"` attribute (or inheriting
  // it via a body's `childclass`). MuJoCo's own writer always uses <general>
  // for actuator defaults regardless of which shortcut tag (motor/position/...)
  // an instance uses, since motor/position/velocity are just presets of the
  // same underlying attribute set -- so scene JSON's `actuator` sub-object on
  // a class is emitted as <general> too. mesh/material/light/pair/equality/
  // tendon defaults exist in MJCF but aren't exposed here; joint/geom/site/
  // camera/actuator cover the attributes this schema's instances actually use.

  // Shared with validate_scene's defaultClasses[] check so a bad/typo'd type in
  // a class default is rejected up front instead of either landing unescaped in
  // an XML attribute or failing later inside mj_loadXML with an opaque message
  // (same reasoning as site_shape_types() above).
  const std::set<std::string>& joint_type_names() {
    static const std::set<std::string> kTypes = {"hinge", "slide", "ball", "free"};
    return kTypes;
  }
  const std::set<std::string>& geom_type_names() {
    static const std::set<std::string> kTypes
        = {"box", "sphere", "cylinder", "capsule", "ellipsoid", "plane", "mesh", "hfield", "sdf"};
    return kTypes;
  }

  std::string default_joint_attrs(const nlohmann::json& joint) {
    std::ostringstream xml;
    if (joint.contains("type")) {
      const auto type = joint["type"].get<std::string>();
      if (!joint_type_names().count(type))
        throw std::invalid_argument("unsupported joint type: " + type);
      xml << " type=\"" << xml_escape(type) << "\"";
    }
    if (joint.contains("axis")) xml << " axis=\"" << numeric_list(joint["axis"], "", 3) << "\"";
    if (joint.contains("range"))
      xml << " range=\"" << numeric_list(joint["range"], "", 2) << "\" limited=\"true\"";
    if (joint.contains("damping")) xml << " damping=\"" << joint["damping"].get<double>() << "\"";
    if (joint.contains("stiffness"))
      xml << " stiffness=\"" << joint["stiffness"].get<double>() << "\"";
    return xml.str();
  }

  std::string default_geom_attrs(const nlohmann::json& geom) {
    std::ostringstream xml;
    if (geom.contains("type")) {
      const auto type = geom["type"].get<std::string>();
      if (!geom_type_names().count(type))
        throw std::invalid_argument("unsupported geom type: " + type);
      xml << " type=\"" << xml_escape(type) << "\"";
    }
    if (geom.contains("size")) xml << " size=\"" << numeric_list(geom["size"], "", 0) << "\"";
    if (geom.contains("rgba")) xml << " rgba=\"" << numeric_list(geom["rgba"], "", 4) << "\"";
    if (!geom.value("material", "").empty())
      xml << " material=\"" << xml_escape(geom.value("material", "")) << "\"";
    if (geom.contains("mass")) xml << " mass=\"" << geom["mass"].get<double>() << "\"";
    if (geom.contains("friction") && geom["friction"].is_number())
      xml << " friction=\"" << geom["friction"].get<double>() << " 0.005 0.0001\"";
    if (geom.contains("contype")) xml << " contype=\"" << geom["contype"].get<int>() << "\"";
    if (geom.contains("conaffinity"))
      xml << " conaffinity=\"" << geom["conaffinity"].get<int>() << "\"";
    if (geom.contains("condim")) xml << " condim=\"" << geom["condim"].get<int>() << "\"";
    return xml.str();
  }

  std::string default_site_attrs(const nlohmann::json& site) {
    std::ostringstream xml;
    if (site.contains("type")) {
      const auto type = site["type"].get<std::string>();
      if (!site_shape_types().count(type))
        throw std::invalid_argument("unsupported site type: " + type);
      xml << " type=\"" << xml_escape(type) << "\"";
    }
    if (site.contains("size")) xml << " size=\"" << numeric_list(site["size"], "", 0) << "\"";
    return xml.str();
  }

  std::string default_camera_attrs(const nlohmann::json& camera) {
    std::ostringstream xml;
    if (camera.contains("fovy")) xml << " fovy=\"" << camera["fovy"].get<double>() << "\"";
    return xml.str();
  }

  std::string default_actuator_attrs(const nlohmann::json& actuator) {
    std::ostringstream xml;
    if (actuator.contains("gear"))
      xml << " gear=\"" << numeric_list(actuator["gear"], "1", 0) << "\"";
    if (actuator.contains("ctrlrange"))
      xml << " ctrlrange=\"" << numeric_list(actuator["ctrlrange"], "", 2) << "\"";
    if (actuator.contains("forcerange"))
      xml << " forcerange=\"" << numeric_list(actuator["forcerange"], "", 2) << "\"";
    if (actuator.contains("kp")) xml << " kp=\"" << actuator["kp"].get<double>() << "\"";
    if (actuator.contains("kv")) xml << " kv=\"" << actuator["kv"].get<double>() << "\"";
    return xml.str();
  }

  // defaultClasses[] is a flat array with an optional `parent` id, exactly
  // like bodies[] -- MJCF requires child <default> blocks to be literally
  // nested inside their parent's, so this recurses the same way
  // compile_body_forest does.
  std::string compile_default_node(
      const nlohmann::json& cls,
      const std::unordered_map<std::string, std::vector<const nlohmann::json*>>& children_of) {
    const std::string id = cls.value("id", "");
    std::ostringstream xml;
    xml << "    <default class=\"" << xml_escape(id) << "\">\n";
    if (cls.contains("joint")) {
      const auto attrs = default_joint_attrs(cls["joint"]);
      if (!attrs.empty()) xml << "      <joint" << attrs << "/>\n";
    }
    if (cls.contains("geom")) {
      const auto attrs = default_geom_attrs(cls["geom"]);
      if (!attrs.empty()) xml << "      <geom" << attrs << "/>\n";
    }
    if (cls.contains("site")) {
      const auto attrs = default_site_attrs(cls["site"]);
      if (!attrs.empty()) xml << "      <site" << attrs << "/>\n";
    }
    if (cls.contains("camera")) {
      const auto attrs = default_camera_attrs(cls["camera"]);
      if (!attrs.empty()) xml << "      <camera" << attrs << "/>\n";
    }
    if (cls.contains("actuator")) {
      const auto attrs = default_actuator_attrs(cls["actuator"]);
      if (!attrs.empty()) xml << "      <general" << attrs << "/>\n";
    }
    if (const auto it = children_of.find(id); it != children_of.end()) {
      for (const auto* child : it->second) xml << compile_default_node(*child, children_of);
    }
    xml << "    </default>\n";
    return xml.str();
  }

  std::string generate_default_xml(const nlohmann::json& scene) {
    const auto classes = scene.value("defaultClasses", nlohmann::json::array());
    if (!classes.is_array() || classes.empty()) return {};
    std::unordered_map<std::string, std::vector<const nlohmann::json*>> children_of;
    for (const auto& cls : classes) {
      if (!cls.is_object() || cls.value("id", "").empty()) continue;
      children_of[cls.value("parent", "")].push_back(&cls);
    }
    std::ostringstream xml;
    // Root classes (no parent) each become their own top-level <default>;
    // MJCF's single implicit "main" class isn't required here since every
    // instance names its class explicitly via `class=`/`childclass=`.
    if (const auto it = children_of.find(std::string{}); it != children_of.end()) {
      for (const auto* root : it->second) xml << compile_default_node(*root, children_of);
    }
    return xml.str();
  }

  // ---------------- equality / tendon / actuator / keyframe ----------------

  // type -> MJCF transmission target attribute name. Every shortcut tag except
  // `adhesion` targets a joint; `adhesion` targets a body instead (confirmed
  // against xml_native_reader.cc:391-436's per-tag attribute tables). Only
  // joint/body targets are supported here (not tendon/site/slidercrank/
  // jointinparent transmissions), matching how this schema already treats
  // joints as the one thing actuators drive.
  const std::unordered_map<std::string, std::string>& actuator_registry() {
    static const std::unordered_map<std::string, std::string> kRegistry
        = {{"motor", "joint"},
           {"position", "joint"},
           {"velocity", "joint"},
           {"general", "joint"},
           {"intvelocity", "joint"},
           {"damper", "joint"},
           {"cylinder", "joint"},
           {"muscle", "joint"},
           {"adhesion", "body"},
           // Plugin-backed actuator (e.g. mujoco.pid): like the other shortcuts
           // this schema only wires up a joint transmission, the common case for
           // first-party actuator plugins.
           {"plugin", "joint"}};
    return kRegistry;
  }

  std::string compile_actuator(const nlohmann::json& actuator, const nlohmann::json& scene) {
    const std::string id = actuator.value("id", "");
    if (id.empty()) throw std::invalid_argument("actuator missing 'id'");
    const std::string type = actuator.value("type", "");
    const auto& registry = actuator_registry();
    const auto spec = registry.find(type);
    if (spec == registry.end()) throw std::invalid_argument("unsupported actuator type: " + type);
    const std::string& target_key = spec->second;  // "joint" or "body"
    const std::string target = actuator.value(target_key, "");
    if (target.empty()) throw std::invalid_argument("actuator missing '" + target_key + "'");

    std::ostringstream xml;
    xml << "    <" << type << " name=\"" << xml_escape(id) << "\" " << target_key << "=\""
        << xml_escape(target) << "\"";
    if (target_key == "joint" && actuator.contains("gear"))
      xml << " gear=\"" << numeric_list(actuator["gear"], "1", 0) << "\"";
    if (actuator.contains("ctrlrange"))
      xml << " ctrlrange=\"" << numeric_list(actuator["ctrlrange"], "", 2) << "\"";
    if (actuator.contains("forcerange"))
      xml << " forcerange=\"" << numeric_list(actuator["forcerange"], "", 2) << "\"";

    // Per-type extra attributes (confirmed against xml_native_reader.cc:391-436
    // and the mjs_setTo*() call sites around line 2360-2430 for each shortcut's
    // exact parameter counts: muscle.timeconst is 2 numbers, cylinder.timeconst
    // is 1, cylinder.bias is 3).
    if (type == "position") {
      emit_num_attr(xml, actuator, "kp");
      emit_num_attr(xml, actuator, "kv");
      emit_num_attr(xml, actuator, "dampratio");
      emit_num_attr(xml, actuator, "timeconst");
    } else if (type == "velocity") {
      emit_num_attr(xml, actuator, "kv");
    } else if (type == "intvelocity") {
      emit_num_attr(xml, actuator, "kp");
      emit_num_attr(xml, actuator, "kv");
      emit_num_attr(xml, actuator, "dampratio");
      emit_vec_attr(xml, actuator, "actrange", 2);
    } else if (type == "damper") {
      emit_num_attr(xml, actuator, "kv");
    } else if (type == "cylinder") {
      emit_num_attr(xml, actuator, "timeconst");
      emit_num_attr(xml, actuator, "area");
      emit_num_attr(xml, actuator, "diameter");
      emit_vec_attr(xml, actuator, "bias", 3);
    } else if (type == "muscle") {
      emit_vec_attr(xml, actuator, "timeconst", 2);
      emit_vec_attr(xml, actuator, "range", 2);
      emit_num_attr(xml, actuator, "force");
      emit_num_attr(xml, actuator, "scale");
      emit_num_attr(xml, actuator, "lmin");
      emit_num_attr(xml, actuator, "lmax");
      emit_num_attr(xml, actuator, "vmax");
      emit_num_attr(xml, actuator, "fpmax");
      emit_num_attr(xml, actuator, "fvmax");
    } else if (type == "adhesion") {
      emit_num_attr(xml, actuator, "gain");
    } else if (type == "general") {
      // Full low-level parametrization -- lets a scene bypass the shortcut
      // tags entirely and hand-specify MuJoCo's underlying actuator model.
      emit_str_attr(xml, actuator, "dyntype");
      emit_str_attr(xml, actuator, "gaintype");
      emit_str_attr(xml, actuator, "biastype");
      emit_vec_attr(xml, actuator, "dynprm", 0);
      emit_vec_attr(xml, actuator, "gainprm", 0);
      emit_vec_attr(xml, actuator, "biasprm", 0);
    } else if (type == "plugin") {
      // The MJCF tag itself is <plugin> here (type == the tag name, same as
      // every other shortcut above), carrying plugin/instance attributes
      // alongside the joint= transmission already written above.
      const std::string instance_id = actuator.value("plugin_instance", "");
      if (instance_id.empty())
        throw std::invalid_argument("actuator.plugin requires 'plugin_instance'");
      xml << plugin_ref_attrs(scene, instance_id);
    }
    xml << class_attr(actuator) << "/>\n";
    return xml.str();
  }

  std::string compile_equality(const nlohmann::json& eq) {
    const std::string id = eq.value("id", "");
    if (id.empty()) throw std::invalid_argument("equality constraint missing 'id'");
    const std::string type = eq.value("type", "");

    // MJCF's connect/weld/joint/tendon equality elements all accept an `active`
    // attribute (initial enable/disable state, mirrored at runtime by
    // mjData::eq_active -- see xml_native_reader.cc's equality schema table).
    // Only emitted when explicitly set so an omitted value keeps MuJoCo's own
    // default (active=true).
    std::string active_attr;
    if (eq.contains("active") && eq["active"].is_boolean())
      active_attr = eq["active"].get<bool>() ? " active=\"true\"" : " active=\"false\"";

    std::ostringstream xml;
    if (type == "connect") {
      const std::string body1 = eq.value("body1", "");
      const std::string body2 = eq.value("body2", "");
      if (body1.empty() || body2.empty())
        throw std::invalid_argument("equality.connect requires body1 and body2");
      xml << "    <connect name=\"" << xml_escape(id) << "\" body1=\"" << xml_escape(body1)
          << "\" body2=\"" << xml_escape(body2) << "\" anchor=\""
          << numeric_list(eq.value("anchor", nlohmann::json::array()), "0 0 0", 3) << "\""
          << active_attr << "/>\n";
    } else if (type == "weld") {
      const std::string body1 = eq.value("body1", "");
      const std::string body2 = eq.value("body2", "");
      if (body1.empty() || body2.empty())
        throw std::invalid_argument("equality.weld requires body1 and body2");
      xml << "    <weld name=\"" << xml_escape(id) << "\" body1=\"" << xml_escape(body1)
          << "\" body2=\"" << xml_escape(body2) << "\"" << active_attr << "/>\n";
    } else if (type == "joint") {
      const std::string joint1 = eq.value("joint1", "");
      const std::string joint2 = eq.value("joint2", "");
      if (joint1.empty()) throw std::invalid_argument("equality.joint requires joint1");
      xml << "    <joint name=\"" << xml_escape(id) << "\" joint1=\"" << xml_escape(joint1) << "\"";
      if (!joint2.empty()) xml << " joint2=\"" << xml_escape(joint2) << "\"";
      xml << active_attr << "/>\n";
    } else if (type == "tendon") {
      const std::string tendon1 = eq.value("tendon1", "");
      const std::string tendon2 = eq.value("tendon2", "");
      if (tendon1.empty()) throw std::invalid_argument("equality.tendon requires tendon1");
      xml << "    <tendon name=\"" << xml_escape(id) << "\" tendon1=\"" << xml_escape(tendon1)
          << "\"";
      if (!tendon2.empty()) xml << " tendon2=\"" << xml_escape(tendon2) << "\"";
      if (eq.contains("polycoef"))
        xml << " polycoef=\"" << numeric_list(eq["polycoef"], "", 5) << "\"";
      xml << active_attr << "/>\n";
    } else {
      throw std::invalid_argument("unsupported equality type: " + type);
    }
    return xml.str();
  }

  std::string compile_tendon(const nlohmann::json& tendon) {
    const std::string id = tendon.value("id", "");
    if (id.empty()) throw std::invalid_argument("tendon missing 'id'");
    const std::string type = tendon.value("type", "");

    std::ostringstream attrs;
    if (tendon.contains("range"))
      attrs << " range=\"" << numeric_list(tendon["range"], "", 2) << "\"";
    if (tendon.contains("stiffness"))
      attrs << " stiffness=\"" << tendon["stiffness"].get<double>() << "\"";
    if (tendon.contains("damping"))
      attrs << " damping=\"" << tendon["damping"].get<double>() << "\"";

    std::ostringstream xml;
    if (type == "fixed") {
      const auto joints = tendon.value("joints", nlohmann::json::array());
      if (joints.empty()) throw std::invalid_argument("tendon.fixed requires 'joints'");
      xml << "    <fixed name=\"" << xml_escape(id) << "\"" << attrs.str() << ">\n";
      for (const auto& j : joints) {
        xml << "      <joint joint=\"" << xml_escape(j.value("joint", "")) << "\" coef=\""
            << j.value("coef", 1.0) << "\"/>\n";
      }
      xml << "    </fixed>\n";
    } else if (type == "spatial") {
      // `path[]` (ordered site/geom/pulley path elements, confirmed against
      // xml_native_reader.cc:378-382) is the general form supporting geom
      // wrap/pulley obstacles; the older `sites[]` (plain site-name array) is
      // kept as a shortcut for the common "just thread some sites" case and
      // used whenever `path` is absent, so existing simple spatial tendons
      // keep compiling unchanged.
      const bool has_path
          = tendon.contains("path") && tendon["path"].is_array() && !tendon["path"].empty();
      const auto sites = tendon.value("sites", nlohmann::json::array());
      if (!has_path && sites.size() < 2)
        throw std::invalid_argument("tendon.spatial requires at least 2 'sites' (or a 'path')");
      xml << "    <spatial name=\"" << xml_escape(id) << "\"" << attrs.str() << ">\n";
      if (has_path) {
        for (const auto& p : tendon["path"]) {
          const std::string ptype = p.value("type", "site");
          if (ptype == "site") {
            xml << "      <site site=\"" << xml_escape(p.value("site", "")) << "\"/>\n";
          } else if (ptype == "geom") {
            xml << "      <geom geom=\"" << xml_escape(p.value("geom", ""));
            if (!p.value("sidesite", "").empty())
              xml << "\" sidesite=\"" << xml_escape(p.value("sidesite", ""));
            xml << "\"/>\n";
          } else if (ptype == "pulley") {
            xml << "      <pulley divisor=\"" << p.value("divisor", 1.0) << "\"/>\n";
          } else {
            throw std::invalid_argument("unsupported tendon path element type: " + ptype);
          }
        }
      } else {
        for (const auto& s : sites) {
          xml << "      <site site=\"" << xml_escape(s.get<std::string>()) << "\"/>\n";
        }
      }
      xml << "    </spatial>\n";
    } else {
      throw std::invalid_argument("unsupported tendon type: " + type);
    }
    return xml.str();
  }

  std::string compile_keyframe(const nlohmann::json& kf) {
    const std::string id = kf.value("id", "");
    if (id.empty()) throw std::invalid_argument("keyframe missing 'id'");

    std::ostringstream xml;
    xml << "    <key name=\"" << xml_escape(id) << "\"";
    if (kf.contains("time")) xml << " time=\"" << kf["time"].get<double>() << "\"";
    if (kf.contains("qpos")) xml << " qpos=\"" << numeric_list(kf["qpos"], "", 0) << "\"";
    if (kf.contains("qvel")) xml << " qvel=\"" << numeric_list(kf["qvel"], "", 0) << "\"";
    if (kf.contains("ctrl")) xml << " ctrl=\"" << numeric_list(kf["ctrl"], "", 0) << "\"";
    xml << "/>\n";
    return xml.str();
  }

  // ---------------- scene-declared assets: mesh / texture / material / hfield ----------------
  // Referenced by id from bodies[].geoms[] (mesh=/material=) -- see compile_geom above.
  // Files are resolved the same way models[].source is (relative to the scene's
  // own file, or absolute); there's no packaging/snapshotting step here the way
  // the model registry does it for registered assets, so a moved/renamed scene
  // file's relative asset paths need to move with it, same as any model source.

  std::string compile_asset_declaration(const nlohmann::json& scene, const nlohmann::json& asset) {
    const std::string id = asset.value("id", "");
    if (id.empty()) throw std::invalid_argument("scene asset missing 'id'");
    const std::string kind = asset.value("kind", "");

    std::ostringstream xml;
    if (kind == "texture") {
      const std::string file = asset.value("file", "");
      if (file.empty()) throw std::invalid_argument("texture asset '" + id + "' missing 'file'");
      xml << "    <texture name=\"" << xml_escape(id) << "\" type=\""
          << xml_escape(asset.value("type", "2d")) << "\" file=\""
          << xml_escape(resolve_scene_path(scene, file).string()) << "\"/>\n";
    } else if (kind == "material") {
      xml << "    <material name=\"" << xml_escape(id) << "\"";
      if (!asset.value("texture", "").empty())
        xml << " texture=\"" << xml_escape(asset.value("texture", "")) << "\"";
      if (asset.contains("rgba"))
        xml << " rgba=\"" << numeric_list(asset["rgba"], "1 1 1 1", 4) << "\"";
      if (asset.contains("specular"))
        xml << " specular=\"" << asset["specular"].get<double>() << "\"";
      if (asset.contains("shininess"))
        xml << " shininess=\"" << asset["shininess"].get<double>() << "\"";
      if (asset.contains("reflectance"))
        xml << " reflectance=\"" << asset["reflectance"].get<double>() << "\"";
      if (asset.contains("emission"))
        xml << " emission=\"" << asset["emission"].get<double>() << "\"";
      xml << "/>\n";
    } else if (kind == "mesh") {
      const std::string plugin_instance_id = asset.value("plugin_instance", "");
      if (!plugin_instance_id.empty()) {
        // Procedural mesh (e.g. mujoco.sdf.torus): no file, geometry comes
        // entirely from the plugin instance. Matches the official <mesh><plugin
        // instance=".."/></mesh> shape (model/plugin/sdf/torus.xml) -- geoms
        // referencing this mesh (type="sdf") still carry their own <plugin>
        // child too; see compile_geom's is_sdf branch.
        xml << "    <mesh name=\"" << xml_escape(id) << "\">\n      <plugin instance=\""
            << xml_escape(plugin_instance_id) << "\"/>\n    </mesh>\n";
      } else {
        const std::string file = asset.value("file", "");
        if (file.empty())
          throw std::invalid_argument("mesh asset '" + id
                                       + "' missing 'file' (or 'plugin_instance' for a procedural "
                                         "mesh)");
        xml << "    <mesh name=\"" << xml_escape(id) << "\" file=\""
            << xml_escape(resolve_scene_path(scene, file).string()) << "\"";
        if (asset.contains("scale"))
          xml << " scale=\"" << numeric_list(asset["scale"], "1 1 1", 3) << "\"";
        xml << "/>\n";
      }
    } else if (kind == "hfield") {
      const std::string file = asset.value("file", "");
      if (file.empty()) throw std::invalid_argument("hfield asset '" + id + "' missing 'file'");
      xml << "    <hfield name=\"" << xml_escape(id) << "\" file=\""
          << xml_escape(resolve_scene_path(scene, file).string()) << "\" size=\""
          << numeric_list(asset.value("size", nlohmann::json::array()), "1 1 1 0.1", 4) << "\"/>\n";
    } else {
      throw std::invalid_argument("unsupported scene asset kind: " + kind);
    }
    return xml.str();
  }

  std::string generate_user_asset_xml(const nlohmann::json& scene) {
    std::ostringstream out;
    const auto assets = scene.value("assets", nlohmann::json::array());
    // Emit grouped by kind (texture before material before mesh/hfield) so a
    // material's `texture=` reference always names something already declared
    // earlier in the <asset> block, matching mjXWriter::Asset's own ordering
    // (xml_native_writer.cc): texture, material, mesh, hfield.
    for (const char* kind : {"texture", "material", "mesh", "hfield"}) {
      for (const auto& asset : assets) {
        if (asset.is_object() && asset.value("kind", "") == kind)
          out << compile_asset_declaration(scene, asset);
      }
    }
    return out.str();
  }

  // ---------------- extension: MuJoCo plugin instances (scene.plugins[]) ----------------
  std::string generate_extension_xml(const nlohmann::json& scene) {
    const auto plugins = scene.value("plugins", nlohmann::json::array());
    if (!plugins.is_array() || plugins.empty()) return {};
    // Group instances by plugin family so each family gets one <plugin
    // plugin="..."> block listing all of its instances, matching
    // mjXReader::Extension() (xml_native_reader.cc:2934-2964).
    std::vector<std::string> family_order;
    std::unordered_map<std::string, std::vector<const nlohmann::json*>> by_family;
    for (const auto& plugin : plugins) {
      if (!plugin.is_object()) continue;
      const std::string family = plugin.value("plugin", "");
      if (family.empty()) continue;
      if (!by_family.count(family)) family_order.push_back(family);
      by_family[family].push_back(&plugin);
    }
    std::ostringstream xml;
    for (const auto& family : family_order) {
      xml << "    <plugin plugin=\"" << xml_escape(family) << "\">\n";
      for (const auto* plugin : by_family[family]) {
        xml << "      <instance name=\"" << xml_escape(plugin->value("id", "")) << "\">\n";
        const auto config = plugin->value("config", nlohmann::json::object());
        if (config.is_object()) {
          for (const auto& [key, value] : config.items()) {
            xml << "        <config key=\"" << xml_escape(key) << "\" value=\""
                << xml_escape(value.is_string() ? value.get<std::string>() : value.dump())
                << "\"/>\n";
          }
        }
        xml << "      </instance>\n";
      }
      xml << "    </plugin>\n";
    }
    return xml.str();
  }

  // ---------------- per-section xml assembly ----------------
  // Each returns the section's *inner* content only (no wrapping tag); the
  // caller (SimulationCompiler::generate_scene_xml) decides whether to wrap it
  // and in what order -- see the canonical MJCF top-level order there.

  std::string generate_env_asset_xml(const nlohmann::json& environment) {
    const nlohmann::json ground = ground_config(environment);
    const bool ground_on = !ground.is_null();
    const bool ground_checker = ground_on && ground.value("checker", false);
    const double ground_reflectance = ground_on ? ground.value("reflectance", 0.0) : 0.0;
    const bool ground_material = ground_checker || ground_reflectance > 0.0;
    const double ground_size = ground_on ? std::max(0.1, ground.value("size", 5.0)) : 5.0;

    std::ostringstream env_assets;
    if (skybox_enabled(environment)) {
      const nlohmann::json skybox
          = environment["skybox"].is_object() ? environment["skybox"] : nlohmann::json::object();
      env_assets << "    <texture name=\"scene_skybox\" type=\"skybox\" builtin=\"gradient\" "
                    "rgb1=\""
                 << numeric_list(skybox.value("rgb1", nlohmann::json::array()), "0.45 0.62 0.82", 3)
                 << "\" rgb2=\""
                 << numeric_list(skybox.value("rgb2", nlohmann::json::array()), "0.9 0.94 0.99", 3)
                 << "\" width=\"512\" height=\"3072\"/>\n";
    }
    if (ground_checker) {
      env_assets << "    <texture name=\"scene_ground_tex\" type=\"2d\" builtin=\"checker\" "
                    "rgb1=\""
                 << numeric_list(ground.value("rgb1", nlohmann::json::array()), "0.2 0.25 0.3", 3)
                 << "\" rgb2=\""
                 << numeric_list(ground.value("rgb2", nlohmann::json::array()), "0.32 0.38 0.45", 3)
                 << "\" width=\"300\" height=\"300\" mark=\"edge\" markrgb=\"0.85 0.85 0.85\"/>\n";
    }
    if (ground_material) {
      env_assets << "    <material name=\"scene_ground_mat\"";
      if (ground_checker) {
        env_assets << " texture=\"scene_ground_tex\" texrepeat=\"" << ground_size << " "
                   << ground_size << "\" texuniform=\"true\"";
      } else {
        env_assets << " rgba=\""
                   << numeric_list(ground.value("rgba", nlohmann::json::array()), "0.35 0.4 0.45 1",
                                   4)
                   << "\"";
      }
      if (ground_reflectance > 0.0) env_assets << " reflectance=\"" << ground_reflectance << "\"";
      env_assets << "/>\n";
    }
    return env_assets.str();
  }

  std::string generate_asset_xml(
      const nlohmann::json& scene, const nlohmann::json& environment,
      const std::function<std::string(const nlohmann::json&, const std::filesystem::path&)>&
          model_file_ref) {
    std::ostringstream out;
    out << generate_env_asset_xml(environment);
    out << generate_user_asset_xml(scene);
    for (const auto& asset : scene.value("models", nlohmann::json::array())) {
      const std::string id = asset.value("id", "");
      const std::string asset_name = "scene_model_" + id;
      const auto source = resolve_scene_path(scene, asset.value("source", ""));
      out << "    <model name=\"" << xml_escape(asset_name) << "\" file=\""
          << xml_escape(model_file_ref(asset, source)) << "\"/>\n";
    }
    return out.str();
  }

  std::string generate_worldbody_xml(const nlohmann::json& scene,
                                     const nlohmann::json& environment) {
    std::ostringstream xml;
    const nlohmann::json ground = ground_config(environment);
    const bool ground_on = !ground.is_null();
    const bool ground_checker = ground_on && ground.value("checker", false);
    const double ground_reflectance = ground_on ? ground.value("reflectance", 0.0) : 0.0;
    const bool ground_material = ground_checker || ground_reflectance > 0.0;
    const double ground_size = ground_on ? std::max(0.1, ground.value("size", 5.0)) : 5.0;

    int light_index = 0;
    for (const auto& light : scene_lights(environment)) {
      if (!light.is_object()) continue;
      xml << "    <light name=\"scene_light_" << light_index << "\" pos=\""
          << numeric_list(light.value("pos", nlohmann::json::array()), "0 0 3", 3) << "\" dir=\""
          << numeric_list(light.value("dir", nlohmann::json::array()), "0 0 -1", 3)
          << "\" directional=\"" << (light.value("directional", true) ? "true" : "false") << "\"";
      if (light.contains("diffuse"))
        xml << " diffuse=\"" << numeric_list(light["diffuse"], "0.8 0.8 0.8", 3) << "\"";
      if (light.contains("ambient"))
        xml << " ambient=\"" << numeric_list(light["ambient"], "0 0 0", 3) << "\"";
      if (light.contains("specular"))
        xml << " specular=\"" << numeric_list(light["specular"], "0.3 0.3 0.3", 3) << "\"";
      if (light.contains("castshadow"))
        xml << " castshadow=\"" << (light.value("castshadow", true) ? "true" : "false") << "\"";
      xml << "/>\n";
      ++light_index;
    }
    if (ground_on) {
      xml << "    <geom name=\"scene_ground\" type=\"plane\" size=\"" << ground_size << " "
          << ground_size << " 0.1\"";
      if (ground_material)
        xml << " material=\"scene_ground_mat\"";
      else
        xml << " rgba=\""
            << numeric_list(ground.value("rgba", nlohmann::json::array()), "0.35 0.4 0.45 1", 4)
            << "\"";
      if (ground.contains("friction") && ground["friction"].is_number())
        xml << " friction=\"" << ground["friction"].get<double>() << " 0.005 0.0001\"";
      xml << "/>\n";
    }

    for (const auto& asset : scene.value("models", nlohmann::json::array())) {
      const std::string id = asset.value("id", "");
      const std::string asset_name = "scene_model_" + id;
      const std::string prefix = asset.value("prefix", id + "_");
      xml << "    <frame name=\"" << xml_escape(id + "_mount") << "\" pos=\""
          << numeric_list(asset.value("pos", nlohmann::json::array()), "0 0 0", 3) << "\" quat=\""
          << numeric_list(asset.value("quat", nlohmann::json::array()), "1 0 0 0", 4)
          << "\">\n      <attach model=\"" << xml_escape(asset_name) << "\"";
      if (!asset.value("body", "").empty())
        xml << " body=\"" << xml_escape(asset.value("body", "")) << "\"";
      xml << " prefix=\"" << xml_escape(prefix) << "\"/>\n    </frame>\n";
    }

    if (scene.contains("objects") && scene["objects"].is_array()) {
      for (const auto& object : scene["objects"]) {
        if (!is_sensor_type(object.value("type", ""))) xml << compile_primitive_object(object);
      }
    }

    // World-level cameras (scene.cameras[], distinct from bodies[].cameras[]):
    // fixed in world space, not attached to any moving body, so they compile
    // directly as <worldbody> children rather than nested inside a <body>.
    for (const auto& camera : scene.value("cameras", nlohmann::json::array()))
      xml << compile_camera(camera);

    xml << compile_body_forest(scene.value("bodies", nlohmann::json::array()), scene);

    return xml.str();
  }

  // Confirmed against mjXReader::OnePair (xml_native_reader.cc:2019-2048):
  // solref/solreffriction take mjNREF=2 numbers, solimp takes mjNIMP=5, friction
  // takes 5. `id` (optional) becomes the MJCF `name`.
  std::string compile_contact_pair(const nlohmann::json& pair) {
    const std::string geom1 = pair.value("geom1", "");
    const std::string geom2 = pair.value("geom2", "");
    if (geom1.empty() || geom2.empty())
      throw std::invalid_argument("contact pair requires geom1 and geom2");

    std::ostringstream xml;
    xml << "    <pair";
    if (!pair.value("id", "").empty())
      xml << " name=\"" << xml_escape(pair.value("id", "")) << "\"";
    xml << " geom1=\"" << xml_escape(geom1) << "\" geom2=\"" << xml_escape(geom2) << "\"";
    if (pair.contains("condim")) xml << " condim=\"" << pair["condim"].get<int>() << "\"";
    if (pair.contains("friction"))
      xml << " friction=\"" << numeric_list(pair["friction"], "", 5) << "\"";
    if (pair.contains("solref")) xml << " solref=\"" << numeric_list(pair["solref"], "", 2) << "\"";
    if (pair.contains("solreffriction"))
      xml << " solreffriction=\"" << numeric_list(pair["solreffriction"], "", 2) << "\"";
    if (pair.contains("solimp")) xml << " solimp=\"" << numeric_list(pair["solimp"], "", 5) << "\"";
    if (pair.contains("margin")) xml << " margin=\"" << pair["margin"].get<double>() << "\"";
    if (pair.contains("gap")) xml << " gap=\"" << pair["gap"].get<double>() << "\"";
    xml << "/>\n";
    return xml.str();
  }

  std::string generate_contact_xml(const nlohmann::json& scene) {
    std::ostringstream xml;
    const auto contacts = scene.value("contacts", nlohmann::json::object());
    for (const auto& item : contacts.value("pairs", nlohmann::json::array()))
      xml << compile_contact_pair(item);
    for (const auto& item : contacts.value("excludes", nlohmann::json::array())) {
      xml << "    <exclude";
      if (!item.value("name", "").empty())
        xml << " name=\"" << xml_escape(item.value("name", "")) << "\"";
      xml << " body1=\"" << xml_escape(item.value("body1", "")) << "\" body2=\""
          << xml_escape(item.value("body2", "")) << "\"/>\n";
    }
    return xml.str();
  }

  std::string generate_equality_xml(const nlohmann::json& scene) {
    std::ostringstream xml;
    for (const auto& eq : scene.value("equality", nlohmann::json::array()))
      xml << compile_equality(eq);
    return xml.str();
  }

  std::string generate_tendon_xml(const nlohmann::json& scene) {
    std::ostringstream xml;
    for (const auto& tendon : scene.value("tendon", nlohmann::json::array()))
      xml << compile_tendon(tendon);
    return xml.str();
  }

  std::string generate_actuator_xml(const nlohmann::json& scene) {
    std::ostringstream xml;
    for (const auto& actuator : scene.value("actuators", nlohmann::json::array()))
      xml << compile_actuator(actuator, scene);
    return xml.str();
  }

  std::string generate_sensor_xml(const nlohmann::json& scene) {
    std::ostringstream xml;
    if (scene.contains("objects") && scene["objects"].is_array()) {
      for (const auto& object : scene["objects"]) {
        if (is_sensor_type(object.value("type", ""))) xml << compile_sensor_object(object, scene);
      }
    }
    if (scene.contains("sensors") && scene["sensors"].is_array()) {
      for (const auto& sensor : scene["sensors"]) xml << compile_sensor_object(sensor, scene);
    }
    return xml.str();
  }

  std::string generate_keyframe_xml(const nlohmann::json& scene) {
    std::ostringstream xml;
    for (const auto& kf : scene.value("keyframes", nlohmann::json::array()))
      xml << compile_keyframe(kf);
    return xml.str();
  }

  // ---------------- compiler / size / statistic / visual (attribute-only or
  // fixed-sub-block globals; see xml_native_writer.cc:953-1256 for the
  // authoritative attribute lists this mirrors) ----------------

  // Full attribute set confirmed against xml_native_reader.cc:103-106's MJCF
  // schema table (the writer only re-emits attributes that differ from
  // engine defaults, so it under-reports what's actually accepted here).
  std::string generate_compiler_xml(const nlohmann::json& physics) {
    const auto compiler = physics.value("compiler", nlohmann::json::object());
    if (!compiler.is_object() || compiler.empty()) return {};
    std::ostringstream xml;
    xml << "  <compiler";
    emit_bool_attr(xml, compiler, "autolimits");
    emit_num_attr(xml, compiler, "boundmass");
    emit_num_attr(xml, compiler, "boundinertia");
    emit_num_attr(xml, compiler, "settotalmass");
    emit_bool_attr(xml, compiler, "balanceinertia");
    emit_bool_attr(xml, compiler, "strippath");
    if (compiler.contains("coordinate")) {
      const std::string coordinate = compiler.value("coordinate", "");
      if (coordinate == "local" || coordinate == "global")
        xml << " coordinate=\"" << coordinate << "\"";
    }
    emit_str_attr(xml, compiler, "angle");
    emit_bool_attr(xml, compiler, "fitaabb");
    emit_str_attr(xml, compiler, "eulerseq");
    emit_str_attr(xml, compiler, "meshdir");
    emit_str_attr(xml, compiler, "texturedir");
    emit_bool_attr(xml, compiler, "discardvisual");
    emit_bool_attr(xml, compiler, "usethread");
    emit_bool_attr(xml, compiler, "fusestatic");
    if (compiler.contains("inertiafromgeom")) {
      const std::string mode = compiler.value("inertiafromgeom", "");
      if (mode == "true" || mode == "false" || mode == "auto")
        xml << " inertiafromgeom=\"" << mode << "\"";
    }
    emit_vec_attr(xml, compiler, "inertiagrouprange", 2);
    emit_bool_attr(xml, compiler, "saveinertial");
    emit_str_attr(xml, compiler, "assetdir");
    emit_bool_attr(xml, compiler, "alignfree");
    xml << "/>\n";
    return xml.str();
  }

  std::string generate_size_xml(const nlohmann::json& physics) {
    const auto size = physics.value("size", nlohmann::json::object());
    if (!size.is_object() || size.empty()) return {};
    std::ostringstream xml;
    xml << "  <size";
    emit_str_attr(xml, size, "memory");
    static const char* kIntFields[]
        = {"njmax",        "nconmax",        "nstack",      "nuserdata",  "nkey",
           "nuser_body",   "nuser_jnt",      "nuser_geom",  "nuser_site", "nuser_cam",
           "nuser_tendon", "nuser_actuator", "nuser_sensor"};
    for (const char* field : kIntFields) emit_int_attr(xml, size, field);
    xml << "/>\n";
    return xml.str();
  }

  std::string generate_statistic_xml(const nlohmann::json& environment) {
    const auto statistic = environment.value("statistic", nlohmann::json::object());
    if (!statistic.is_object() || statistic.empty()) return {};
    std::ostringstream xml;
    xml << "  <statistic";
    emit_num_attr(xml, statistic, "meaninertia");
    emit_num_attr(xml, statistic, "meanmass");
    emit_num_attr(xml, statistic, "meansize");
    emit_num_attr(xml, statistic, "extent");
    emit_vec_attr(xml, statistic, "center", 3);
    xml << "/>\n";
    return xml.str();
  }

  std::string generate_visual_xml(const nlohmann::json& environment) {
    const auto visual = environment.value("visual", nlohmann::json::object());
    if (!visual.is_object() || visual.empty()) return {};
    std::ostringstream body;

    const auto emit_block
        = [&](const char* tag, const nlohmann::json& obj,
              const std::function<void(std::ostringstream&, const nlohmann::json&)>& fn) {
            if (!obj.is_object() || obj.empty()) return;
            std::ostringstream attrs;
            fn(attrs, obj);
            const auto s = attrs.str();
            if (!s.empty()) body << "    <" << tag << s << "/>\n";
          };

    emit_block("global", visual.value("global", nlohmann::json::object()), [](auto& a, auto& o) {
      emit_int_attr(a, o, "cameraid");
      emit_bool_attr(a, o, "orthographic");
      emit_num_attr(a, o, "fovy");
      emit_num_attr(a, o, "ipd");
      emit_num_attr(a, o, "azimuth");
      emit_num_attr(a, o, "elevation");
      emit_num_attr(a, o, "linewidth");
      emit_num_attr(a, o, "glow");
      emit_num_attr(a, o, "realtime");
      emit_int_attr(a, o, "offwidth");
      emit_int_attr(a, o, "offheight");
      emit_bool_attr(a, o, "ellipsoidinertia");
      emit_bool_attr(a, o, "bvactive");
    });
    emit_block("quality", visual.value("quality", nlohmann::json::object()), [](auto& a, auto& o) {
      emit_int_attr(a, o, "shadowsize");
      emit_int_attr(a, o, "offsamples");
      emit_int_attr(a, o, "numslices");
      emit_int_attr(a, o, "numstacks");
      emit_int_attr(a, o, "numquads");
    });
    emit_block("headlight", visual.value("headlight", nlohmann::json::object()),
               [](auto& a, auto& o) {
                 emit_vec_attr(a, o, "ambient", 3);
                 emit_vec_attr(a, o, "diffuse", 3);
                 emit_vec_attr(a, o, "specular", 3);
                 emit_int_attr(a, o, "active");
               });
    emit_block("map", visual.value("map", nlohmann::json::object()), [](auto& a, auto& o) {
      for (const char* key :
           {"stiffness", "stiffnessrot", "force", "torque", "alpha", "fogstart", "fogend", "znear",
            "zfar", "haze", "shadowclip", "shadowscale", "actuatortendon"})
        emit_num_attr(a, o, key);
    });
    emit_block("scale", visual.value("scale", nlohmann::json::object()), [](auto& a, auto& o) {
      for (const char* key :
           {"forcewidth", "contactwidth", "contactheight", "connect", "com", "camera", "light",
            "selectpoint", "jointlength", "jointwidth", "actuatorlength", "actuatorwidth",
            "framelength", "framewidth", "constraint", "slidercrank", "frustum"})
        emit_num_attr(a, o, key);
    });
    emit_block("rgba", visual.value("rgba", nlohmann::json::object()), [](auto& a, auto& o) {
      for (const char* key : {"fog",
                              "haze",
                              "force",
                              "inertia",
                              "joint",
                              "actuator",
                              "actuatornegative",
                              "actuatorpositive",
                              "com",
                              "camera",
                              "light",
                              "selectpoint",
                              "connect",
                              "contactpoint",
                              "contactforce",
                              "contactfriction",
                              "contacttorque",
                              "contactgap",
                              "rangefinder",
                              "constraint",
                              "slidercrank",
                              "crankbroken",
                              "frustum",
                              "bv",
                              "bvactive"})
        emit_vec_attr(a, o, key, 4);
    });

    const std::string inner = body.str();
    if (inner.empty()) return {};
    std::ostringstream xml;
    xml << "  <visual>\n" << inner << "  </visual>\n";
    return xml.str();
  }

  // ---------------- custom: <numeric>/<text>/<tuple> ----------------

  std::string generate_custom_xml(const nlohmann::json& scene) {
    const auto custom = scene.value("custom", nlohmann::json::object());
    if (!custom.is_object() || custom.empty()) return {};
    std::ostringstream xml;
    for (const auto& item : custom.value("numeric", nlohmann::json::array())) {
      const std::string id = item.value("id", "");
      if (id.empty()) continue;
      xml << "    <numeric name=\"" << xml_escape(id) << "\"";
      if (item.contains("size") && item["size"].is_number_integer())
        xml << " size=\"" << item["size"].get<int>() << "\"";
      xml << " data=\"" << numeric_list(item.value("data", nlohmann::json::array()), "0", 0)
          << "\"/>\n";
    }
    for (const auto& item : custom.value("text", nlohmann::json::array())) {
      const std::string id = item.value("id", "");
      if (id.empty()) continue;
      xml << "    <text name=\"" << xml_escape(id) << "\" data=\""
          << xml_escape(item.value("data", "")) << "\"/>\n";
    }
    for (const auto& item : custom.value("tuple", nlohmann::json::array())) {
      const std::string id = item.value("id", "");
      if (id.empty()) continue;
      xml << "    <tuple name=\"" << xml_escape(id) << "\">\n";
      for (const auto& obj : item.value("objects", nlohmann::json::array())) {
        xml << "      <element objtype=\"" << xml_escape(obj.value("objtype", ""))
            << "\" objname=\"" << xml_escape(obj.value("objname", "")) << "\"";
        if (obj.contains("prm") && obj["prm"].is_number())
          xml << " prm=\"" << obj["prm"].get<double>() << "\"";
        xml << "/>\n";
      }
      xml << "    </tuple>\n";
    }
    return xml.str();
  }

  // ---------------- deformable: <flex>/<skin> (static/bind-pose only) ----------------

  std::string compile_flex(const nlohmann::json& flex) {
    const std::string id = flex.value("id", "");
    if (id.empty()) throw std::invalid_argument("flex missing 'id'");
    const std::string body = flex.value("body", "");
    // MuJoCo requires a single rigid `body` for a flex whose vertices don't each
    // carry their own body assignment via <node> (a more advanced per-vertex-body
    // form this schema doesn't support yet) -- confirmed by a live compile
    // ("required attribute missing: 'body'") rather than the schema table alone,
    // which doesn't mark it required.
    if (body.empty()) throw std::invalid_argument("flex '" + id + "' missing 'body'");
    const int dim = flex.value("dim", 2);
    std::ostringstream xml;
    xml << "    <flex name=\"" << xml_escape(id) << "\" dim=\"" << dim << "\" body=\""
        << xml_escape(body) << "\"";
    if (flex.contains("radius") && flex["radius"].is_number())
      xml << " radius=\"" << flex["radius"].get<double>() << "\"";
    if (flex.contains("rgba"))
      xml << " rgba=\"" << numeric_list(flex["rgba"], "0.2 0.4 0.8 1", 4) << "\"";
    xml << " vertex=\"" << numeric_list(flex.value("vertex", nlohmann::json::array()), "", 0)
        << "\"";
    xml << " element=\"";
    bool first = true;
    for (const auto& elem : flex.value("element", nlohmann::json::array())) {
      for (const auto& idx : elem) {
        if (!first) xml << " ";
        xml << idx.get<int>();
        first = false;
      }
    }
    xml << "\"/>\n";
    return xml.str();
  }

  std::string compile_skin(const nlohmann::json& skin) {
    const std::string id = skin.value("id", "");
    if (id.empty()) throw std::invalid_argument("skin missing 'id'");
    std::ostringstream xml;
    xml << "    <skin name=\"" << xml_escape(id) << "\"";
    if (!skin.value("material", "").empty())
      xml << " material=\"" << xml_escape(skin.value("material", "")) << "\"";
    if (skin.contains("rgba"))
      xml << " rgba=\"" << numeric_list(skin["rgba"], "1 1 1 1", 4) << "\"";
    xml << " vertex=\"" << numeric_list(skin.value("vertex", nlohmann::json::array()), "", 0)
        << "\"";
    xml << " face=\"";
    bool first = true;
    for (const auto& face : skin.value("face", nlohmann::json::array())) {
      for (const auto& idx : face) {
        if (!first) xml << " ";
        xml << idx.get<int>();
        first = false;
      }
    }
    xml << "\">\n";
    for (const auto& bone : skin.value("bone", nlohmann::json::array())) {
      xml << "      <bone body=\"" << xml_escape(bone.value("body", "")) << "\" bindpos=\""
          << numeric_list(bone.value("bindpos", nlohmann::json::array()), "0 0 0", 3)
          << "\" bindquat=\""
          << numeric_list(bone.value("bindquat", nlohmann::json::array()), "1 0 0 0", 4)
          << "\" vertid=\"";
      bool first_v = true;
      for (const auto& idx : bone.value("vertid", nlohmann::json::array())) {
        if (!first_v) xml << " ";
        xml << idx.get<int>();
        first_v = false;
      }
      xml << "\" vertweight=\""
          << numeric_list(bone.value("vertweight", nlohmann::json::array()), "", 0) << "\"/>\n";
    }
    xml << "    </skin>\n";
    return xml.str();
  }

  std::string generate_deformable_xml(const nlohmann::json& scene) {
    std::ostringstream xml;
    for (const auto& flex : scene.value("flexes", nlohmann::json::array()))
      xml << compile_flex(flex);
    for (const auto& skin : scene.value("skins", nlohmann::json::array()))
      xml << compile_skin(skin);
    return xml.str();
  }

}  // namespace

SimulationCompiler::SimulationCompiler(std::filesystem::path output_dir, size_t max_cached_models)
    : output_dir_(std::move(output_dir)),
      max_cached_models_(max_cached_models == 0 ? 1 : max_cached_models) {}

nlohmann::json SimulationCompiler::validate_scene(const nlohmann::json& scene) const {
  nlohmann::json errors = nlohmann::json::array();
  nlohmann::json warnings = nlohmann::json::array();

  if (!scene.is_object()) {
    add_issue(errors, "$", "scene must be an object");
    return {{"valid", false}, {"errors", errors}, {"warnings", warnings}};
  }

  if (!scene.contains("id") || !scene["id"].is_string() || scene.value("id", "").empty()) {
    add_issue(errors, "$.id", "scene missing string id");
  }
  if (!scene.value("model_path", "").empty()) {
    add_issue(errors, "$.model_path",
              "scene model_path is no longer supported; register the base model as an asset and "
              "reference it in models[], and use scene physics/environment for globals");
  }
  const auto check_vec = [&errors](const nlohmann::json& parent, const char* key,
                                   const std::string& path, size_t count) {
    if (parent.contains(key) && (!parent[key].is_array() || parent[key].size() != count))
      add_issue(errors, path + "." + key,
                std::string(key) + " must be an array of " + std::to_string(count) + " numbers");
  };
  if (scene.contains("physics") && !scene["physics"].is_object()) {
    add_issue(errors, "$.physics", "physics must be an object");
  } else if (scene.contains("physics")) {
    const auto& physics = scene["physics"];
    if (physics.contains("timestep")
        && (!physics["timestep"].is_number() || physics["timestep"].get<double>() <= 0.0))
      add_issue(errors, "$.physics.timestep", "physics.timestep must be a positive number");
    check_vec(physics, "gravity", "$.physics", 3);
    check_vec(physics, "wind", "$.physics", 3);
    check_vec(physics, "magnetic", "$.physics", 3);
    for (const char* key : {"density", "viscosity"}) {
      if (physics.contains(key) && (!physics[key].is_number() || physics[key].get<double>() < 0.0))
        add_issue(errors, std::string("$.physics.") + key,
                  std::string(key) + " must be a non-negative number");
    }
    if (physics.contains("integrator")
        && (!physics["integrator"].is_string()
            || !canonical_integrator(physics.value("integrator", ""))))
      add_issue(errors, "$.physics.integrator",
                "integrator must be one of: euler, rk4, implicit, implicitfast");
    if (physics.contains("solver")
        && (!physics["solver"].is_string() || !canonical_solver(physics.value("solver", ""))))
      add_issue(errors, "$.physics.solver", "solver must be one of: pgs, cg, newton");
    if (physics.contains("iterations")
        && (!physics["iterations"].is_number_integer() || physics["iterations"].get<int>() <= 0))
      add_issue(errors, "$.physics.iterations", "iterations must be a positive integer");
  }
  if (scene.contains("environment") && !scene["environment"].is_object()) {
    add_issue(errors, "$.environment", "environment must be an object");
  } else if (scene.contains("environment")) {
    const auto& environment = scene["environment"];
    if (environment.contains("ground") && !environment["ground"].is_boolean()
        && !environment["ground"].is_object()) {
      add_issue(errors, "$.environment.ground", "ground must be a boolean or an object");
    } else if (environment.contains("ground") && environment["ground"].is_object()) {
      const auto& ground = environment["ground"];
      if (ground.contains("size")
          && (!ground["size"].is_number() || ground["size"].get<double>() <= 0.0))
        add_issue(errors, "$.environment.ground.size", "size must be a positive number");
      if (ground.contains("reflectance")
          && (!ground["reflectance"].is_number() || ground["reflectance"].get<double>() < 0.0
              || ground["reflectance"].get<double>() > 1.0))
        add_issue(errors, "$.environment.ground.reflectance",
                  "reflectance must be a number in [0, 1]");
      if (ground.contains("friction")
          && (!ground["friction"].is_number() || ground["friction"].get<double>() <= 0.0))
        add_issue(errors, "$.environment.ground.friction", "friction must be a positive number");
      check_vec(ground, "rgba", "$.environment.ground", 4);
      check_vec(ground, "rgb1", "$.environment.ground", 3);
      check_vec(ground, "rgb2", "$.environment.ground", 3);
    }
    if (environment.contains("skybox") && !environment["skybox"].is_boolean()
        && !environment["skybox"].is_object()) {
      add_issue(errors, "$.environment.skybox", "skybox must be a boolean or an object");
    } else if (environment.contains("skybox") && environment["skybox"].is_object()) {
      check_vec(environment["skybox"], "rgb1", "$.environment.skybox", 3);
      check_vec(environment["skybox"], "rgb2", "$.environment.skybox", 3);
    }
    if (environment.contains("lights")) {
      if (!environment["lights"].is_array()) {
        add_issue(errors, "$.environment.lights", "lights must be an array");
      } else {
        for (size_t i = 0; i < environment["lights"].size(); ++i) {
          const auto& light = environment["lights"][i];
          const std::string path = "$.environment.lights[" + std::to_string(i) + "]";
          if (!light.is_object()) {
            add_issue(errors, path, "light must be an object");
            continue;
          }
          check_vec(light, "pos", path, 3);
          check_vec(light, "dir", path, 3);
          check_vec(light, "diffuse", path, 3);
          check_vec(light, "ambient", path, 3);
          check_vec(light, "specular", path, 3);
        }
      }
    }
  }

  std::set<std::string> ids;
  if (scene.contains("models")) {
    if (!scene["models"].is_array()) {
      add_issue(errors, "$.models", "models must be an array");
    } else {
      for (size_t i = 0; i < scene["models"].size(); ++i) {
        const auto& model = scene["models"][i];
        const std::string path = "$.models[" + std::to_string(i) + "]";
        if (!model.is_object()) {
          add_issue(errors, path, "model must be an object");
          continue;
        }
        const std::string id = model.value("id", "");
        const std::string model_id = model.value("model_id", "");
        const std::string source = model.value("source", "");
        if (id.empty())
          add_issue(errors, path + ".id", "model missing id");
        else if (!ids.insert(id).second)
          add_issue(errors, path + ".id", "duplicate scene id: " + id);
        if (model_id.empty()) {
          add_issue(errors, path + ".model_id",
                    "model missing model_id (register the asset and reference it by id)");
        } else if (source.empty()) {
          add_issue(errors, path + ".model_id",
                    "model_id did not resolve to a registered asset: " + model_id);
        } else {
          try {
            const auto source_path = resolve_scene_path(scene, source);
            if (!std::filesystem::exists(source_path))
              add_issue(errors, path + ".source",
                        "model source does not exist: " + source_path.string());
            if (source_path.extension() == ".urdf")
              add_issue(errors, path + ".source",
                        "attached models must be MJCF; URDF is only supported as a base model");
          } catch (const std::exception& e) {
            add_issue(errors, path + ".source", e.what());
          }
        }
      }
    }
  }
  // ---- plugins[]: MuJoCo <extension> plugin instances. Generic passthrough
  // (id + dotted plugin family name + opaque config) referenced by id (as
  // `plugin_instance`) from bodies[].geoms[], actuators[] and sensors[]/
  // objects[] -- validated before those so the reference can be cross-checked.
  std::set<std::string> plugin_ids;
  if (scene.contains("plugins")) {
    if (!scene["plugins"].is_array()) {
      add_issue(errors, "$.plugins", "plugins must be an array");
    } else {
      for (size_t i = 0; i < scene["plugins"].size(); ++i) {
        const auto& plugin = scene["plugins"][i];
        const std::string path = "$.plugins[" + std::to_string(i) + "]";
        if (!plugin.is_object()) {
          add_issue(errors, path, "plugin must be an object");
          continue;
        }
        const std::string id = plugin.value("id", "");
        if (id.empty()) {
          add_issue(errors, path + ".id", "plugin missing id");
          continue;
        }
        if (!ids.insert(id).second) {
          add_issue(errors, path + ".id", "duplicate scene id: " + id);
          continue;
        }
        if (plugin.value("plugin", "").empty())
          add_issue(errors, path + ".plugin",
                    "plugin missing 'plugin' (dotted MuJoCo plugin name, e.g. "
                    "mujoco.elasticity.cable)");
        if (plugin.contains("config") && !plugin["config"].is_object())
          add_issue(errors, path + ".config", "plugin config must be an object");
        plugin_ids.insert(id);
      }
    }
  }
  const auto check_plugin_ref
      = [&](const nlohmann::json& obj, const std::string& path, const char* key) {
          const std::string ref = obj.value(key, "");
          if (!ref.empty() && !plugin_ids.count(ref))
            add_issue(errors, path + "." + key,
                      std::string(key) + " references unknown plugin instance: " + ref);
        };

  // ---- assets[]: scene-declared mesh/texture/material/hfield, referenced by
  // id from objects[] (obstacle.mesh/obstacle.hfield) and bodies[].geoms[]
  // (mesh=/material=). Populated before objects[]/bodies[] are validated below
  // so those references can be cross-checked.
  std::unordered_map<std::string, std::string> asset_kind_by_id;
  if (scene.contains("assets")) {
    if (!scene["assets"].is_array()) {
      add_issue(errors, "$.assets", "assets must be an array");
    } else {
      static const std::set<std::string> kAssetKinds = {"mesh", "texture", "material", "hfield"};
      const auto& assets_arr = scene["assets"];
      // Collect ids/kinds first so a material's `texture=` can reference a
      // texture declared later in the array (asset order isn't significant).
      for (const auto& asset : assets_arr) {
        if (!asset.is_object()) continue;
        const std::string id = asset.value("id", "");
        const std::string kind = asset.value("kind", "");
        if (!id.empty() && kAssetKinds.count(kind)) asset_kind_by_id[id] = kind;
      }
      for (size_t i = 0; i < assets_arr.size(); ++i) {
        const auto& asset = assets_arr[i];
        const std::string path = "$.assets[" + std::to_string(i) + "]";
        if (!asset.is_object()) {
          add_issue(errors, path, "asset must be an object");
          continue;
        }
        const std::string id = asset.value("id", "");
        if (id.empty()) {
          add_issue(errors, path + ".id", "asset missing id");
          continue;
        }
        if (id.rfind("scene_", 0) == 0)
          add_issue(errors, path + ".id", "asset id must not start with reserved prefix 'scene_'");
        else if (!ids.insert(id).second)
          add_issue(errors, path + ".id", "duplicate scene id: " + id);
        const std::string kind = asset.value("kind", "");
        if (!kAssetKinds.count(kind)) {
          add_issue(errors, path + ".kind", "unsupported asset kind: " + kind);
          continue;
        }
        if (kind == "mesh" && !asset.value("plugin_instance", "").empty()) {
          // Procedural mesh: no file, geometry comes from the plugin instance.
          check_plugin_ref(asset, path, "plugin_instance");
        } else if (kind == "mesh" || kind == "texture" || kind == "hfield") {
          const std::string file = asset.value("file", "");
          if (file.empty()) {
            add_issue(
                errors, path + ".file",
                kind + " asset requires 'file'"
                    + (kind == "mesh" ? " (or 'plugin_instance' for a procedural mesh)" : ""));
          } else {
            try {
              const auto file_path = resolve_scene_path(scene, file);
              if (!std::filesystem::exists(file_path))
                add_issue(errors, path + ".file",
                          "asset file does not exist: " + file_path.string());
            } catch (const std::exception& e) {
              add_issue(errors, path + ".file", e.what());
            }
          }
        }
        if (kind == "hfield") check_vec(asset, "size", path, 4);
        if (kind == "material" && !asset.value("texture", "").empty()) {
          const std::string texture_id = asset.value("texture", "");
          const auto it = asset_kind_by_id.find(texture_id);
          if (it == asset_kind_by_id.end() || it->second != "texture")
            add_issue(errors, path + ".texture",
                      "material references unknown texture asset: " + texture_id);
        }
      }
    }
  }

  if (scene.contains("objects")) {
    if (!scene["objects"].is_array()) {
      add_issue(errors, "$.objects", "objects must be an array");
    } else {
      static const std::set<std::string> kObstacleTypes
          = {"obstacle.box",       "obstacle.sphere", "obstacle.cylinder", "obstacle.capsule",
             "obstacle.ellipsoid", "obstacle.mesh",   "obstacle.hfield"};
      for (size_t i = 0; i < scene["objects"].size(); ++i) {
        const auto& object = scene["objects"][i];
        const std::string path = "$.objects[" + std::to_string(i) + "]";
        if (!object.is_object()) {
          add_issue(errors, path, "object must be an object");
          continue;
        }
        const std::string id = object.value("id", "");
        const std::string type = object.value("type", "");
        if (id.empty()) {
          add_issue(errors, path + ".id", "object missing id");
        } else if (!ids.insert(id).second) {
          add_issue(errors, path + ".id", "duplicate scene object id: " + id);
        }
        if (type.empty()) add_issue(errors, path + ".type", "object missing type");
        if (is_sensor_type(type)) {
          validate_sensor_fields(errors, path, object, type);
          if (type == "sensor.plugin") check_plugin_ref(object, path, "plugin_instance");
        } else if (!kObstacleTypes.count(type)) {
          add_issue(errors, path + ".type", "unsupported object type: " + type);
        } else if (type == "obstacle.mesh") {
          const std::string mesh_ref = object.value("mesh", "");
          const auto it = asset_kind_by_id.find(mesh_ref);
          if (mesh_ref.empty() || it == asset_kind_by_id.end() || it->second != "mesh")
            add_issue(errors, path + ".mesh", "obstacle.mesh must reference a declared mesh asset");
        } else if (type == "obstacle.hfield") {
          const std::string hfield_ref = object.value("hfield", "");
          const auto it = asset_kind_by_id.find(hfield_ref);
          if (hfield_ref.empty() || it == asset_kind_by_id.end() || it->second != "hfield")
            add_issue(errors, path + ".hfield",
                      "obstacle.hfield must reference a declared hfield asset");
        }
      }
    }
  }

  if (scene.contains("sensors")) {
    if (!scene["sensors"].is_array()) {
      add_issue(errors, "$.sensors", "sensors must be an array");
    } else {
      for (size_t i = 0; i < scene["sensors"].size(); ++i) {
        const auto& sensor = scene["sensors"][i];
        const std::string path = "$.sensors[" + std::to_string(i) + "]";
        if (!sensor.is_object()) {
          add_issue(errors, path, "sensor must be an object");
          continue;
        }
        const std::string id = sensor.value("id", "");
        if (id.empty())
          add_issue(errors, path + ".id", "sensor missing id");
        else if (!ids.insert(id).second)
          add_issue(errors, path + ".id", "duplicate scene object id: " + id);
        const std::string type = sensor.value("type", "");
        if (!is_sensor_type(type)) {
          add_issue(errors, path + ".type", "unsupported sensor type: " + type);
        } else {
          validate_sensor_fields(errors, path, sensor, type);
          if (type == "sensor.plugin") check_plugin_ref(sensor, path, "plugin_instance");
        }
      }
    }
  }

  if (scene.contains("contacts")) {
    const auto& contacts = scene["contacts"];
    if (!contacts.is_object() || !contacts.value("excludes", nlohmann::json::array()).is_array()) {
      add_issue(errors, "$.contacts.excludes", "contacts.excludes must be an array");
    } else {
      const auto excludes = contacts.value("excludes", nlohmann::json::array());
      for (size_t i = 0; i < excludes.size(); ++i) {
        const auto& item = excludes[i];
        const std::string path = "$.contacts.excludes[" + std::to_string(i) + "]";
        if (!item.is_object() || item.value("body1", "").empty() || item.value("body2", "").empty())
          add_issue(errors, path, "contact exclude requires body1 and body2");
      }
    }
    if (contacts.is_object() && contacts.contains("pairs")) {
      if (!contacts["pairs"].is_array()) {
        add_issue(errors, "$.contacts.pairs", "contacts.pairs must be an array");
      } else {
        const auto& pairs = contacts["pairs"];
        for (size_t i = 0; i < pairs.size(); ++i) {
          const auto& pair = pairs[i];
          const std::string path = "$.contacts.pairs[" + std::to_string(i) + "]";
          if (!pair.is_object()) {
            add_issue(errors, path, "contact pair must be an object");
            continue;
          }
          const std::string id = pair.value("id", "");
          if (!id.empty() && !ids.insert(id).second)
            add_issue(errors, path + ".id", "duplicate scene id: " + id);
          if (pair.value("geom1", "").empty() || pair.value("geom2", "").empty())
            add_issue(errors, path, "contact pair requires geom1 and geom2");
          check_vec(pair, "friction", path, 5);
          check_vec(pair, "solref", path, 2);
          check_vec(pair, "solreffriction", path, 2);
          check_vec(pair, "solimp", path, 5);
        }
      }
    }
  }

  // ---- defaultClasses[]: MJCF <default>/class inheritance tree. Class names
  // live in their own namespace (referenced via `class=`/`childclass=`, never
  // by the shared `ids` pool that models/bodies/joints/etc share), so they get
  // their own id set here. Populated before bodies[]/actuators[] are validated
  // below so their `class`/`childclass` fields can be cross-checked.
  std::unordered_map<std::string, std::string> class_parent;
  std::set<std::string> class_ids;
  if (scene.contains("defaultClasses")) {
    if (!scene["defaultClasses"].is_array()) {
      add_issue(errors, "$.defaultClasses", "defaultClasses must be an array");
    } else {
      const auto& classes = scene["defaultClasses"];
      for (size_t i = 0; i < classes.size(); ++i) {
        const auto& cls = classes[i];
        const std::string path = "$.defaultClasses[" + std::to_string(i) + "]";
        if (!cls.is_object()) {
          add_issue(errors, path, "default class must be an object");
          continue;
        }
        const std::string id = cls.value("id", "");
        if (id.empty()) {
          add_issue(errors, path + ".id", "default class missing id");
          continue;
        }
        if (!class_ids.insert(id).second) {
          add_issue(errors, path + ".id", "duplicate default class id: " + id);
          continue;
        }
        class_parent[id] = cls.value("parent", "");
        // compile_default_node's default_joint_attrs/default_geom_attrs/
        // default_site_attrs throw on an unsupported `type`, but that only
        // surfaces at compile time -- check it here too so scene.validate can
        // catch it up front (mirrors the same check already done for
        // bodies[].joints/geoms/sites below).
        if (cls.contains("joint") && cls["joint"].is_object() && cls["joint"].contains("type")) {
          const auto jtype = cls["joint"].value("type", "");
          if (!joint_type_names().count(jtype))
            add_issue(errors, path + ".joint.type", "unsupported joint type: " + jtype);
        }
        if (cls.contains("geom") && cls["geom"].is_object() && cls["geom"].contains("type")) {
          const auto gtype = cls["geom"].value("type", "");
          if (!geom_type_names().count(gtype))
            add_issue(errors, path + ".geom.type", "unsupported geom type: " + gtype);
        }
        if (cls.contains("site") && cls["site"].is_object() && cls["site"].contains("type")) {
          const auto stype = cls["site"].value("type", "");
          if (!site_shape_types().count(stype))
            add_issue(errors, path + ".site.type", "unsupported site type: " + stype);
        }
      }
      for (const auto& [id, parent] : class_parent) {
        if (parent.empty()) continue;
        if (!class_ids.count(parent)) {
          add_issue(errors, "$.defaultClasses", "class '" + id + "' has unknown parent: " + parent);
          continue;
        }
        std::set<std::string> seen{id};
        std::string cursor = parent;
        bool cyclic = false;
        while (!cursor.empty()) {
          if (!seen.insert(cursor).second) {
            cyclic = true;
            break;
          }
          const auto it = class_parent.find(cursor);
          cursor = it == class_parent.end() ? std::string{} : it->second;
        }
        if (cyclic)
          add_issue(errors, "$.defaultClasses", "class parent chain has a cycle involving: " + id);
      }
    }
  }
  const auto check_class_ref
      = [&](const nlohmann::json& obj, const std::string& path, const char* key) {
          const std::string cls = obj.value(key, "");
          if (!cls.empty() && !class_ids.count(cls))
            add_issue(errors, path + "." + key,
                      std::string(key) + " references unknown default class: " + cls);
        };

  // ---- bodies[]: nested body/joint/geom/site/camera tree ----
  std::unordered_map<std::string, std::string> body_parent;
  std::set<std::string> body_ids;
  if (scene.contains("bodies")) {
    if (!scene["bodies"].is_array()) {
      add_issue(errors, "$.bodies", "bodies must be an array");
    } else {
      for (size_t i = 0; i < scene["bodies"].size(); ++i) {
        const auto& body = scene["bodies"][i];
        const std::string path = "$.bodies[" + std::to_string(i) + "]";
        if (!body.is_object()) {
          add_issue(errors, path, "body must be an object");
          continue;
        }
        const std::string id = body.value("id", "");
        if (id.empty()) {
          add_issue(errors, path + ".id", "body missing id");
          continue;
        }
        if (!ids.insert(id).second) {
          add_issue(errors, path + ".id", "duplicate scene id: " + id);
          continue;
        }
        body_ids.insert(id);
        body_parent[id] = body.value("parent", "");
        check_class_ref(body, path, "childclass");

        if (body.contains("replicate")) {
          const std::string rpath = path + ".replicate";
          if (!body.value("parent", "").empty())
            add_issue(errors, rpath, "replicate is only allowed on a root body (no parent)");
          const auto& rep = body["replicate"];
          if (!rep.is_object()) {
            add_issue(errors, rpath, "replicate must be an object");
          } else {
            if (!rep.contains("count") || !rep["count"].is_number_integer()
                || rep["count"].get<int>() <= 0)
              add_issue(errors, rpath + ".count", "replicate.count must be a positive integer");
            if (!rep.contains("offset"))
              add_issue(errors, rpath + ".offset", "replicate.offset is required");
            else
              check_vec(rep, "offset", rpath, 3);
            if (rep.contains("euler")) check_vec(rep, "euler", rpath, 3);
            if (rep.contains("separate") && !rep["separate"].is_boolean())
              add_issue(errors, rpath + ".separate", "replicate.separate must be a boolean");
          }
        }

        const auto joints = body.value("joints", nlohmann::json::array());
        for (size_t j = 0; j < joints.size(); ++j) {
          const auto& joint = joints[j];
          const std::string jpath = path + ".joints[" + std::to_string(j) + "]";
          const std::string jid = joint.value("id", "");
          if (jid.empty())
            add_issue(errors, jpath + ".id", "joint missing id");
          else if (!ids.insert(jid).second)
            add_issue(errors, jpath + ".id", "duplicate scene id: " + jid);
          const std::string jtype = joint.value("type", "hinge");
          if (jtype != "hinge" && jtype != "slide" && jtype != "ball" && jtype != "free")
            add_issue(errors, jpath + ".type", "unsupported joint type: " + jtype);
          // compile_joint enforces these lengths via numeric_list's expected_size
          // (axis/range only apply to hinge/slide, matching compile_joint's own
          // condition) -- without checking them here, a bad length passes
          // validate_scene and only fails much later inside compile_scene.
          if ((jtype == "hinge" || jtype == "slide") && joint.contains("axis"))
            check_vec(joint, "axis", jpath, 3);
          if ((jtype == "hinge" || jtype == "slide") && joint.contains("range"))
            check_vec(joint, "range", jpath, 2);
          if (joint.contains("pos")) check_vec(joint, "pos", jpath, 3);
          check_class_ref(joint, jpath, "class");
        }
        const auto geoms = body.value("geoms", nlohmann::json::array());
        for (size_t j = 0; j < geoms.size(); ++j) {
          const auto& geom = geoms[j];
          const std::string gpath = path + ".geoms[" + std::to_string(j) + "]";
          const std::string gid = geom.value("id", "");
          if (gid.empty())
            add_issue(errors, gpath + ".id", "geom missing id");
          else if (!ids.insert(gid).second)
            add_issue(errors, gpath + ".id", "duplicate scene id: " + gid);
          const std::string gtype = geom.value("type", "box");
          if (!geom_type_names().count(gtype)) {
            add_issue(errors, gpath + ".type", "unsupported geom type: " + gtype);
          } else if (gtype == "mesh") {
            const std::string mesh_ref = geom.value("mesh", "");
            const auto it = asset_kind_by_id.find(mesh_ref);
            if (mesh_ref.empty() || it == asset_kind_by_id.end() || it->second != "mesh")
              add_issue(errors, gpath + ".mesh", "geom.mesh must reference a declared mesh asset");
          } else if (gtype == "hfield") {
            const std::string hfield_ref = geom.value("hfield", "");
            const auto it = asset_kind_by_id.find(hfield_ref);
            if (hfield_ref.empty() || it == asset_kind_by_id.end() || it->second != "hfield")
              add_issue(errors, gpath + ".hfield",
                        "geom.hfield must reference a declared hfield asset");
          } else if (gtype == "sdf") {
            if (geom.value("plugin_instance", "").empty())
              add_issue(errors, gpath + ".plugin_instance", "sdf geom requires 'plugin_instance'");
            else
              check_plugin_ref(geom, gpath, "plugin_instance");
            const std::string mesh_ref = geom.value("mesh", "");
            const auto mit = asset_kind_by_id.find(mesh_ref);
            if (mesh_ref.empty() || mit == asset_kind_by_id.end() || mit->second != "mesh")
              add_issue(errors, gpath + ".mesh",
                        "sdf geom requires 'mesh' referencing a declared (procedural) mesh asset");
          }
          const std::string material_ref = geom.value("material", "");
          if (!material_ref.empty()) {
            const auto it = asset_kind_by_id.find(material_ref);
            if (it == asset_kind_by_id.end() || it->second != "material")
              add_issue(errors, gpath + ".material",
                        "geom.material must reference a declared material asset");
          }
          // Matches compile_geom's own has_type-gated size-count enforcement
          // exactly: a size given without an explicit type is left to the class
          // chain to resolve there too, so no fixed length is enforced here
          // either (that geom would fail structural_signature/compile if the
          // class chain genuinely doesn't cover it, which is expected).
          if (geom.contains("size") && geom.contains("type")) {
            size_t expected = 0;
            if (gtype == "box" || gtype == "ellipsoid" || gtype == "plane")
              expected = 3;
            else if (gtype == "sphere")
              expected = 1;
            else if (gtype == "cylinder" || gtype == "capsule")
              expected = 2;
            if (expected > 0) check_vec(geom, "size", gpath, expected);
          }
          if (geom.contains("pos")) check_vec(geom, "pos", gpath, 3);
          if (geom.contains("quat")) check_vec(geom, "quat", gpath, 4);
          if (geom.contains("rgba")) check_vec(geom, "rgba", gpath, 4);
          check_class_ref(geom, gpath, "class");
        }
        const auto sites = body.value("sites", nlohmann::json::array());
        for (size_t j = 0; j < sites.size(); ++j) {
          const std::string sid = sites[j].value("id", "");
          const std::string spath = path + ".sites[" + std::to_string(j) + "]";
          if (sid.empty())
            add_issue(errors, spath + ".id", "site missing id");
          else if (!ids.insert(sid).second)
            add_issue(errors, spath + ".id", "duplicate scene id: " + sid);
          if (sites[j].contains("type")) {
            const auto stype = sites[j].value("type", "");
            if (!site_shape_types().count(stype))
              add_issue(errors, spath + ".type", "unsupported site type: " + stype);
          }
          if (sites[j].contains("pos")) check_vec(sites[j], "pos", spath, 3);
          if (sites[j].contains("quat")) check_vec(sites[j], "quat", spath, 4);
          check_class_ref(sites[j], spath, "class");
        }
        const auto cameras = body.value("cameras", nlohmann::json::array());
        for (size_t j = 0; j < cameras.size(); ++j) {
          const std::string cid = cameras[j].value("id", "");
          const std::string cpath = path + ".cameras[" + std::to_string(j) + "]";
          if (cid.empty())
            add_issue(errors, cpath + ".id", "camera missing id");
          else if (!ids.insert(cid).second)
            add_issue(errors, cpath + ".id", "duplicate scene id: " + cid);
          if (cameras[j].contains("pos")) check_vec(cameras[j], "pos", cpath, 3);
          if (cameras[j].contains("quat")) check_vec(cameras[j], "quat", cpath, 4);
          check_class_ref(cameras[j], cpath, "class");
        }
      }
      // parent references must resolve within this scene and must not cycle.
      for (const auto& [id, parent] : body_parent) {
        if (parent.empty()) continue;
        if (!body_ids.count(parent)) {
          add_issue(errors, "$.bodies", "body '" + id + "' has unknown parent: " + parent);
          continue;
        }
        std::set<std::string> seen{id};
        std::string cursor = parent;
        bool cyclic = false;
        while (!cursor.empty()) {
          if (!seen.insert(cursor).second) {
            cyclic = true;
            break;
          }
          const auto it = body_parent.find(cursor);
          cursor = it == body_parent.end() ? std::string{} : it->second;
        }
        if (cyclic) add_issue(errors, "$.bodies", "body parent chain has a cycle involving: " + id);
      }
    }
  }

  // ---- <replicate> cross-reference guard ----
  // A body wrapped in <replicate> gets renamed with a MuJoCo-generated numeric
  // suffix per copy; the scene JSON's flat `id` can't address "copy N", so
  // referencing a replicated body (or anything in its subtree) from
  // equality/contacts/sensors -- which all resolve by exact body name -- can
  // never work. Reject it here instead of failing confusingly at mj_loadXML
  // time. Re-scans the raw scene JSON (rather than reusing state from the
  // blocks above, which run before or after this one depending on section) so
  // it doesn't depend on validation order elsewhere in this function.
  std::set<std::string> replicated_ids;
  if (scene.contains("bodies") && scene["bodies"].is_array()) {
    std::set<std::string> replicate_roots;
    for (const auto& body : scene["bodies"]) {
      if (body.is_object() && body.contains("replicate") && body.value("parent", "").empty()) {
        const std::string id = body.value("id", "");
        if (!id.empty()) replicate_roots.insert(id);
      }
    }
    if (!replicate_roots.empty()) {
      std::function<void(const std::string&)> collect = [&](const std::string& id) {
        if (!replicated_ids.insert(id).second) return;
        for (const auto& [child, parent] : body_parent) {
          if (parent == id) collect(child);
        }
      };
      for (const auto& root : replicate_roots) collect(root);
    }
  }
  if (!replicated_ids.empty()) {
    const auto check_not_replicated
        = [&](const std::string& body_id, const std::string& path, const char* key) {
            if (!body_id.empty() && replicated_ids.count(body_id))
              add_issue(errors, path,
                        std::string(key) + " references a replicated body ('" + body_id
                            + "'): bodies inside <replicate> get MuJoCo-generated per-copy names "
                              "and cannot be addressed by a single scene id from "
                              "equality/contacts/sensors");
          };
    if (scene.contains("equality") && scene["equality"].is_array()) {
      for (size_t i = 0; i < scene["equality"].size(); ++i) {
        const auto& eq = scene["equality"][i];
        if (!eq.is_object()) continue;
        const std::string path = "$.equality[" + std::to_string(i) + "]";
        check_not_replicated(eq.value("body1", ""), path + ".body1", "body1");
        check_not_replicated(eq.value("body2", ""), path + ".body2", "body2");
      }
    }
    if (scene.contains("contacts") && scene["contacts"].is_object()) {
      const auto excludes = scene["contacts"].value("excludes", nlohmann::json::array());
      for (size_t i = 0; i < excludes.size(); ++i) {
        const auto& item = excludes[i];
        if (!item.is_object()) continue;
        const std::string path = "$.contacts.excludes[" + std::to_string(i) + "]";
        check_not_replicated(item.value("body1", ""), path + ".body1", "body1");
        check_not_replicated(item.value("body2", ""), path + ".body2", "body2");
      }
    }
    const auto check_sensor_array = [&](const nlohmann::json& arr, const char* array_name) {
      for (size_t i = 0; i < arr.size(); ++i) {
        const auto& sensor = arr[i];
        if (!sensor.is_object()) continue;
        const auto it = sensor_registry().find(sensor.value("type", ""));
        if (it == sensor_registry().end()) continue;
        const std::string path = std::string("$.") + array_name + "[" + std::to_string(i) + "]";
        for (const auto& ref : sensor_body_refs(sensor, it->second))
          check_not_replicated(ref, path, "body reference");
      }
    };
    if (scene.contains("objects") && scene["objects"].is_array())
      check_sensor_array(scene["objects"], "objects");
    if (scene.contains("sensors") && scene["sensors"].is_array())
      check_sensor_array(scene["sensors"], "sensors");
  }

  // ---- actuators[] ----
  if (scene.contains("actuators")) {
    if (!scene["actuators"].is_array()) {
      add_issue(errors, "$.actuators", "actuators must be an array");
    } else {
      for (size_t i = 0; i < scene["actuators"].size(); ++i) {
        const auto& actuator = scene["actuators"][i];
        const std::string path = "$.actuators[" + std::to_string(i) + "]";
        if (!actuator.is_object()) {
          add_issue(errors, path, "actuator must be an object");
          continue;
        }
        const std::string id = actuator.value("id", "");
        if (id.empty())
          add_issue(errors, path + ".id", "actuator missing id");
        else if (!ids.insert(id).second)
          add_issue(errors, path + ".id", "duplicate scene id: " + id);
        const std::string type = actuator.value("type", "");
        const auto spec = actuator_registry().find(type);
        if (spec == actuator_registry().end()) {
          add_issue(errors, path + ".type", "unsupported actuator type: " + type);
        } else if (actuator.value(spec->second, "").empty()) {
          add_issue(errors, path + "." + spec->second,
                    "actuator." + type + " requires '" + spec->second + "'");
        }
        if (type == "plugin") {
          if (actuator.value("plugin_instance", "").empty())
            add_issue(errors, path + ".plugin_instance",
                      "actuator.plugin requires 'plugin_instance'");
          else
            check_plugin_ref(actuator, path, "plugin_instance");
        }
        if (actuator.contains("ctrlrange")) check_vec(actuator, "ctrlrange", path, 2);
        if (actuator.contains("forcerange")) check_vec(actuator, "forcerange", path, 2);
        check_class_ref(actuator, path, "class");
      }
    }
  }

  // ---- equality[] ----
  if (scene.contains("equality")) {
    if (!scene["equality"].is_array()) {
      add_issue(errors, "$.equality", "equality must be an array");
    } else {
      for (size_t i = 0; i < scene["equality"].size(); ++i) {
        const auto& eq = scene["equality"][i];
        const std::string path = "$.equality[" + std::to_string(i) + "]";
        if (!eq.is_object()) {
          add_issue(errors, path, "equality constraint must be an object");
          continue;
        }
        const std::string id = eq.value("id", "");
        if (id.empty())
          add_issue(errors, path + ".id", "equality constraint missing id");
        else if (!ids.insert(id).second)
          add_issue(errors, path + ".id", "duplicate scene id: " + id);
        const std::string type = eq.value("type", "");
        if (type == "connect" || type == "weld") {
          if (eq.value("body1", "").empty() || eq.value("body2", "").empty())
            add_issue(errors, path, "equality." + type + " requires body1 and body2");
          if (type == "connect" && eq.contains("anchor")) check_vec(eq, "anchor", path, 3);
        } else if (type == "joint") {
          if (eq.value("joint1", "").empty())
            add_issue(errors, path + ".joint1", "equality.joint requires joint1");
        } else if (type == "tendon") {
          if (eq.value("tendon1", "").empty())
            add_issue(errors, path + ".tendon1", "equality.tendon requires tendon1");
          if (eq.contains("polycoef")) check_vec(eq, "polycoef", path, 5);
        } else {
          add_issue(errors, path + ".type", "unsupported equality type: " + type);
        }
        if (eq.contains("active") && !eq["active"].is_boolean())
          add_issue(errors, path + ".active", "active must be a boolean");
      }
    }
  }

  // ---- tendon[] ----
  if (scene.contains("tendon")) {
    if (!scene["tendon"].is_array()) {
      add_issue(errors, "$.tendon", "tendon must be an array");
    } else {
      for (size_t i = 0; i < scene["tendon"].size(); ++i) {
        const auto& tendon = scene["tendon"][i];
        const std::string path = "$.tendon[" + std::to_string(i) + "]";
        if (!tendon.is_object()) {
          add_issue(errors, path, "tendon must be an object");
          continue;
        }
        const std::string id = tendon.value("id", "");
        if (id.empty())
          add_issue(errors, path + ".id", "tendon missing id");
        else if (!ids.insert(id).second)
          add_issue(errors, path + ".id", "duplicate scene id: " + id);
        const std::string type = tendon.value("type", "");
        if (type == "fixed") {
          const auto tjoints = tendon.value("joints", nlohmann::json::array());
          if (tjoints.empty()) {
            add_issue(errors, path + ".joints", "tendon.fixed requires at least one joint");
          } else {
            // compile_tendon writes an empty joint="" attribute verbatim if this
            // is missing, which mj_loadXML only rejects much later with an
            // opaque message -- catch it here instead.
            for (size_t j = 0; j < tjoints.size(); ++j) {
              if (tjoints[j].value("joint", "").empty())
                add_issue(errors, path + ".joints[" + std::to_string(j) + "].joint",
                          "tendon.fixed joint entry requires 'joint'");
            }
          }
        } else if (type == "spatial") {
          const auto pathElements = tendon.value("path", nlohmann::json::array());
          if (pathElements.is_array() && !pathElements.empty()) {
            for (size_t j = 0; j < pathElements.size(); ++j) {
              const auto& p = pathElements[j];
              const std::string ppath = path + ".path[" + std::to_string(j) + "]";
              const std::string ptype = p.value("type", "site");
              if (ptype == "site" && p.value("site", "").empty())
                add_issue(errors, ppath + ".site", "tendon path 'site' element requires 'site'");
              else if (ptype == "geom" && p.value("geom", "").empty())
                add_issue(errors, ppath + ".geom", "tendon path 'geom' element requires 'geom'");
              else if (ptype != "site" && ptype != "geom" && ptype != "pulley")
                add_issue(errors, ppath + ".type",
                          "unsupported tendon path element type: " + ptype);
            }
          } else if (tendon.value("sites", nlohmann::json::array()).size() < 2) {
            add_issue(errors, path + ".sites",
                      "tendon.spatial requires at least 2 sites (or a path)");
          }
        } else {
          add_issue(errors, path + ".type", "unsupported tendon type: " + type);
        }
      }
    }
  }

  // ---- keyframes[] ----
  if (scene.contains("keyframes")) {
    if (!scene["keyframes"].is_array()) {
      add_issue(errors, "$.keyframes", "keyframes must be an array");
    } else {
      for (size_t i = 0; i < scene["keyframes"].size(); ++i) {
        const auto& kf = scene["keyframes"][i];
        const std::string path = "$.keyframes[" + std::to_string(i) + "]";
        if (!kf.is_object()) {
          add_issue(errors, path, "keyframe must be an object");
          continue;
        }
        const std::string id = kf.value("id", "");
        if (id.empty())
          add_issue(errors, path + ".id", "keyframe missing id");
        else if (!ids.insert(id).second)
          add_issue(errors, path + ".id", "duplicate scene id: " + id);
      }
    }
  }

  // ---- cameras[]: world-level cameras (not attached to any body) ----
  if (scene.contains("cameras")) {
    if (!scene["cameras"].is_array()) {
      add_issue(errors, "$.cameras", "cameras must be an array");
    } else {
      for (size_t i = 0; i < scene["cameras"].size(); ++i) {
        const auto& camera = scene["cameras"][i];
        const std::string path = "$.cameras[" + std::to_string(i) + "]";
        if (!camera.is_object()) {
          add_issue(errors, path, "camera must be an object");
          continue;
        }
        const std::string id = camera.value("id", "");
        if (id.empty())
          add_issue(errors, path + ".id", "camera missing id");
        else if (!ids.insert(id).second)
          add_issue(errors, path + ".id", "duplicate scene id: " + id);
        check_class_ref(camera, path, "class");
      }
    }
  }

  // ---- custom{}: <numeric>/<text>/<tuple> ----
  if (scene.contains("custom")) {
    if (!scene["custom"].is_object()) {
      add_issue(errors, "$.custom", "custom must be an object");
    } else {
      const auto& custom = scene["custom"];
      for (const char* group : {"numeric", "text", "tuple"}) {
        if (!custom.contains(group)) continue;
        if (!custom[group].is_array()) {
          add_issue(errors, std::string("$.custom.") + group,
                    std::string(group) + " must be an array");
          continue;
        }
        for (size_t i = 0; i < custom[group].size(); ++i) {
          const auto& item = custom[group][i];
          const std::string path = "$.custom." + std::string(group) + "[" + std::to_string(i) + "]";
          if (!item.is_object()) {
            add_issue(errors, path, "custom entry must be an object");
            continue;
          }
          const std::string id = item.value("id", "");
          if (id.empty())
            add_issue(errors, path + ".id", "custom entry missing id");
          else if (!ids.insert(id).second)
            add_issue(errors, path + ".id", "duplicate scene id: " + id);
        }
      }
    }
  }

  // ---- flexes[]/skins[]: <deformable> ----
  if (scene.contains("flexes")) {
    if (!scene["flexes"].is_array()) {
      add_issue(errors, "$.flexes", "flexes must be an array");
    } else {
      for (size_t i = 0; i < scene["flexes"].size(); ++i) {
        const auto& flex = scene["flexes"][i];
        const std::string path = "$.flexes[" + std::to_string(i) + "]";
        if (!flex.is_object()) {
          add_issue(errors, path, "flex must be an object");
          continue;
        }
        const std::string id = flex.value("id", "");
        if (id.empty())
          add_issue(errors, path + ".id", "flex missing id");
        else if (!ids.insert(id).second)
          add_issue(errors, path + ".id", "duplicate scene id: " + id);
        if (flex.value("body", "").empty())
          add_issue(errors, path + ".body",
                    "flex missing body (per-vertex body assignment via <node> isn't supported)");
        const int dim = flex.value("dim", 2);
        if (dim < 1 || dim > 3) add_issue(errors, path + ".dim", "flex.dim must be 1, 2 or 3");
        const auto vertex = flex.value("vertex", nlohmann::json::array());
        const size_t vertex_count = vertex.is_array() ? vertex.size() / 3 : 0;
        for (const auto& elem : flex.value("element", nlohmann::json::array())) {
          if (!elem.is_array()) continue;
          for (const auto& idx : elem) {
            if (idx.is_number_integer()
                && (idx.get<int>() < 0 || static_cast<size_t>(idx.get<int>()) >= vertex_count))
              add_issue(errors, path + ".element", "flex.element references out-of-range vertex");
          }
        }
      }
    }
  }
  if (scene.contains("skins")) {
    if (!scene["skins"].is_array()) {
      add_issue(errors, "$.skins", "skins must be an array");
    } else {
      for (size_t i = 0; i < scene["skins"].size(); ++i) {
        const auto& skin = scene["skins"][i];
        const std::string path = "$.skins[" + std::to_string(i) + "]";
        if (!skin.is_object()) {
          add_issue(errors, path, "skin must be an object");
          continue;
        }
        const std::string id = skin.value("id", "");
        if (id.empty())
          add_issue(errors, path + ".id", "skin missing id");
        else if (!ids.insert(id).second)
          add_issue(errors, path + ".id", "duplicate scene id: " + id);
        for (size_t b = 0; b < skin.value("bone", nlohmann::json::array()).size(); ++b) {
          const auto& bone = skin["bone"][b];
          const std::string bpath = path + ".bone[" + std::to_string(b) + "]";
          if (bone.value("body", "").empty())
            add_issue(errors, bpath + ".body", "skin bone missing body");
          const auto vertid = bone.value("vertid", nlohmann::json::array());
          const auto vertweight = bone.value("vertweight", nlohmann::json::array());
          if (vertid.is_array() && vertweight.is_array() && vertid.size() != vertweight.size())
            add_issue(errors, bpath, "skin bone vertid/vertweight length mismatch");
        }
      }
    }
  }

  if (!scene.contains("initial_state") && !scene.contains("defaults")) {
    add_issue(warnings, "$.initial_state",
              "scene has no initial_state; instance will use runtime defaults");
  }

  return {{"valid", errors.empty()}, {"errors", errors}, {"warnings", warnings}};
}

nlohmann::json SimulationCompiler::diff_scenes(const nlohmann::json& old_scene,
                                               const nlohmann::json& new_scene) const {
  nlohmann::json structural_changes = nlohmann::json::array();
  nlohmann::json runtime_changes = nlohmann::json::array();

  const auto old_signature = structural_signature(old_scene);
  const auto new_signature = structural_signature(new_scene);
  std::set<std::string> keys;
  for (const auto& [key, value] : old_signature.items()) {
    (void)value;
    keys.insert(key);
  }
  for (const auto& [key, value] : new_signature.items()) {
    (void)value;
    keys.insert(key);
  }
  for (const auto& key : keys) {
    if (old_signature.value(key, nlohmann::json()) != new_signature.value(key, nlohmann::json()))
      append_change(structural_changes, "$." + key, key);
  }

  if (runtime_state(old_scene) != runtime_state(new_scene))
    append_change(runtime_changes, "$.initial_state", "initial_state");

  const bool structural = !structural_changes.empty();
  const bool runtime = !runtime_changes.empty();
  return {
      {"structural_changed", structural},         {"runtime_changed", runtime},
      {"requires_recreate", structural},          {"can_apply_runtime", runtime && !structural},
      {"structural_changes", structural_changes}, {"runtime_changes", runtime_changes},
      {"old_signature", old_signature},           {"new_signature", new_signature},
  };
}

nlohmann::json SimulationCompiler::compile_scene(const nlohmann::json& scene,
                                                 ModelPtr* out_model) const {
  const auto validation = validate_scene(scene);
  if (!validation.value("valid", false)) {
    throw std::invalid_argument("scene validation failed: " + validation["errors"].dump());
  }

  // structural_model_id already folds in the scene id, structural signature and
  // every referenced model file's mtime, so it changes whenever the compiled
  // output would change. Check the cache with it *before* doing any of the
  // expensive XML-generation/disk-write/model-load work below, so a cache hit
  // (the common case: repeated instance.create/scene.apply against an
  // unchanged scene) skips all of it instead of redoing it and discarding the
  // result.
  const std::string compiled_model_id = structural_model_id(scene);
  ModelPtr model;
  std::filesystem::path compiled_model_path;
  {
    std::lock_guard lock{model_mutex_};
    auto it = models_.find(compiled_model_id);
    if (it != models_.end()) {
      model = touch_and_reclaim_locked(compiled_model_id, it->second.model);
      compiled_model_path = it->second.compiled_path;
    }
  }
  if (!model) {
    // Name the file by the content-derived compiled_model_id (not just the scene id) so
    // two concurrent compiles of the same scene id with different content can never race
    // on the same path -- one overwriting mid-read by the other.
    compiled_model_path = write_compiled_mjcf(scene, compiled_model_id);
    auto loaded = load_model(compiled_model_path);
    std::lock_guard lock{model_mutex_};
    model = touch_and_reclaim_locked(compiled_model_id, std::move(loaded), compiled_model_path);
  }

  nlohmann::json compiled;
  compiled["scene_id"] = scene.value("id", "");
  compiled["compiled_model_id"] = compiled_model_id;
  compiled["backend"] = "mujoco";
  compiled["model_path"] = compiled_model_path.string();
  compiled["generated"] = true;
  compiled["initial_state"] = runtime_state(scene);
  compiled["sizes"] = {{"nq", model->nq},
                       {"nv", model->nv},
                       {"nu", model->nu},
                       {"nbody", model->nbody},
                       {"nsensor", model->nsensor}};
  // 编译模型里的具名 site/joint：前端场景传感器/执行器编辑器的候选来源。
  // 未命名的对象无法被 MJCF 传感器/执行器引用，不能作为候选。
  compiled["sites"] = nlohmann::json::array();
  for (int i = 0; i < model->nsite; ++i) {
    const char* site_name = mj_id2name(model.get(), mjOBJ_SITE, i);
    if (site_name && site_name[0] != '\0') compiled["sites"].push_back(std::string(site_name));
  }
  compiled["joints"] = nlohmann::json::array();
  for (int i = 0; i < model->njnt; ++i) {
    const char* joint_name = mj_id2name(model.get(), mjOBJ_JOINT, i);
    if (joint_name && joint_name[0] != '\0') compiled["joints"].push_back(std::string(joint_name));
  }
  compiled["mapping"] = {
      {"actuators", nlohmann::json::object()}, {"sensors", nlohmann::json::object()},
      {"bodies", nlohmann::json::object()},    {"models", nlohmann::json::object()},
      {"assets", nlohmann::json::object()},
  };

  if (scene.contains("models") && scene["models"].is_array()) {
    for (const auto& asset : scene["models"]) {
      const std::string id = asset.value("id", "");
      if (!id.empty()) {
        compiled["mapping"]["models"][id] = {
            {"source", resolve_scene_path(scene, asset.value("source", "")).string()},
            {"prefix", asset.value("prefix", id + "_")},
            {"body", asset.value("body", "")},
        };
      }
    }
  }

  if (scene.contains("objects") && scene["objects"].is_array()) {
    for (const auto& object : scene["objects"]) {
      const std::string id = object.value("id", "");
      const std::string type = object.value("type", "");
      if (id.empty()) continue;
      if (is_sensor_type(type)) {
        // Echo the sensor entry wholesale rather than hand-picking fields:
        // reference shapes now range from a single site/joint/tendon/actuator/
        // body to objtype+objname(+reftype+refname) to geom1|body1+geom2|body2,
        // and enumerating every possible field here would need updating every
        // time sensor_registry() grows.
        compiled["mapping"]["sensors"][id] = object;
      } else {
        compiled["mapping"]["bodies"][id] = id;
      }
    }
  }

  if (scene.contains("sensors") && scene["sensors"].is_array()) {
    for (const auto& sensor : scene["sensors"]) {
      const std::string id = sensor.value("id", "");
      if (!id.empty()) compiled["mapping"]["sensors"][id] = sensor;
    }
  }

  if (scene.contains("actuators") && scene["actuators"].is_array()) {
    for (const auto& actuator : scene["actuators"]) {
      const std::string id = actuator.value("id", "");
      if (!id.empty()) {
        compiled["mapping"]["actuators"][id] = {
            {"type", actuator.value("type", "")},
            {"joint", actuator.value("joint", "")},
        };
      }
    }
  }

  if (scene.contains("assets") && scene["assets"].is_array()) {
    for (const auto& asset : scene["assets"]) {
      const std::string id = asset.value("id", "");
      if (!id.empty()) compiled["mapping"]["assets"][id] = {{"kind", asset.value("kind", "")}};
    }
  }

  if (out_model) *out_model = model;
  return compiled;
}

nlohmann::json SimulationCompiler::build_visual_scene(const nlohmann::json& scene) const {
  const auto validation = validate_scene(scene);
  if (!validation.value("valid", false)) {
    throw std::invalid_argument("scene validation failed: " + validation["errors"].dump());
  }

  nlohmann::json visual = {
      {"scene_id", scene.value("id", "")},
      {"backend", "threejs"},
      {"units", "m"},
      {"coordinate_system", "mujoco_xyz"},
      {"items", nlohmann::json::array()},
  };

  const auto append_item = [&visual](nlohmann::json item) {
    if (!item.contains("pos")) item["pos"] = nlohmann::json::array({0.0, 0.0, 0.0});
    if (!item.contains("quat")) item["quat"] = nlohmann::json::array({1.0, 0.0, 0.0, 0.0});
    visual["items"].push_back(std::move(item));
  };

  if (scene.contains("models") && scene["models"].is_array()) {
    for (const auto& model : scene["models"]) {
      const std::string id = model.value("id", "");
      if (id.empty()) continue;
      append_item(
          {{"id", id},
           {"source", "scene.models"},
           {"kind", "model_ref"},
           {"shape", "model_ref"},
           {"model_path", resolve_scene_path(scene, model.value("source", "")).string()},
           {"pos", numeric_array(model.value("pos", nlohmann::json::array()), {0, 0, 0}, 3)},
           {"quat", numeric_array(model.value("quat", nlohmann::json::array()), {1, 0, 0, 0}, 4)},
           {"size", nlohmann::json::array({0.18, 0.18, 0.18})}});
    }
  }

  if (scene.contains("objects") && scene["objects"].is_array()) {
    for (const auto& object : scene["objects"]) {
      const std::string id = object.value("id", "");
      const std::string type = object.value("type", "");
      if (id.empty()) continue;

      if (type == "obstacle.box") {
        append_item(
            {{"id", id},
             {"source", "scene.objects"},
             {"kind", "primitive"},
             {"shape", "box"},
             {"pos", numeric_array(object.value("pos", nlohmann::json::array()), {0, 0, 0}, 3)},
             {"quat",
              numeric_array(object.value("quat", nlohmann::json::array()), {1, 0, 0, 0}, 4)},
             {"size",
              numeric_array(object.value("size", nlohmann::json::array()), {0.1, 0.1, 0.1}, 3)},
             {"rgba", numeric_array(object.value("rgba", nlohmann::json::array()),
                                    {0.7, 0.2, 0.2, 1}, 4)}});
      } else if (type == "obstacle.sphere") {
        append_item(
            {{"id", id},
             {"source", "scene.objects"},
             {"kind", "primitive"},
             {"shape", "sphere"},
             {"pos", numeric_array(object.value("pos", nlohmann::json::array()), {0, 0, 0}, 3)},
             {"quat",
              numeric_array(object.value("quat", nlohmann::json::array()), {1, 0, 0, 0}, 4)},
             {"size", numeric_array(object.value("size", nlohmann::json::array()), {0.1}, 1)},
             {"rgba", numeric_array(object.value("rgba", nlohmann::json::array()),
                                    {0.2, 0.45, 0.9, 1}, 4)}});
      } else if (type == "obstacle.cylinder") {
        append_item(
            {{"id", id},
             {"source", "scene.objects"},
             {"kind", "primitive"},
             {"shape", "cylinder"},
             {"pos", numeric_array(object.value("pos", nlohmann::json::array()), {0, 0, 0}, 3)},
             {"quat",
              numeric_array(object.value("quat", nlohmann::json::array()), {1, 0, 0, 0}, 4)},
             {"size", numeric_array(object.value("size", nlohmann::json::array()), {0.1, 0.1}, 2)},
             {"rgba", numeric_array(object.value("rgba", nlohmann::json::array()),
                                    {0.1, 0.55, 0.35, 1}, 4)}});
      } else if (type == "obstacle.capsule") {
        append_item(
            {{"id", id},
             {"source", "scene.objects"},
             {"kind", "primitive"},
             {"shape", "capsule"},
             {"pos", numeric_array(object.value("pos", nlohmann::json::array()), {0, 0, 0}, 3)},
             {"quat",
              numeric_array(object.value("quat", nlohmann::json::array()), {1, 0, 0, 0}, 4)},
             {"size", numeric_array(object.value("size", nlohmann::json::array()), {0.1, 0.1}, 2)},
             {"rgba", numeric_array(object.value("rgba", nlohmann::json::array()),
                                    {0.55, 0.35, 0.85, 1}, 4)}});
      } else if (type == "obstacle.ellipsoid") {
        append_item(
            {{"id", id},
             {"source", "scene.objects"},
             {"kind", "primitive"},
             {"shape", "ellipsoid"},
             {"pos", numeric_array(object.value("pos", nlohmann::json::array()), {0, 0, 0}, 3)},
             {"quat",
              numeric_array(object.value("quat", nlohmann::json::array()), {1, 0, 0, 0}, 4)},
             {"size",
              numeric_array(object.value("size", nlohmann::json::array()), {0.1, 0.1, 0.1}, 3)},
             {"rgba", numeric_array(object.value("rgba", nlohmann::json::array()),
                                    {0.85, 0.55, 0.2, 1}, 4)}});
        // obstacle.mesh/obstacle.hfield have no client-computable geometry from
        // `size` alone (unlike the primitive shapes above) -- they only render
        // once resolved through the compiled model's visual snapshot, so this
        // scene-only preview intentionally emits no item for them.
      } else if (is_sensor_type(type)) {
        append_item(
            {{"id", id},
             {"source", "scene.objects"},
             {"kind", "sensor"},
             {"shape", "sensor"},
             {"sensor_type", type},
             {"site", object.value("site", "")},
             {"pos", numeric_array(object.value("pos", nlohmann::json::array()), {0, 0, 0}, 3)},
             {"rgba", nlohmann::json::array({0.98, 0.75, 0.14, 1.0})}});
      }
    }
  }

  if (scene.contains("sensors") && scene["sensors"].is_array()) {
    int index = 0;
    for (const auto& sensor : scene["sensors"]) {
      const std::string id = sensor.value("id", "");
      if (id.empty()) continue;
      append_item({{"id", id},
                   {"source", "scene.sensors"},
                   {"kind", "sensor"},
                   {"shape", "sensor"},
                   {"sensor_type", sensor.value("type", "")},
                   {"site", sensor.value("site", "")},
                   {"pos", numeric_array(sensor.value("pos", nlohmann::json::array()),
                                         {-0.42 + index * 0.12, 0, 0.18}, 3)},
                   {"rgba", nlohmann::json::array({0.98, 0.75, 0.14, 1.0})}});
      ++index;
    }
  }

  return visual;
}
nlohmann::json SimulationCompiler::build_instance_config(
    const nlohmann::json& request, const nlohmann::json& compiled_scene) const {
  if (!request.is_object()) throw std::invalid_argument("instance create data must be an object");

  const std::string scene_id = request.value("scene_id", "");
  if (scene_id.empty()) return request;
  if (!compiled_scene.is_object()) throw std::invalid_argument("compiled scene must be an object");
  if (compiled_scene.value("scene_id", "") != scene_id) {
    throw std::invalid_argument("compiled scene id mismatch");
  }

  nlohmann::json config = compiled_scene.value("initial_state", nlohmann::json::object());
  for (auto it = request.begin(); it != request.end(); ++it) {
    config[it.key()] = it.value();
  }
  config["scene_id"] = scene_id;
  config["compiled_model_id"] = compiled_scene.value("compiled_model_id", "");
  config["model_path"] = compiled_scene.value("model_path", "");
  return config;
}

SimulationCompiler::ModelPtr SimulationCompiler::get_compiled_model(
    const std::string& compiled_model_id) const {
  std::lock_guard lock{model_mutex_};
  auto it = models_.find(compiled_model_id);
  if (it == models_.end())
    throw std::out_of_range("compiled model not found: " + compiled_model_id);
  return touch_and_reclaim_locked(compiled_model_id, it->second.model);
}

std::pair<std::string, SimulationCompiler::ModelPtr> SimulationCompiler::get_or_load_model(
    const std::string& model_path) const {
  if (model_path.empty()) throw std::invalid_argument("missing 'model_path'");
  std::filesystem::path path{model_path};
  if (path.is_relative()) path = std::filesystem::current_path() / path;
  path = std::filesystem::weakly_canonical(path);
  if (!std::filesystem::exists(path))
    throw std::invalid_argument("model path does not exist: " + path.string());

  std::error_code ec;
  const auto timestamp = std::filesystem::last_write_time(path, ec);
  const std::string input
      = path.string() + (ec ? "" : std::to_string(timestamp.time_since_epoch().count()));
  const std::string model_id = "direct-" + simulation::sha256_string(input).substr(0, 32);

  {
    std::lock_guard lock{model_mutex_};
    auto it = models_.find(model_id);
    if (it != models_.end())
      return {model_id, touch_and_reclaim_locked(model_id, it->second.model)};
  }
  auto loaded = load_model(path);
  std::lock_guard lock{model_mutex_};
  auto model = touch_and_reclaim_locked(model_id, std::move(loaded));
  return {model_id, std::move(model)};
}

std::string SimulationCompiler::structural_model_id(const nlohmann::json& scene) {
  std::ostringstream input;
  input << structural_signature(scene).dump();
  const auto append_timestamp = [&input](const std::filesystem::path& path) {
    std::error_code ec;
    const auto timestamp = std::filesystem::last_write_time(path, ec);
    if (!ec) input << ':' << timestamp.time_since_epoch().count();
  };
  for (const auto& model : scene.value("models", nlohmann::json::array()))
    append_timestamp(resolve_scene_path(scene, model.value("source", "")));
  // assets[] (mesh/texture/hfield) embed the same resolve_scene_path-resolved
  // absolute path into the compiled <mesh file=".."/>/<texture file=".."/>/
  // <hfield file=".."/> attributes -- without this, overwriting a referenced
  // asset file's *content* (same path, so the scene JSON itself is unchanged)
  // would silently keep serving the stale cached model.
  for (const auto& asset : scene.value("assets", nlohmann::json::array())) {
    const std::string file = asset.value("file", "");
    if (!file.empty()) append_timestamp(resolve_scene_path(scene, file));
  }

  return scene.value("id", "scene") + '-' + simulation::sha256_string(input.str()).substr(0, 32);
}

SimulationCompiler::ModelPtr SimulationCompiler::touch_and_reclaim_locked(
    const std::string& model_id, ModelPtr model, std::filesystem::path compiled_path) const {
  auto& entry = models_[model_id];
  entry.model = std::move(model);
  entry.last_used = ++access_tick_;
  if (!compiled_path.empty()) entry.compiled_path = std::move(compiled_path);
  ModelPtr result = entry.model;

  // Evict least-recently-used entries that no live instance still references
  // (use_count of 1 means only the cache holds the model).
  while (models_.size() > max_cached_models_) {
    auto victim = models_.end();
    for (auto it = models_.begin(); it != models_.end(); ++it) {
      if (it->second.model.use_count() != 1) continue;
      if (victim == models_.end() || it->second.last_used < victim->second.last_used) victim = it;
    }
    if (victim == models_.end()) break;  // everything is still referenced
    models_.erase(victim);
  }
  return result;
}

size_t SimulationCompiler::cached_model_count() const {
  std::lock_guard lock{model_mutex_};
  return models_.size();
}

size_t SimulationCompiler::prune_unused_models() const {
  std::lock_guard lock{model_mutex_};
  const size_t before = models_.size();
  for (auto it = models_.begin(); it != models_.end();) {
    if (it->second.model.use_count() == 1)
      it = models_.erase(it);
    else
      ++it;
  }
  return before - models_.size();
}

SimulationCompiler::ModelPtr SimulationCompiler::load_model(const std::filesystem::path& path) {
  char error[2048] = {};
  std::lock_guard xml_lock{simulation_mujoco_xml_mutex()};
  mjModel* model = mj_loadXML(path.string().c_str(), nullptr, error, sizeof(error));
  if (!model) throw std::runtime_error(error[0] ? error : "failed to compile MuJoCo model");
  return ModelPtr(model, mj_deleteModel);
}

std::filesystem::path SimulationCompiler::default_output_dir() {
  return simulation_data_dir("cache/generated_scenes", "generated_scenes");
}

std::filesystem::path SimulationCompiler::default_export_dir() {
  return simulation_data_dir("exports", "scene_exports");
}

std::string SimulationCompiler::generate_scene_xml(
    const nlohmann::json& scene,
    const std::function<std::string(const nlohmann::json&, const std::filesystem::path&)>&
        model_file_ref) const {
  const auto physics = scene.value("physics", nlohmann::json::object());
  const auto environment = scene.value("environment", nlohmann::json::object());

  std::ostringstream doc;
  doc << "<mujoco model=\"" << xml_escape(scene.value("id", "scene")) << "\">\n";

  // ---- <compiler>：编译期指令（恒尝试生成，内部全 conditional，空则整段跳过） ----
  doc << generate_compiler_xml(physics);

  // ---- <option>：物理介质与求解器（属性 only，无子元素，恒生成） ----
  doc << "  <option timestep=\"" << physics.value("timestep", 0.002) << "\" gravity=\""
      << numeric_list(physics.value("gravity", nlohmann::json::array()), "0 0 -9.81", 3) << "\"";
  if (physics.contains("wind"))
    doc << " wind=\"" << numeric_list(physics["wind"], "0 0 0", 3) << "\"";
  if (physics.contains("magnetic"))
    doc << " magnetic=\"" << numeric_list(physics["magnetic"], "0 -0.5 0", 3) << "\"";
  if (physics.contains("density") && physics["density"].is_number())
    doc << " density=\"" << physics["density"].get<double>() << "\"";
  if (physics.contains("viscosity") && physics["viscosity"].is_number())
    doc << " viscosity=\"" << physics["viscosity"].get<double>() << "\"";
  if (const auto integrator = canonical_integrator(physics.value("integrator", "")))
    doc << " integrator=\"" << *integrator << "\"";
  if (const auto solver = canonical_solver(physics.value("solver", "")))
    doc << " solver=\"" << *solver << "\"";
  if (physics.contains("iterations") && physics["iterations"].is_number_integer())
    doc << " iterations=\"" << physics["iterations"].get<int>() << "\"";
  if (physics.contains("impratio") && physics["impratio"].is_number())
    doc << " impratio=\"" << physics["impratio"].get<double>() << "\"";
  if (physics.contains("tolerance") && physics["tolerance"].is_number())
    doc << " tolerance=\"" << physics["tolerance"].get<double>() << "\"";
  if (physics.contains("ls_tolerance") && physics["ls_tolerance"].is_number())
    doc << " ls_tolerance=\"" << physics["ls_tolerance"].get<double>() << "\"";
  if (physics.contains("noslip_tolerance") && physics["noslip_tolerance"].is_number())
    doc << " noslip_tolerance=\"" << physics["noslip_tolerance"].get<double>() << "\"";
  if (physics.contains("ccd_tolerance") && physics["ccd_tolerance"].is_number())
    doc << " ccd_tolerance=\"" << physics["ccd_tolerance"].get<double>() << "\"";
  if (physics.contains("sleep_tolerance") && physics["sleep_tolerance"].is_number())
    doc << " sleep_tolerance=\"" << physics["sleep_tolerance"].get<double>() << "\"";
  if (physics.contains("o_margin") && physics["o_margin"].is_number())
    doc << " o_margin=\"" << physics["o_margin"].get<double>() << "\"";
  if (physics.contains("o_solref"))
    doc << " o_solref=\"" << numeric_list(physics["o_solref"], "", 2) << "\"";
  if (physics.contains("o_solimp"))
    doc << " o_solimp=\"" << numeric_list(physics["o_solimp"], "", 5) << "\"";
  if (physics.contains("o_friction"))
    doc << " o_friction=\"" << numeric_list(physics["o_friction"], "", 5) << "\"";
  if (const auto cone = canonical_cone(physics.value("cone", "")))
    doc << " cone=\"" << *cone << "\"";
  if (const auto jacobian = canonical_jacobian(physics.value("jacobian", "")))
    doc << " jacobian=\"" << *jacobian << "\"";
  if (physics.contains("ls_iterations") && physics["ls_iterations"].is_number_integer())
    doc << " ls_iterations=\"" << physics["ls_iterations"].get<int>() << "\"";
  if (physics.contains("noslip_iterations") && physics["noslip_iterations"].is_number_integer())
    doc << " noslip_iterations=\"" << physics["noslip_iterations"].get<int>() << "\"";
  if (physics.contains("ccd_iterations") && physics["ccd_iterations"].is_number_integer())
    doc << " ccd_iterations=\"" << physics["ccd_iterations"].get<int>() << "\"";
  if (physics.contains("sdf_iterations") && physics["sdf_iterations"].is_number_integer())
    doc << " sdf_iterations=\"" << physics["sdf_iterations"].get<int>() << "\"";
  if (physics.contains("sdf_initpoints") && physics["sdf_initpoints"].is_number_integer())
    doc << " sdf_initpoints=\"" << physics["sdf_initpoints"].get<int>() << "\"";
  const auto flags = physics.value("flags", nlohmann::json::object());
  const bool has_flags = flags.is_object()
                         && ((flags.contains("disable") && !flags["disable"].empty())
                             || (flags.contains("enable") && !flags["enable"].empty()));
  if (!has_flags) {
    doc << "/>\n";
  } else {
    doc << ">\n    <flag";
    for (const auto& name : flags.value("disable", nlohmann::json::array()))
      if (name.is_string() && known_option_flags().count(name.get<std::string>()))
        doc << " " << name.get<std::string>() << "=\"disable\"";
    for (const auto& name : flags.value("enable", nlohmann::json::array()))
      if (name.is_string() && known_option_flags().count(name.get<std::string>()))
        doc << " " << name.get<std::string>() << "=\"enable\"";
    doc << "/>\n  </option>\n";
  }

  // <size>/<statistic> are attribute-only self-closing tags (no children);
  // <visual> only ever has child *sub-blocks* (global/quality/.../rgba), never
  // loose text, but still needs to sit strictly between size and statistic in
  // official order, which the generic sections loop below can't interleave
  // mid-vector -- so all three, like <compiler>/<option> above, are written
  // directly and each returns its own complete tag (or "" if nothing was set).
  doc << generate_size_xml(physics);
  doc << generate_visual_xml(environment);
  doc << generate_statistic_xml(environment);

  // 官方顶层子元素顺序（build/_deps/mujoco-src/src/xml/xml_native_writer.cc:927-945）：
  // compiler, option, size, visual, statistic, default, extension, custom, asset,
  // worldbody, contact, deformable, equality, tendon, actuator, sensor, keyframe。
  // compiler/option/size/visual/statistic 都是恒生成或自带空判断、且各自负责包好
  // 自己的完整标签，直接写在 sections 循环之前；从 default 开始都是"有真正子元素、
  // 空则整段跳过"，进 sections vector 统一按顺序处理 —— 这条列表就是新增任意 MJCF
  // 段落时唯一要接线的地方。generate_default_xml() 自己已经把每个根 class 包成了完整的
  // <default class="...">...</default>，这里的包装再套一层恰好就是 MJCF 要求的
  // "唯一顶层 <default>，内部允许多个具名 class 作为同级子块" 的结构。
  const std::vector<std::pair<std::string, std::string>> sections = {
      {"default", generate_default_xml(scene)},
      {"extension", generate_extension_xml(scene)},
      {"custom", generate_custom_xml(scene)},
      {"asset", generate_asset_xml(scene, environment, model_file_ref)},
      {"worldbody", generate_worldbody_xml(scene, environment)},
      {"contact", generate_contact_xml(scene)},
      {"deformable", generate_deformable_xml(scene)},
      {"equality", generate_equality_xml(scene)},
      {"tendon", generate_tendon_xml(scene)},
      {"actuator", generate_actuator_xml(scene)},
      {"sensor", generate_sensor_xml(scene)},
      {"keyframe", generate_keyframe_xml(scene)},
  };
  for (const auto& [tag, children] : sections) {
    if (children.empty() && tag != "worldbody") continue;
    doc << "  <" << tag << ">\n" << children << "  </" << tag << ">\n";
  }
  doc << "</mujoco>\n";
  return doc.str();
}

std::filesystem::path SimulationCompiler::write_compiled_mjcf(
    const nlohmann::json& scene, const std::string& compiled_model_id) const {
  const std::string xml = generate_scene_xml(
      scene,
      [](const nlohmann::json&, const std::filesystem::path& source) { return source.string(); });

  std::filesystem::create_directories(output_dir_);
  const auto output_path = output_dir_ / (compiled_model_id + ".compiled.xml");
  std::ofstream output(output_path);
  if (!output) throw std::runtime_error("failed to write compiled model: " + output_path.string());
  output << xml;
  output.close();
  return std::filesystem::weakly_canonical(output_path);
}

namespace {

  std::string sanitize_folder(const std::string& value) {
    std::string out;
    out.reserve(value.size());
    for (unsigned char c : value)
      out.push_back((std::isalnum(c) || c == '_' || c == '-' || c == '.') ? static_cast<char>(c)
                                                                          : '_');
    return out.empty() ? std::string{"model"} : out;
  }

  // First regular file under any root whose filename matches `basename`. Used to
  // locate an asset source when flattening discards the original directory.
  std::optional<std::filesystem::path> find_by_basename(
      const std::vector<std::filesystem::path>& roots, const std::string& basename) {
    namespace fs = std::filesystem;
    for (const auto& root : roots) {
      if (!fs::is_directory(root)) continue;
      for (auto it
           = fs::recursive_directory_iterator(root, fs::directory_options::skip_permission_denied);
           it != fs::recursive_directory_iterator(); ++it) {
        if (it->is_regular_file() && it->path().filename().string() == basename) return it->path();
      }
    }
    return std::nullopt;
  }

  std::string parse_xml_attr(const std::string& xml, const std::string& attr) {
    const std::string key = attr + "=\"";
    const auto pos = xml.find(key);
    if (pos == std::string::npos) return "";
    const auto start = pos + key.size();
    const auto end = xml.find('"', start);
    if (end == std::string::npos) return "";
    return xml.substr(start, end - start);
  }

}  // namespace

nlohmann::json SimulationCompiler::export_scene(const nlohmann::json& scene,
                                                const std::string& out_dir_text,
                                                bool flatten) const {
  namespace fs = std::filesystem;
  const std::string scene_id = scene.value("id", "scene");

  fs::path out_dir;
  if (out_dir_text.empty()) {
    out_dir = default_export_dir() / scene_id;
  } else {
    out_dir = fs::path(out_dir_text);
    if (out_dir.is_relative()) out_dir = fs::current_path() / out_dir;
  }
  out_dir = fs::weakly_canonical(out_dir);
  fs::create_directories(out_dir);
  std::error_code ec;

  nlohmann::json warnings = nlohmann::json::array();
  const fs::path model_file = out_dir / "scene.xml";

  if (!flatten) {
    // ----- modular <attach> bundle (MuJoCo 3.2+) -----
    // The skeleton is generated and every mechanism comes from a registered
    // asset package, so the bundle is always self-contained.

    const fs::path assets_dir = out_dir / "assets";
    fs::remove_all(assets_dir, ec);
    fs::create_directories(assets_dir);

    nlohmann::json bundled = nlohmann::json::array();
    int index = 0;
    auto resolver = [&](const nlohmann::json& asset, const fs::path& source) -> std::string {
      const std::string id = asset.value("id", asset.value("model_id", "model"));
      fs::path pkg;
      const std::string pkg_text = asset.value("package_dir", "");
      if (!pkg_text.empty() && fs::is_directory(pkg_text))
        pkg = fs::weakly_canonical(pkg_text);
      else
        pkg = source.parent_path();

      const std::string folder = std::to_string(index++) + "_" + sanitize_folder(id);
      const fs::path dest = assets_dir / folder;
      // Same caps registration enforces on this package (max_asset_bytes' default,
      // and the same file-count ceiling) -- export bundling must not be able to
      // copy something the registry itself would have rejected or skipped.
      constexpr uint64_t kMaxExportBytes = static_cast<uint64_t>(1) << 30;
      constexpr uint64_t kMaxExportFiles = 50000;
      simulation::copy_directory_tree(pkg, dest, kMaxExportBytes, kMaxExportFiles);
      const fs::path file_rel = fs::path("assets") / folder / fs::relative(source, pkg);
      bundled.push_back(
          {{"id", id}, {"file", file_rel.generic_string()}, {"package", pkg.string()}});
      return file_rel.generic_string();
    };

    const std::string xml = generate_scene_xml(scene, resolver);
    {
      std::ofstream output(model_file);
      if (!output)
        throw std::runtime_error("failed to write exported scene: " + model_file.string());
      output << xml;
    }

    char error[2048] = {};
    nlohmann::json sizes;
    {
      std::lock_guard xml_lock{simulation_mujoco_xml_mutex()};
      mjModel* model = mj_loadXML(model_file.string().c_str(), nullptr, error, sizeof(error));
      if (!model)
        throw std::runtime_error(std::string("exported scene failed to load standalone: ")
                                 + (error[0] ? error : "unknown error"));
      sizes = {{"nq", model->nq},
               {"nv", model->nv},
               {"nu", model->nu},
               {"nbody", model->nbody},
               {"nsensor", model->nsensor}};
      mj_deleteModel(model);
    }

    return {
        {"scene_id", scene_id},
        {"export_dir", out_dir.string()},
        {"model_file", model_file.string()},
        {"open_with", "simulate \"" + model_file.string() + "\""},
        {"mode", "attach"},
        {"models", bundled},
        {"sizes", sizes},
        {"warnings", warnings},
        {"self_contained", warnings.empty()},
        {"min_mujoco", "3.2.0"},
    };
  }

  // ----- flattened single-file MJCF (default; opens in MuJoCo 3.1+) -----
  // Compose with absolute model refs so the temporary document loads regardless
  // of the export location, then let MuJoCo inline everything via mj_saveLastXML.
  const std::string composed = generate_scene_xml(
      scene, [](const nlohmann::json&, const fs::path& source) { return source.string(); });
  const fs::path compose_path = out_dir / ".compose.xml";
  {
    std::ofstream output(compose_path);
    if (!output) throw std::runtime_error("failed to write compose file: " + compose_path.string());
    output << composed;
  }

  // Package roots to locate asset files that flattening reduces to bare names.
  std::vector<fs::path> roots;
  for (const auto& asset : scene.value("models", nlohmann::json::array())) {
    const std::string pkg_text = asset.value("package_dir", "");
    if (!pkg_text.empty() && fs::is_directory(pkg_text))
      roots.push_back(fs::weakly_canonical(pkg_text));
    else {
      const auto src = resolve_scene_path(scene, asset.value("source", ""));
      if (!src.empty()) roots.push_back(src.parent_path());
    }
  }

  char error[2048] = {};
  char save_error[2048] = {};
  nlohmann::json sizes;
  std::vector<std::pair<std::string, std::string>> refs;  // (ref, kind)
  bool saved = false;
  {
    std::lock_guard xml_lock{simulation_mujoco_xml_mutex()};
    mjModel* model = mj_loadXML(compose_path.string().c_str(), nullptr, error, sizeof(error));
    if (!model) {
      fs::remove(compose_path, ec);
      throw std::runtime_error(std::string("scene composition failed: ")
                               + (error[0] ? error : "unknown error"));
    }
    saved = mj_saveLastXML(model_file.string().c_str(), model, save_error, sizeof(save_error)) != 0;
    auto collect = [&](int count, const int* pathadr, const char* kind) {
      for (int i = 0; i < count; ++i) {
        if (pathadr && pathadr[i] >= 0) {
          std::string ref(model->paths + pathadr[i]);
          if (!ref.empty()) refs.emplace_back(ref, kind);
        }
      }
    };
    collect(model->nmesh, model->mesh_pathadr, "mesh");
    collect(model->ntex, model->tex_pathadr, "texture");
    collect(model->nhfield, model->hfield_pathadr, "hfield");
    collect(model->nskin, model->skin_pathadr, "skin");
    sizes = {{"nq", model->nq},
             {"nv", model->nv},
             {"nu", model->nu},
             {"nbody", model->nbody},
             {"nsensor", model->nsensor}};
    mj_deleteModel(model);
    mj_freeLastXML();
  }
  fs::remove(compose_path, ec);
  if (!saved)
    throw std::runtime_error(std::string("failed to flatten scene: ")
                             + (save_error[0] ? save_error : "unknown error"));

  // Copy referenced assets next to scene.xml, honouring any dir the saver wrote.
  std::string flat_text;
  {
    std::ifstream in(model_file);
    std::ostringstream ss;
    ss << in.rdbuf();
    flat_text = ss.str();
  }
  const std::string meshdir = parse_xml_attr(flat_text, "meshdir");
  const std::string texturedir = parse_xml_attr(flat_text, "texturedir");
  nlohmann::json assets_copied = nlohmann::json::array();
  std::set<std::string> seen;
  for (const auto& [ref, kind] : refs) {
    if (!seen.insert(kind + '\0' + ref).second) continue;
    const std::string basename = fs::path(ref).filename().string();
    const auto found = find_by_basename(roots, basename);
    if (!found) {
      warnings.push_back("asset not found for bundling: " + ref);
      continue;
    }
    std::string sub = kind == std::string("texture") ? texturedir : meshdir;
    fs::path target = out_dir;
    if (!sub.empty() && fs::path(sub).is_relative()) target /= sub;
    target /= ref;
    fs::create_directories(target.parent_path(), ec);
    fs::copy_file(*found, target, fs::copy_options::overwrite_existing, ec);
    if (ec) {
      warnings.push_back("failed to copy asset " + ref + ": " + ec.message());
      continue;
    }
    assets_copied.push_back({{"ref", ref}, {"kind", kind}, {"source", found->string()}});
  }

  // Verify the flattened bundle loads standalone with its copied assets.
  {
    char verify_error[2048] = {};
    std::lock_guard xml_lock{simulation_mujoco_xml_mutex()};
    mjModel* model
        = mj_loadXML(model_file.string().c_str(), nullptr, verify_error, sizeof(verify_error));
    if (!model)
      throw std::runtime_error(std::string("flattened scene failed to load standalone: ")
                               + (verify_error[0] ? verify_error : "unknown error"));
    mj_deleteModel(model);
  }

  return {
      {"scene_id", scene_id},
      {"export_dir", out_dir.string()},
      {"model_file", model_file.string()},
      {"open_with", "simulate \"" + model_file.string() + "\""},
      {"mode", "flatten"},
      {"assets", assets_copied},
      {"sizes", sizes},
      {"warnings", warnings},
      {"self_contained", warnings.empty()},
      {"min_mujoco", "3.1.0"},
  };
}
