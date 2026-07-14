# Simulation Plugin Architecture

The plugin uses four primary concepts with explicit ownership boundaries:

```text
Model Asset -> Scene Configuration -> Compiled Model -> Runtime Instance
```

## 1. Model Asset

A model asset is one reusable MJCF file describing a robot, tool, workpiece, fixture, or other mechanism. It owns its body tree, joints, actuators, sensors, defaults, and referenced mesh/texture assets. It does not own placement in a workcell.

Assets are registered by stable `model_id + version`. Registry metadata is persisted in `build/model_assets/registry.json`. URDF assets are validated and converted to generated MJCF during registration so they can participate in MuJoCo `asset/model + attach` composition.

### Registration semantics

Registration is transactional and produces a self-contained, portable, integrity-checked asset:

- **Self-contained packaging.** By default (`copy_assets: true`) the model and its neighbouring asset directory are snapshotted into `build/model_assets/<id>/<version>/`, preserving relative structure so `meshdir`, `include`, and mesh/texture references keep resolving. A registered version therefore survives moving or deleting the original source files. The snapshot root defaults to the model file's parent directory; pass `asset_root` to widen or narrow it (it must be an ancestor of the model file), and it is bounded by size (`max_asset_bytes`, default 1 GiB) and file-count caps to prevent accidentally copying an unrelated tree. Set `copy_assets: false` to keep referencing the original file in place (not portable; intended for development).
- **Portability.** The manifest stores paths relative to the storage root and hydrates them back to absolute paths on read, so a registry directory can be relocated between working directories or machines.
- **Integrity.** SHA-256 digests are recorded at registration: the effective (compiled) model file, plus — for packaged assets — a whole-package digest covering every snapshotted file (meshes, textures, includes), so tampering with any asset is detectable, not just the top-level model. `model.verify` recomputes and compares (reporting `scope: package` or `model-file` on mismatch); `model.info` embeds the same status under `integrity`.
- **Robot metadata.** The effective model is compiled during registration; the resulting entry carries `sizes`, a `controllable` flag (`nu > 0`), a joints/actuators/sensors `summary`, and `warnings` (e.g. no actuators, no joints). Pass `require_actuators: true` to reject a model that has no actuators.
- **Format detection.** The format is determined by content sniffing (`<robot>` → URDF, `<mujoco>` → MJCF) with the file extension as a fallback, so a URDF named `.xml` is classified correctly.
- **Safety.** `id` and `version` are restricted to letters, digits, `.`, `_`, `-` and may not be `.` or `..` (path-traversal guard). A corrupt `registry.json` is backed up to `registry.json.corrupt-<ts>` and the registry starts empty rather than failing plugin startup.

RPC modules:

- `model.register`: register a versioned MJCF/URDF asset; use `replace: true` to replace a version. Accepts `copy_assets`, `asset_root`, `require_actuators`, and `max_asset_bytes`.
- `model.list`, `model.info`, `model.remove`: manage registered versions. Omitting `version` resolves the latest by semantic-version ordering (`1.10 > 1.9`). `model.remove` deletes the version's package directory.
- `model.verify`: re-check that a registered version's effective file still matches the digest recorded at registration.
- `model.cache_prune`: release cached compiled models which have no live instance references.
- `model.validate`: parse and compile an arbitrary model file temporarily.
- `model.inspect`: inspect the model used by an existing instance.

## 2. Scene Configuration

A scene is the stable editor/API contract. It places reusable model assets and defines environment-specific primitives, sensors, initial state, runtime rates, and collision policy.

```json
{
  "id": "cell_001",
  "models": [
    {
      "id": "robot_1",
      "model_id": "ur5",
      "version": "1.0.0",
      "pos": [0, 0, 0],
      "quat": [1, 0, 0, 0],
      "prefix": "robot_1_"
    },
    {
      "id": "workpiece_1",
      "model_id": "workpiece",
      "version": "2",
      "pos": [0.5, 0, 0.2]
    }
  ],
  "objects": [
    {"id": "table", "type": "obstacle.box", "pos": [0, 0, 0.4], "size": [0.8, 0.8, 0.4]}
  ],
  "contacts": {
    "excludes": [
      {"body1": "robot_1_base", "body2": "table"}
    ]
  },
  "defaults": {"step_hz": 100, "publish_hz": 10, "qpos": [], "qvel": [], "ctrl": []}
}
```

`model_path` remains optional for compatibility with a legacy complete base MJCF. New scenes should use registered `model_id + version`; direct `source` remains supported for development. Omitting version resolves the latest registered version, while production scenes should pin it explicitly.

RPC modules: `scene.load`, `scene.unload`, `scene.update`, `scene.apply`, `scene.list`, `scene.info`, `scene.validate`, `scene.diff`, `scene.compile`, `scene.export`.

Structural changes are `model_path`, `models`, `objects`, `sensors`, and `contacts`; they require recompilation and instance recreation. `defaults` changes can be applied at runtime.

## 3. Compiled Model

`scene.compile` performs the complete compilation pipeline:

1. Validate the backend-neutral scene JSON.
2. Generate a complete MJCF document.
3. Resolve each `model_id + version`, add its effective MJCF as an `asset/model`, and place it with `frame/attach`.
4. Add primitive bodies, sensors, and `contact/exclude` rules.
5. Call `mj_loadXML`, which parses and compiles the document into `mjModel`.
6. Cache the immutable `mjModel` by a structural model id.

The result contains `compiled_model_id`, generated model path, model sizes, initial state, and mapping tables. Runtime-only defaults do not change the structural model id.

The structural model id is a content-addressed SHA-256 of the structural signature plus each source file's timestamp, so distinct scenes cannot collide onto the same cached `mjModel`, and editing a source file transparently forces a recompile.

Legacy `objects` entries of type `model.include` remain supported for body-level MJCF fragments, but new full model assets should use the `models` array.

### Scene export

`scene.export` emits a **self-contained, relocatable MJCF bundle** that opens directly in MuJoCo `simulate` (or any MJCF tool) on any machine. The export is always verified by loading the written `scene.xml` standalone before returning, and the result includes `model_file`, an `open_with` (`simulate "<path>"`) hint, model sizes, `mode`, `min_mujoco`, `self_contained`, and any `warnings`. `out_dir` defaults to `build/scene_exports/<scene_id>`.

Two `mode`s are available via the `flatten` flag:

- **`flatten: true` (default) — single flattened file, opens in MuJoCo 3.1+.** The scene is composed and then written with `mj_saveLastXML`, which inlines every body and drops the `<model>`/`<attach>` composition syntax. Only the meshes/textures actually referenced by the model are copied next to `scene.xml`, so the bundle is minimal:

  ```text
  <out_dir>/scene.xml   flattened MJCF (no <attach>)
  <out_dir>/<mesh files> only the referenced assets, beside scene.xml
  ```

  This is the portable, widely-compatible form. `<attach>` and `<model>` assets were only added in MuJoCo 3.2, so older `simulate` builds report an XML parse error on the modular form — the flattened file avoids that.

- **`flatten: false` — modular `<attach>` bundle, requires MuJoCo 3.2+.** Each registered model's package is copied in full under `assets/<i>_<id>/`, and `scene.xml` references them with `<model file="assets/…">` + `<attach>`. This preserves the modular structure but only loads in MuJoCo 3.2 or newer.

In both modes a legacy base `model_path` scene or a `model.include` object embeds external content that is not relocated (reported under `warnings`). The modular mode additionally carries whatever the registration snapshot root captured, so for a clean bundle register production models against their own description-package directory (or a tight `asset_root`); the flattened mode is unaffected because it copies only referenced assets.

## 4. Runtime Instance

An instance owns one `mjData`, its lifecycle state, initial qpos/qvel/ctrl, stepping thread, and publication rate. It holds shared ownership of an immutable compiled `mjModel`. Both scene-based and direct `model_path` creation use the same timestamp-aware model cache. Cache entries remain stable across compile/create transactions and can be released explicitly with `model.cache_prune`.

The compiled-model cache is bounded (default 64 entries): when it grows past capacity it evicts the least-recently-used entries that no live instance still references, so a long-running server does not accumulate compiled models without bound. Models held by an instance are never evicted.

Direct `model_path` creation is a development convenience that bypasses the registry: it has no packaging, integrity digest, or version pinning. Production scenes should create instances from registered `model_id + version` so all of those guarantees apply.

```text
Compiled model (shared mjModel)
  +-- instance A (mjData A)
  +-- instance B (mjData B)
  +-- instance C (mjData C)
```

This is the MuJoCo data-parallel model: topology and constant parameters are shared, while position, velocity, controls, contacts, solver workspace, sensor values, and plugin state remain independent in each `mjData`.

RPC modules: `instance.create`, `instance.recreate_from_scene`, `instance.start`, `instance.pause`, `instance.stop`, `instance.step`, `instance.reset`, `instance.apply_runtime`, `instance.state`, `instance.metadata`, `instance.list`, `instance.destroy`.

State topic: `simulation.instance.<id>.state`.

## Supporting Layers

### Control Adapter

`control.joint_state`, `control.sensor_state`, and `control.write_ctrl` expose stable named/indexed access. Planning, IK, `movej`, and `movel` remain outside the simulation runtime.

### Task Layer

`task.create`, `task.remove`, `task.list`, `task.info`, and `task.run` implement synchronous low-level sequences using `step`, `set_ctrl`, `reset`, `pause`, and `stop`.

### Record Layer

`record.start`, `record.stop`, `record.list`, `record.info`, and `record.remove` sample instance state into JSONL without accessing MuJoCo objects directly.

### Visualization

`visual.scene` returns the editor-side scene representation. `visual.model` returns geometry derived from the compiled MuJoCo model and current instance state.

## End-to-End Flow

```text
Model files
  -> Scene JSON places models and defines contacts/defaults
  -> scene.compile generates MJCF and caches mjModel
  -> instance.create allocates an independent mjData
  -> control/task APIs modify the instance
  -> state/record APIs expose independent results
```


## Compiled model visualization

`visual.model` reads geometry from the instance's compiled `mjModel` and world transforms
from its locked `mjData`.

- The default `include_geometry: true` response includes primitive sizes, mesh vertices and
  triangle faces, height-field samples, material RGBA, and current `geom_xpos/geom_xmat`.
- `include_geometry: false` omits static mesh and height-field payloads and returns only
  current geom transforms plus instance time/status. This is the real-time preview path.
- The browser loads full geometry once after instance creation, keys render objects by
  `geom_id`, and applies lightweight transform updates without rebuilding the scene.

Scene-only preview remains a configuration fallback before an instance exists. Once an
instance is created, the preview automatically switches to the compiled-model representation.

