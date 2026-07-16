#pragma once

#include <mujoco/mujoco.h>

#include <nlohmann/json.hpp>

// Editor-facing visual payload (model_counts / sites / geom items) shared by
// the instance-level `visual.model` route and `scene.compile`'s
// `include_visual` preview. `data` must already be forwarded (mj_forward) so
// geom/site world poses are valid. The result carries no instance state
// (id/status/time); callers merge those fields themselves.
nlohmann::json simulation_visual_json(const mjModel* model, const mjData* data,
                                      bool include_geometry);
