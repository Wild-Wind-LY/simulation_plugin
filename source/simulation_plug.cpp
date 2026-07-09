#include "simulation_plug.hpp"

bool SimulationPlug::on_init(IPluginContext& ctx) noexcept {
  // 设置插件全局日志
  LOGX_GLOBAL_CLASS(PLUGIN_LOG_NAME)::Construct();
  G_LOG()->set(logger());

  LOG_INFO("[{}] Initialized!", TAG);
  return true;
}

bool SimulationPlug::on_start() noexcept {
  LOG_INFO("[{}] Started!", TAG);
  return true;
}

void SimulationPlug::on_stop() noexcept { LOG_INFO("[{}] Stopped!", TAG); }

void SimulationPlug::on_unload() noexcept {
  LOG_INFO("[{}] onUnload called!", TAG);

  LOGX_GLOBAL_CLASS(PLUGIN_LOG_NAME)::Destruct();
}

PLUGIN_DECLARE(SimulationPlug, "SimulationPlugin", "1.0", "liuyan", "Simulation Plugin")
