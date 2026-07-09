#pragma once

#include <string>

#include "plugin_core/sdk/plugin_logx.hpp"
#include "plugin_core/sdk/plugin_sdk.hpp"

class SimulationPlug final : public PluginBase {
public:
  explicit SimulationPlug() = default;
  ~SimulationPlug() override = default;

private:
  bool on_init(IPluginContext& ctx) noexcept override;
  bool on_start() noexcept override;
  void on_stop() noexcept override;
  void on_unload() noexcept override;

private:
  const std::string TAG = "仿真插件";
};
