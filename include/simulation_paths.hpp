#pragma once

#include <cstdint>
#include <filesystem>
#include <string>

// 插件持久数据根目录。默认 <cwd>/simulation_data；设置环境变量
// SIMULATION_DATA_DIR（绝对路径，或相对网关工作目录）可整体重定位。
std::filesystem::path simulation_data_root();

// 解析数据子目录 <root>/<name>（name 可含层级，如 "cache/generated_scenes"）。
// 若目标不存在而旧布局 <cwd>/build/<legacy_name> 存在，自动 rename 迁移；
// 迁移失败（跨设备等）时沿用旧目录，保证不丢数据。目录本身不在此创建，
// 由调用方按需 create_directories。
std::filesystem::path simulation_data_dir(const std::string& name, const std::string& legacy_name);

namespace simulation {

  // Recursively copies `from` into `to`, preserving relative structure. Skips
  // symlinks and VCS directories (.git/.svn/.hg). If `max_bytes` and/or
  // `max_files` are non-zero, pre-scans `from` first and throws
  // std::invalid_argument without copying anything if either cap is exceeded.
  // Shared by the model registry's asset-package snapshot and scene export's
  // asset bundling so both enforce the same caps/skip rules instead of each
  // reimplementing (and potentially drifting from) this independently.
  void copy_directory_tree(const std::filesystem::path& from, const std::filesystem::path& to,
                           uint64_t max_bytes = 0, uint64_t max_files = 0);

}  // namespace simulation
