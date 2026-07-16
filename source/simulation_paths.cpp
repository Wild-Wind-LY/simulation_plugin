#include "simulation_paths.hpp"

#include <cstdlib>
#include <system_error>

namespace {

  std::filesystem::path normalize(const std::filesystem::path& path) {
    std::error_code ec;
    auto canonical = std::filesystem::weakly_canonical(path, ec);
    return ec ? path : canonical;
  }

}  // namespace

std::filesystem::path simulation_data_root() {
  if (const char* env = std::getenv("SIMULATION_DATA_DIR"); env && *env) {
    std::filesystem::path root{env};
    if (root.is_relative()) root = std::filesystem::current_path() / root;
    return normalize(root);
  }
  return normalize(std::filesystem::current_path() / "simulation_data");
}

std::filesystem::path simulation_data_dir(const std::string& name, const std::string& legacy_name) {
  const auto target = simulation_data_root() / std::filesystem::path(name);
  std::error_code ec;
  if (std::filesystem::exists(target, ec)) return target;

  // 旧布局迁移：<cwd>/build/<legacy_name>（历史上数据错放在 build 下）
  const auto legacy = normalize(std::filesystem::current_path() / "build" / legacy_name);
  if (std::filesystem::exists(legacy, ec)) {
    std::filesystem::create_directories(target.parent_path(), ec);
    std::error_code rename_ec;
    std::filesystem::rename(legacy, target, rename_ec);
    if (!rename_ec) return target;
    // 并发迁移竞争：另一线程已完成
    if (std::filesystem::exists(target, ec)) return target;
    // 迁移失败（跨设备/权限）：沿用旧目录，绝不丢数据
    return legacy;
  }
  return target;
}
