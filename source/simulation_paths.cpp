#include "simulation_paths.hpp"

#include <cstdlib>
#include <stdexcept>
#include <system_error>

namespace {

  std::filesystem::path normalize(const std::filesystem::path& path) {
    std::error_code ec;
    auto canonical = std::filesystem::weakly_canonical(path, ec);
    return ec ? path : canonical;
  }

  bool is_ignored_dir_name(const std::string& name) {
    return name == ".git" || name == ".svn" || name == ".hg";
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

namespace simulation {

  void copy_directory_tree(const std::filesystem::path& from, const std::filesystem::path& to,
                           uint64_t max_bytes, uint64_t max_files) {
    namespace fs = std::filesystem;
    const fs::path root = fs::weakly_canonical(from);
    if (!fs::is_directory(root))
      throw std::invalid_argument("source is not a directory: " + root.string());

    if (max_bytes > 0 || max_files > 0) {
      uint64_t total_bytes = 0;
      uint64_t file_count = 0;
      for (auto it
           = fs::recursive_directory_iterator(root, fs::directory_options::skip_permission_denied);
           it != fs::recursive_directory_iterator(); ++it) {
        const auto& entry = *it;
        if (entry.is_directory() && is_ignored_dir_name(entry.path().filename().string())) {
          it.disable_recursion_pending();
          continue;
        }
        if (entry.is_symlink()) continue;
        if (entry.is_regular_file()) {
          std::error_code ec;
          total_bytes += static_cast<uint64_t>(entry.file_size(ec));
          if (max_files > 0 && ++file_count > max_files)
            throw std::invalid_argument(
                "directory exceeds file-count limit; narrow the source directory: "
                + root.string());
          if (max_bytes > 0 && total_bytes > max_bytes)
            throw std::invalid_argument(
                "directory exceeds size limit; narrow the source directory or raise the limit: "
                + root.string());
        }
      }
    }

    fs::create_directories(to);
    for (auto it
         = fs::recursive_directory_iterator(root, fs::directory_options::skip_permission_denied);
         it != fs::recursive_directory_iterator(); ++it) {
      const auto& entry = *it;
      if (entry.is_directory() && is_ignored_dir_name(entry.path().filename().string())) {
        it.disable_recursion_pending();
        continue;
      }
      if (entry.is_symlink()) continue;
      const auto rel = fs::relative(entry.path(), root);
      const auto dest = to / rel;
      std::error_code ec;
      if (entry.is_directory()) {
        fs::create_directories(dest, ec);
      } else if (entry.is_regular_file()) {
        fs::create_directories(dest.parent_path(), ec);
        fs::copy_file(entry.path(), dest, fs::copy_options::overwrite_existing, ec);
        if (ec)
          throw std::runtime_error("failed to copy " + entry.path().string() + ": " + ec.message());
      }
    }
  }

}  // namespace simulation
