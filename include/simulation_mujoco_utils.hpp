#pragma once

#include <algorithm>
#include <cctype>
#include <filesystem>
#include <fstream>
#include <mutex>
#include <string>

inline std::mutex& simulation_mujoco_xml_mutex() {
  static std::mutex mutex;
  return mutex;
}

// 模型格式识别：内容嗅探优先，扩展名兜底，注册与校验共用同一套判断。
// 嗅探压过扩展名，这样叫 .xml 的 URDF（或叫 .urdf 的 MJCF）也能识别正确。
inline std::string simulation_detect_model_format(const std::filesystem::path& path) {
  auto extension = path.extension().string();
  std::transform(extension.begin(), extension.end(), extension.begin(),
                 [](unsigned char c) { return static_cast<char>(std::tolower(c)); });

  std::ifstream input(path, std::ios::binary);
  if (input) {
    std::string head(4096, '\0');
    input.read(head.data(), static_cast<std::streamsize>(head.size()));
    head.resize(static_cast<size_t>(input.gcount()));
    if (head.find("<robot") != std::string::npos) return "urdf";
    if (head.find("<mujoco") != std::string::npos) return "mjcf";
  }

  if (extension == ".urdf") return "urdf";
  if (extension == ".xml" || extension == ".mjcf") return "mjcf";
  return extension.empty() ? "unknown" : extension.substr(1);
}
