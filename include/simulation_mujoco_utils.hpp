#pragma once

#include <mutex>

inline std::mutex& simulation_mujoco_xml_mutex() {
  static std::mutex mutex;
  return mutex;
}
