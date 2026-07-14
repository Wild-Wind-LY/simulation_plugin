#pragma once

#include <filesystem>
#include <memory>
#include <mutex>
#include <nlohmann/json.hpp>
#include <string>
#include <thread>
#include <unordered_map>

#include "simulation_instance_manager.hpp"

class SimulationRecordManager {
public:
  explicit SimulationRecordManager(std::filesystem::path output_dir = default_output_dir());
  ~SimulationRecordManager();

  nlohmann::json start(const nlohmann::json& data, SimulationInstanceManager& instances);
  nlohmann::json stop(const nlohmann::json& data);
  nlohmann::json list() const;
  nlohmann::json info(const nlohmann::json& data) const;
  nlohmann::json remove(const nlohmann::json& data);
  void stop_all() noexcept;

private:
  struct Session {
    std::string id;
    std::string instance_id;
    std::filesystem::path file_path;
    double sample_hz{10.0};
    uint64_t sample_count{0};
    int64_t started_ms{0};
    int64_t ended_ms{0};
    bool stop_requested{false};
    bool running{false};
    mutable std::mutex mutex;
    std::thread worker;
  };

  static std::filesystem::path default_output_dir();
  static std::string require_id(const nlohmann::json& data);
  static int64_t now_ms();
  static double clamp_sample_hz(double value);
  static nlohmann::json session_info_locked(const Session& session);
  static void worker_loop(std::shared_ptr<Session> session, SimulationInstanceManager& instances);

  std::shared_ptr<Session> find_or_throw(const std::string& id) const;
  nlohmann::json stop_session(const std::shared_ptr<Session>& session);

  std::filesystem::path output_dir_;
  mutable std::mutex mutex_;
  std::unordered_map<std::string, std::shared_ptr<Session>> sessions_;
  uint64_t next_id_{1};
};
