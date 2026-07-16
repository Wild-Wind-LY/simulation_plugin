#include "simulation_record_manager.hpp"

#include <algorithm>
#include <chrono>
#include <fstream>
#include <stdexcept>
#include <utility>

#include "simulation_paths.hpp"

SimulationRecordManager::SimulationRecordManager(std::filesystem::path output_dir)
    : output_dir_(std::move(output_dir)) {}

SimulationRecordManager::~SimulationRecordManager() { stop_all(); }

nlohmann::json SimulationRecordManager::start(const nlohmann::json& data,
                                              SimulationInstanceManager& instances) {
  const std::string instance_id = data.value("instance_id", data.value("id", ""));
  if (instance_id.empty()) throw std::invalid_argument("missing 'instance_id'");

  std::string id = data.value("record_id", data.value("id", ""));
  auto session = std::make_shared<Session>();
  {
    std::lock_guard lock{mutex_};
    if (id.empty() || id == instance_id) id = "record-" + std::to_string(next_id_++);
    if (sessions_.find(id) != sessions_.end())
      throw std::invalid_argument("record already exists: " + id);
    session->id = id;
    session->instance_id = instance_id;
    session->sample_hz = clamp_sample_hz(data.value("sample_hz", 10.0));
    std::filesystem::create_directories(output_dir_);
    session->file_path = output_dir_ / (id + ".jsonl");
    session->started_ms = now_ms();
    session->running = true;
    sessions_[id] = session;
  }

  session->worker = std::thread([session, &instances] { worker_loop(session, instances); });
  std::lock_guard session_lock{session->mutex};
  return session_info_locked(*session);
}

nlohmann::json SimulationRecordManager::stop(const nlohmann::json& data) {
  return stop_session(find_or_throw(require_id(data)));
}

nlohmann::json SimulationRecordManager::list() const {
  std::vector<std::shared_ptr<Session>> sessions;
  {
    std::lock_guard lock{mutex_};
    sessions.reserve(sessions_.size());
    for (const auto& [id, session] : sessions_) {
      (void)id;
      sessions.push_back(session);
    }
  }

  nlohmann::json out = nlohmann::json::array();
  for (const auto& session : sessions) {
    std::lock_guard session_lock{session->mutex};
    out.push_back(session_info_locked(*session));
  }
  return out;
}

nlohmann::json SimulationRecordManager::info(const nlohmann::json& data) const {
  auto session = find_or_throw(require_id(data));
  std::lock_guard lock{session->mutex};
  return session_info_locked(*session);
}

nlohmann::json SimulationRecordManager::remove(const nlohmann::json& data) {
  const auto id = require_id(data);
  std::shared_ptr<Session> session;
  {
    std::lock_guard lock{mutex_};
    auto it = sessions_.find(id);
    if (it == sessions_.end()) throw std::out_of_range("record not found: " + id);
    session = it->second;
    sessions_.erase(it);
  }

  auto result = stop_session(session);
  std::error_code ec;
  const bool removed_file = std::filesystem::remove(session->file_path, ec);
  result["removed"] = true;
  result["file_removed"] = removed_file;
  if (ec) result["file_remove_error"] = ec.message();
  return result;
}

void SimulationRecordManager::stop_all() noexcept {
  try {
    std::vector<std::shared_ptr<Session>> sessions;
    {
      std::lock_guard lock{mutex_};
      sessions.reserve(sessions_.size());
      for (const auto& [id, session] : sessions_) {
        (void)id;
        sessions.push_back(session);
      }
    }
    for (const auto& session : sessions) stop_session(session);
  } catch (...) {
  }
}

std::filesystem::path SimulationRecordManager::default_output_dir() {
  return simulation_data_dir("records", "records");
}

std::string SimulationRecordManager::require_id(const nlohmann::json& data) {
  const std::string id = data.value("record_id", data.value("id", ""));
  if (id.empty()) throw std::invalid_argument("missing 'record_id' or 'id'");
  return id;
}

int64_t SimulationRecordManager::now_ms() {
  using clock = std::chrono::system_clock;
  return std::chrono::duration_cast<std::chrono::milliseconds>(clock::now().time_since_epoch())
      .count();
}

double SimulationRecordManager::clamp_sample_hz(double value) {
  if (value <= 0.0) return 10.0;
  return std::clamp(value, 1.0, 1000.0);
}

nlohmann::json SimulationRecordManager::session_info_locked(const Session& session) {
  return {
      {"id", session.id},
      {"instance_id", session.instance_id},
      {"status", session.running ? "running" : "stopped"},
      {"sample_hz", session.sample_hz},
      {"sample_count", session.sample_count},
      {"started_ms", session.started_ms},
      {"ended_ms", session.ended_ms},
      {"file_path", session.file_path.string()},
  };
}

void SimulationRecordManager::worker_loop(std::shared_ptr<Session> session,
                                          SimulationInstanceManager& instances) {
  std::ofstream output(session->file_path);
  const auto interval = std::chrono::duration<double>(1.0 / session->sample_hz);

  while (true) {
    {
      std::lock_guard lock{session->mutex};
      if (session->stop_requested) break;
    }

    nlohmann::json frame;
    try {
      frame = instances.state({{"id", session->instance_id}});
      frame["record_id"] = session->id;
      frame["sample_index"] = session->sample_count;
      frame["wall_time_ms"] = now_ms();
      output << frame.dump() << '\n';
      output.flush();
      {
        std::lock_guard lock{session->mutex};
        ++session->sample_count;
      }
    } catch (const std::exception& e) {
      frame = {
          {"record_id", session->id},
          {"sample_index", session->sample_count},
          {"wall_time_ms", now_ms()},
          {"error", e.what()},
      };
      output << frame.dump() << '\n';
      output.flush();
      std::lock_guard lock{session->mutex};
      ++session->sample_count;
      session->stop_requested = true;
    }

    std::this_thread::sleep_for(interval);
  }

  std::lock_guard lock{session->mutex};
  session->running = false;
  session->ended_ms = now_ms();
}

std::shared_ptr<SimulationRecordManager::Session> SimulationRecordManager::find_or_throw(
    const std::string& id) const {
  std::lock_guard lock{mutex_};
  auto it = sessions_.find(id);
  if (it == sessions_.end()) throw std::out_of_range("record not found: " + id);
  return it->second;
}

nlohmann::json SimulationRecordManager::stop_session(const std::shared_ptr<Session>& session) {
  {
    std::lock_guard lock{session->mutex};
    session->stop_requested = true;
  }
  if (session->worker.joinable()) session->worker.join();

  std::lock_guard lock{session->mutex};
  session->running = false;
  if (session->ended_ms == 0) session->ended_ms = now_ms();
  return session_info_locked(*session);
}
