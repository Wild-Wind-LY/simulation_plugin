#include <iostream>
#include <memory>
#include <nlohmann/json.hpp>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include "simulation_plug.hpp"

#define CHECK(condition)                                                                    \
  do {                                                                                      \
    if (!(condition)) throw std::runtime_error(std::string("check failed: ") + #condition); \
  } while (false)

namespace {
  struct FakeLifetime final : IPluginLifetime {
    PluginLifetimeHandle self() const noexcept override { return 1; }
    bool is_stopping(PluginLifetimeHandle) const noexcept override { return false; }
    const char* name(PluginLifetimeHandle) const noexcept override { return "simulation-test"; }
  };

  struct FakeLogger final : ILogger {
    std::vector<std::string> warnings;

    void log(Level level, const char* msg, size_t len, const Source*) noexcept override {
      if (level >= Level::Warn) warnings.emplace_back(msg, len);
    }
    bool enabled(Level) const noexcept override { return true; }
  };

  struct FakePubSub final : IPubSub {
    std::vector<std::pair<std::string, std::string>> emitted;

    bool emit(const char* topic, const void* data, size_t size) override {
      emitted.emplace_back(topic ? topic : "", std::string(static_cast<const char*>(data), size));
      return true;
    }
    bool subscribe(const char*, PluginLifetimeHandle,
                   const std::shared_ptr<IEventHandler>&) override {
      return true;
    }
    void unsubscribe(const char*, const std::shared_ptr<IEventHandler>&) override {}
    void unsubscribe_owner(PluginLifetimeHandle) override {}
  };

  struct FakeReqRep final : IReqRep {
    std::shared_ptr<IRequest> request(const char*, const void*, size_t) override { return {}; }
    bool register_handler(const char*, PluginLifetimeHandle,
                          const std::shared_ptr<IReqHandler>&) override {
      return true;
    }
    void unregister_handler(const char*, const std::shared_ptr<IReqHandler>&) override {}
    void unregister_owner(PluginLifetimeHandle) override {}
  };

  struct FakeRuntime final : IPluginRuntimeCap {
    bool add_cleanup(std::function<void()>) noexcept override { return true; }
    bool post_task(Task, CancelHandler) noexcept override { return true; }
    bool start_managed_worker(const char*, Task) noexcept override { return true; }
    PluginWorkerHandle start_worker_handle(const char*, Task) noexcept override { return {}; }
    bool start_managed_timer(std::chrono::milliseconds, Task, bool) noexcept override {
      return true;
    }
  };

  struct FakeHttp final : IPluginHttpCap {
    bool registered{false};

    bool register_endpoint(const char*, PluginLifetimeHandle, Handler,
                           PluginHttpEndpointOptions) noexcept override {
      registered = true;
      return true;
    }
    void unregister_endpoint(const char*, PluginLifetimeHandle) noexcept override {}
    void unregister_owner(PluginLifetimeHandle) noexcept override {}
  };

  struct FakeWebSocket final : IPluginWebSocketCap {
    struct Sent {
      std::string session;
      std::string topic;
      std::string payload;
      bool latest{false};
    };

    std::string endpoint;
    MessageHandler on_message;
    SessionHandler on_open;
    SessionHandler on_close;
    PluginWsEndpointOptions options;
    PluginWsSendResult send_result{PluginWsSendResult::Queued};
    std::vector<Sent> sent;

    bool register_endpoint(const char* value, PluginLifetimeHandle, MessageHandler message,
                           SessionHandler open, SessionHandler close,
                           PluginWsEndpointOptions endpoint_options) noexcept override {
      endpoint = value ? value : "";
      on_message = std::move(message);
      on_open = std::move(open);
      on_close = std::move(close);
      options = endpoint_options;
      return true;
    }
    void unregister_endpoint(const char*) noexcept override {}

    PluginWsSendResult record(const char* session, const char* topic, const void* data, size_t size,
                              bool latest) {
      if (send_result == PluginWsSendResult::Queued) {
        sent.push_back({session ? session : "", topic ? topic : "",
                        std::string(static_cast<const char*>(data), size), latest});
      }
      return send_result;
    }
    PluginWsSendResult send_text(const char* session, const void* data,
                                 size_t size) noexcept override {
      return record(session, nullptr, data, size, false);
    }
    PluginWsSendResult send_binary(const char* session, const void* data,
                                   size_t size) noexcept override {
      return record(session, nullptr, data, size, false);
    }
    PluginWsSendResult send_latest_text(const char* session, const char* topic, const void* data,
                                        size_t size) noexcept override {
      return record(session, topic, data, size, true);
    }
    PluginWsSendResult send_latest_binary(const char* session, const char* topic, const void* data,
                                          size_t size) noexcept override {
      return record(session, topic, data, size, true);
    }
    PluginWsBroadcastResult broadcast_text(const char*, const void*, size_t) noexcept override {
      return {};
    }
    PluginWsBroadcastResult broadcast_binary(const char*, const void*, size_t) noexcept override {
      return {};
    }
    PluginWsBroadcastResult broadcast_latest_text(const char*, const char*, const void*,
                                                  size_t) noexcept override {
      return {};
    }
    PluginWsBroadcastResult broadcast_latest_binary(const char*, const char*, const void*,
                                                    size_t) noexcept override {
      return {};
    }
    bool close_session(const char*, uint16_t, const char*) noexcept override { return true; }
    size_t session_count(const char*) const noexcept override { return 0; }
    bool endpoint_metrics(const char*, PluginWsEndpointMetrics&) const noexcept override {
      return true;
    }

    void message(const std::string& session, const nlohmann::json& value) {
      const auto text = value.dump();
      on_message(session.c_str(), text.data(), text.size(), PluginWsMessageType::Text);
    }
    nlohmann::json last_json() const {
      CHECK(!sent.empty());
      return nlohmann::json::parse(sent.back().payload);
    }
  };

  struct FakeContext final : IPluginContext {
    FakeLifetime lifetime;
    FakeLogger logger;
    FakePubSub pubsub;
    FakeReqRep reqrep;
    FakeRuntime runtime;
    FakeWebSocket websocket;
    FakeHttp http;
    std::unordered_map<uint64_t, ICapability*> capabilities;

    FakeContext() {
      capabilities = {{IPluginLifetime::id(), &lifetime},
                      {ILogger::id(), &logger},
                      {IPubSub::id(), &pubsub},
                      {IReqRep::id(), &reqrep},
                      {IPluginRuntimeCap::id(), &runtime},
                      {IPluginWebSocketCap::id(), &websocket},
                      {IPluginHttpCap::id(), &http}};
    }
    ICapability* query(uint64_t id) override {
      const auto it = capabilities.find(id);
      return it == capabilities.end() ? nullptr : it->second;
    }
    ICapability* query(uint64_t id) const override {
      const auto it = capabilities.find(id);
      return it == capabilities.end() ? nullptr : it->second;
    }
  };
}  // namespace

int main() {
  try {
    FakeContext context;
    SimulationPlug plugin;
    CHECK(plugin.init(context));
    CHECK(plugin.start());
    CHECK(context.http.registered);
    CHECK(context.websocket.endpoint == "simulation");
    CHECK(context.websocket.options.max_receive_message_size == 64 * 1024);
    CHECK(context.websocket.options.max_send_message_size == 8 * 1024 * 1024);
    CHECK(context.websocket.options.max_sessions == 32);

    context.websocket.on_open("s1");
    CHECK(context.websocket.last_json().value("type", "") == "hello");

    context.websocket.message(
        "s1", {{"action", "subscribe"}, {"instances", {"robot1"}}, {"visual", true}});
    CHECK(context.websocket.last_json().value("type", "") == "ack");

    const size_t before_state = context.websocket.sent.size();
    plugin.test_publish_state({{"id", "robot1"}, {"status", "running"}});
    CHECK(context.websocket.sent.size() == before_state + 1);
    CHECK(context.websocket.sent.back().latest);
    CHECK(context.websocket.sent.back().topic == "simulation.instance.robot1.state");

    const size_t before_other = context.websocket.sent.size();
    plugin.test_publish_state({{"id", "robot2"}, {"status", "running"}});
    CHECK(context.websocket.sent.size() == before_other);

    context.websocket.message("s1",
                              {{"action", "subscribe"}, {"instances", nlohmann::json::array()}});
    CHECK(context.websocket.last_json().value("type", "") == "error");
    context.websocket.message(
        "s1", {{"action", "subscribe"}, {"instances", {"robot1"}}, {"visual", "yes"}});
    CHECK(context.websocket.last_json().value("type", "") == "error");

    nlohmann::json too_many = nlohmann::json::array();
    for (int i = 0; i < 65; ++i) too_many.push_back("robot" + std::to_string(i));
    context.websocket.message("s1", {{"action", "subscribe"}, {"instances", too_many}});
    CHECK(context.websocket.last_json().value("type", "") == "error");

    context.websocket.message("s1", {{"action", "unsubscribe"}, {"instances", {"robot1"}}});
    CHECK(context.websocket.last_json().value("type", "") == "ack");
    const size_t after_unsubscribe = context.websocket.sent.size();
    plugin.test_publish_state({{"id", "robot1"}, {"status", "running"}});
    CHECK(context.websocket.sent.size() == after_unsubscribe);

    context.websocket.message("s1", {{"action", "subscribe"}, {"instances", {"robot1"}}});
    context.websocket.on_close("s1");
    const size_t after_close = context.websocket.sent.size();
    plugin.test_publish_state({{"id", "robot1"}, {"status", "running"}});
    CHECK(context.websocket.sent.size() == after_close);

    context.websocket.message("slow", {{"action", "subscribe"}, {"instances", {"robot1"}}});
    context.websocket.send_result = PluginWsSendResult::QueueFull;
    plugin.test_publish_state({{"id", "robot1"}, {"status", "running"}});
    context.websocket.on_open("slow");
    CHECK(context.logger.warnings.size() >= 2);
    CHECK(context.logger.warnings[context.logger.warnings.size() - 2].find(
              "send_latest_text failed: result=QueueFull")
          != std::string::npos);
    CHECK(context.logger.warnings[context.logger.warnings.size() - 2].find("bytes=")
          != std::string::npos);
    CHECK(context.logger.warnings.back().find("send_text failed: result=QueueFull")
          != std::string::npos);
    CHECK(context.logger.warnings.back().find("bytes=") != std::string::npos);

    context.websocket.send_result = PluginWsSendResult::SessionClosed;
    plugin.test_publish_state({{"id", "robot1"}, {"status", "running"}});
    context.websocket.send_result = PluginWsSendResult::Queued;
    const size_t after_stale_session = context.websocket.sent.size();
    plugin.test_publish_state({{"id", "robot1"}, {"status", "running"}});
    CHECK(context.websocket.sent.size() == after_stale_session);

    plugin.stop();
    plugin.onUnload();
    return 0;
  } catch (const std::exception& error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
