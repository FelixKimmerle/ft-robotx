#pragma once

#include <array>
#include <mutex>
#include <stop_token>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "ftrobotx/Extension.hpp"

/// Talks to the controller on a background thread, so the UI never blocks on the serial port.
/// The UI works on snapshots and hands its changes back with setOutputs() and setConfig().
class DeviceWorker
{
  public:
    enum class State
    {
        Disconnected,
        Connecting,
        Connected,
        Failed,
    };

    struct Snapshot
    {
        State state = State::Disconnected;
        std::string error;
        std::vector<ftrobotx::Extension> extensions;
        double cyclesPerSecond = 0;
        std::array<bool, 8> extensionsOnline{}; // extension 1 ... 8 connected to the master, updated every second
    };

    DeviceWorker() = default;
    DeviceWorker(const DeviceWorker&) = delete;
    DeviceWorker& operator=(const DeviceWorker&) = delete;

    /// numControllers counts the master too.
    void connect(const std::string& port, std::size_t numControllers);

    /// Stops all outputs and closes the port. Blocks until the current cycle is done.
    void disconnect();

    [[nodiscard]] Snapshot snapshot() const;

    void setOutputs(std::size_t controller, const ftrobotx::Output& output);
    void setConfig(std::size_t controller, const ftrobotx::Config& config);
    void stopAllOutputs();

    /// Queues a text for the controller's display; an empty text clears it.
    void showMessage(std::size_t controller, std::string text);

  private:
    void run(std::stop_token stop, const std::string& port, std::size_t numControllers);

    mutable std::mutex mutex_;
    Snapshot shared_;
    std::vector<std::pair<std::size_t, std::string>> pendingMessages_;
    std::jthread thread_; // Declared last: stopped and joined before the rest is destroyed.
};
