#include "DeviceWorker.hpp"

#include <chrono>
#include <exception>

#include "ftrobotx/FtConnection.hpp"

using namespace std::chrono_literals;

namespace
{

/// The TX updates its transfer area every 10 ms; polling faster gains nothing.
constexpr auto kCyclePeriod = 10ms;

} // namespace

void DeviceWorker::connect(const std::string& port, std::size_t numControllers)
{
    disconnect();
    {
        std::scoped_lock lock(mutex_);
        shared_ = Snapshot{};
        shared_.state = State::Connecting;
    }
    thread_ = std::jthread([this, port, numControllers](std::stop_token stop) { run(stop, port, numControllers); });
}

void DeviceWorker::disconnect()
{
    if (thread_.joinable())
    {
        thread_.request_stop();
        thread_.join();
    }
}

DeviceWorker::Snapshot DeviceWorker::snapshot() const
{
    std::scoped_lock lock(mutex_);
    return shared_;
}

void DeviceWorker::setOutputs(std::size_t controller, const ftrobotx::Output& output)
{
    std::scoped_lock lock(mutex_);
    if (controller < shared_.extensions.size())
    {
        shared_.extensions[controller].output = output;
    }
}

void DeviceWorker::setConfig(std::size_t controller, const ftrobotx::Config& config)
{
    std::scoped_lock lock(mutex_);
    if (controller < shared_.extensions.size())
    {
        shared_.extensions[controller].config = config;
        shared_.extensions[controller].config.changed = true;
    }
}

void DeviceWorker::stopAllOutputs()
{
    std::scoped_lock lock(mutex_);
    for (ftrobotx::Extension& extension : shared_.extensions)
    {
        extension.output.stopAll();
    }
}

void DeviceWorker::run(std::stop_token stop, const std::string& port, std::size_t numControllers)
{
    try
    {
        ftrobotx::FtConnection ft(port, numControllers);
        std::span<ftrobotx::Extension> device = ft.extensions();
        {
            std::scoped_lock lock(mutex_);
            shared_.extensions.assign(device.begin(), device.end());
            shared_.state = State::Connected;
        }

        auto next = std::chrono::steady_clock::now();
        auto statsStart = next;
        int cycles = 0;

        while (!stop.stop_requested())
        {
            {
                std::scoped_lock lock(mutex_);
                for (std::size_t i = 0; i < device.size(); ++i)
                {
                    ftrobotx::Extension& wanted = shared_.extensions[i];
                    device[i].output = wanted.output;
                    if (wanted.config.changed)
                    {
                        device[i].config = wanted.config; // stays marked changed, update() sends it
                        wanted.config.changed = false;
                    }
                }
            }

            ft.update();

            auto now = std::chrono::steady_clock::now();
            ++cycles;
            {
                std::scoped_lock lock(mutex_);
                for (std::size_t i = 0; i < device.size(); ++i)
                {
                    shared_.extensions[i].input = device[i].input;
                }
                if (now - statsStart >= 1s)
                {
                    shared_.cyclesPerSecond = cycles / std::chrono::duration<double>(now - statsStart).count();
                    statsStart = now;
                    cycles = 0;
                }
            }

            next += kCyclePeriod;
            if (next < now)
            {
                next = now; // fell behind, e.g. after a slow reply; don't try to catch up
            }
            std::this_thread::sleep_until(next);
        }

        // Don't leave anything running when the tool closes.
        ft.stopAllOutputs();
        ft.update();

        std::scoped_lock lock(mutex_);
        shared_.state = State::Disconnected;
    }
    catch (const std::exception& e)
    {
        std::scoped_lock lock(mutex_);
        shared_.state = State::Failed;
        shared_.error = e.what();
    }
}
