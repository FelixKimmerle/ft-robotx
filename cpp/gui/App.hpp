#pragma once

#include <future>
#include <string>
#include <vector>

#include <imgui.h>

#include "DeviceWorker.hpp"
#include "ftrobotx/SerialPort.hpp"

/// The test tool's user interface: connection bar plus one dockable window per controller.
class App
{
public:
    /// Connects right away if a port is given.
    App(std::string port, int numExtensions);

    void draw();

private:
    enum class Layout
    {
        None,
        Tabs,
        SideBySide,
        Stacked,
    };

    void arrangeWindows(unsigned int dockSpace, ImVec2 size, const std::vector<std::string>& windowNames);
    void drawConnectionBar(const DeviceWorker::Snapshot& snapshot);
    void drawController(std::size_t index, ftrobotx::Extension& extension);
    void drawInputs(ftrobotx::Extension& extension, bool& configChanged);
    void drawCounters(ftrobotx::Extension& extension, bool& outputChanged, bool& configChanged);
    void drawOutputs(ftrobotx::Extension& extension, bool& outputChanged, bool& configChanged);

    DeviceWorker worker_;
    std::string port_;
    void startPortScan();

    std::vector<ftrobotx::SerialPort::PortInfo> ports_;
    std::future<std::vector<ftrobotx::SerialPort::PortInfo>> portScan_; // listing can take a moment (BlueZ)
    int numExtensions_ = 1;
    Layout pendingLayout_ = Layout::None; // applied in the next frame
    bool wasConnected_ = false;
    int focusMasterFrames_ = 0;
    double stoppedAt_ = -10; // ImGui time of the last STOP OUTPUTS, for the confirmation
};
