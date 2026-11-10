#pragma once

#include <array>
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
    App(const App&) = delete;
    App& operator=(const App&) = delete;

    void draw();

  private:
    enum class Layout
    {
        None,
        Tabs,
        SideBySide,
        Stacked,
    };

    void registerSettings();
    void arrangeWindows(unsigned int dockSpace, ImVec2 size, const std::vector<std::string>& windowNames);
    void drawConnectionBar(const DeviceWorker::Snapshot& snapshot);
    void drawController(std::size_t index, ftrobotx::Extension& extension, bool online);
    // Settings of the UI per controller that the transfer area doesn't hold.
    struct ControllerUi
    {
        std::string message;
        std::array<bool, ftrobotx::kUniversalInputs> ntc{}; // analog resistance shown as NTC temperature
        std::array<int, ftrobotx::kMotors> distanceSpeed{400, 400, 400, 400};
        std::array<int, ftrobotx::kMotors> distance{190, 190, 190, 190};
        int syncMaster = 0;
        int syncSlave = 1;
        int syncSpeed = 400;
        int syncDistance = 0;
        bool syncOpposite = false;
    };

    ControllerUi& ui(std::size_t index);
    void drawMessage(std::size_t index);
    void drawInputs(std::size_t index, ftrobotx::Extension& extension, bool& configChanged);
    void drawCounters(ftrobotx::Extension& extension, bool& outputChanged, bool& configChanged);
    void drawOutputs(std::size_t index, ftrobotx::Extension& extension, bool& outputChanged, bool& configChanged);
    void drawSync(std::size_t index, ftrobotx::Extension& extension, bool& outputChanged);

    DeviceWorker worker_;
    std::string port_;
    std::vector<ControllerUi> controllerUi_;
    void startPortScan();

    std::vector<ftrobotx::SerialPort::PortInfo> ports_;
    std::future<std::vector<ftrobotx::SerialPort::PortInfo>> portScan_; // listing can take a moment (BlueZ)
    int numExtensions_ = 1;
    Layout pendingLayout_ = Layout::None; // applied in the next frame
    // Saved in imgui.ini: the chosen arrangement, and for how many controllers it was built. Connecting with a
    // different number of controllers rebuilds it, so there are no empty or missing places.
    struct LayoutSettings
    {
        Layout style = Layout::Tabs;
        std::size_t arrangedControllers = 0;
    };
    // Static: ImGui saves the settings once more when it shuts down, after the App is gone.
    static LayoutSettings layoutSettings_;
    Layout& layoutStyle_ = layoutSettings_.style;
    std::size_t& arrangedControllers_ = layoutSettings_.arrangedControllers;
    bool wasConnected_ = false;
    int focusMasterFrames_ = 0;
    double stoppedAt_ = -10; // ImGui time of the last STOP OUTPUTS, for the confirmation
};
