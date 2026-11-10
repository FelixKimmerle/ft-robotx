#include "App.hpp"

#include <algorithm>
#include <array>
#include <cstdio>
#include <optional>
#include <string_view>
#include <utility>

#include <imgui.h>
#include <imgui_internal.h> // DockBuilder and BeginViewportSideBar
#include <misc/cpp/imgui_stdlib.h>

#include "ftrobotx/Sensors.hpp"
#include "ftrobotx/SerialPort.hpp"

namespace
{

const ImVec4 kRed{0.95f, 0.35f, 0.35f, 1.0f};
const ImVec4 kGreen{0.45f, 0.85f, 0.45f, 1.0f};
const ImVec4 kYellow{0.95f, 0.8f, 0.3f, 1.0f};

struct ModeChoice
{
    ftrobotx::InputMode mode;
    const char* label;
    bool ntc; // analog resistance, shown as temperature
};

constexpr std::array kInputModes{
    ModeChoice{ftrobotx::InputMode::Voltage, "Voltage", false},
    ModeChoice{ftrobotx::InputMode::Resistance, "Resistance", false},
    ModeChoice{ftrobotx::InputMode::Resistance,
               "NTC (\xC2\xB0"
               "C)",
               true}, // explicit UTF-8, also for MSVC
    ModeChoice{ftrobotx::InputMode::Ultrasonic, "Ultrasonic", false},
};

constexpr const char* kMotorNames[] = {"M1", "M2", "M3", "M4"};

/// Value of a universal input in the unit of its mode, with the limits from ROBO_TX_FW.h.
struct InputReading
{
    std::string text;
    float fraction;
};

InputReading readInput(ftrobotx::UniversalInputConfig config, bool ntc, std::uint16_t value)
{
    char buffer[48];
    if (config.digital)
    {
        return {value != 0 ? "1" : "0", value != 0 ? 1.0f : 0.0f};
    }
    if (ntc && config.mode == ftrobotx::InputMode::Resistance)
    {
        if (std::optional<double> celsius = ftrobotx::ntcCelsius(value))
        {
            std::snprintf(buffer, sizeof(buffer),
                          "%.1f \xC2\xB0"
                          "C  (%u Ohm)",
                          *celsius, value);
            return {buffer, std::clamp(static_cast<float>(*celsius + 20.0) / 100.0f, 0.0f, 1.0f)}; // -20 ... 80 °C
        }
        return {value == 0 ? "waiting for a value" : "no sensor", 0.0f};
    }
    switch (config.mode)
    {
    case ftrobotx::InputMode::Voltage:
        if (value >= 10000)
        {
            return {"overrun", 1.0f};
        }
        std::snprintf(buffer, sizeof(buffer), "%u mV", value);
        return {buffer, value / 9999.0f};
    case ftrobotx::InputMode::Resistance:
        if (value >= 5000)
        {
            return {"overrun", 1.0f};
        }
        std::snprintf(buffer, sizeof(buffer), "%u Ohm", value);
        return {buffer, value / 4999.0f};
    case ftrobotx::InputMode::Ultrasonic:
        if (std::optional<std::uint16_t> cm = ftrobotx::ultrasonicCm(value))
        {
            std::snprintf(buffer, sizeof(buffer), "%u cm", *cm);
            return {buffer, *cm / static_cast<float>(ftrobotx::kUltrasonicMax)};
        }
        if (value == ftrobotx::kUltrasonicNotPresent)
        {
            return {"no sensor", 0.0f};
        }
        if (value == ftrobotx::kUltrasonicOverload)
        {
            return {"nothing in range", 1.0f};
        }
        std::snprintf(buffer, sizeof(buffer), value == 0 ? "waiting for a value" : "%u (invalid)", value);
        return {buffer, 0.0f};
    default:
        std::snprintf(buffer, sizeof(buffer), "%u", value);
        return {buffer, 0.0f};
    }
}

std::string controllerName(std::size_t index)
{
    return index == 0 ? "Master" : "Extension " + std::to_string(index);
}

} // namespace

App::App(std::string port, int numExtensions) : port_(std::move(port)), numExtensions_(std::clamp(numExtensions, 0, 8))
{
    registerSettings();
    startPortScan();
    if (!port_.empty())
    {
        worker_.connect(port_, static_cast<std::size_t>(numExtensions_) + 1);
    }
}

App::LayoutSettings App::layoutSettings_;

void App::registerSettings()
{
    ImGuiSettingsHandler handler;
    handler.TypeName = "FtRoboTxGui";
    handler.TypeHash = ImHashStr("FtRoboTxGui");
    handler.UserData = &layoutSettings_;
    handler.ReadOpenFn = [](ImGuiContext*, ImGuiSettingsHandler*, const char* name) -> void* {
        return std::string_view(name) == "Layout" ? reinterpret_cast<void*>(1) : nullptr;
    };
    handler.ReadLineFn = [](ImGuiContext*, ImGuiSettingsHandler* settings, void*, const char* line) {
        auto* layout = static_cast<LayoutSettings*>(settings->UserData);
        int value = 0;
        if (std::sscanf(line, "Style=%d", &value) == 1 && value >= 1 && value <= 3)
        {
            layout->style = static_cast<Layout>(value);
        }
        else if (std::sscanf(line, "Controllers=%d", &value) == 1 && value >= 0)
        {
            layout->arrangedControllers = static_cast<std::size_t>(value);
        }
    };
    handler.WriteAllFn = [](ImGuiContext*, ImGuiSettingsHandler* settings, ImGuiTextBuffer* out) {
        const auto* layout = static_cast<const LayoutSettings*>(settings->UserData);
        out->appendf("[%s][Layout]\nStyle=%d\nControllers=%zu\n\n", settings->TypeName, static_cast<int>(layout->style),
                     layout->arrangedControllers);
    };
    ImGui::AddSettingsHandler(&handler);
}

void App::startPortScan()
{
    if (!portScan_.valid())
    {
        portScan_ = std::async(std::launch::async, ftrobotx::SerialPort::listPorts);
    }
}

void App::draw()
{
    DeviceWorker::Snapshot snapshot = worker_.snapshot();
    ImGuiViewport* viewport = ImGui::GetMainViewport();

    // Connection bar fixed at the top; the rest of the window is a dock space for the controller windows.
    float barHeight = ImGui::GetFrameHeight() + ImGui::GetStyle().WindowPadding.y * 2;
    if (ImGui::BeginViewportSideBar("##connection", viewport, ImGuiDir_Up, barHeight,
                                    ImGuiWindowFlags_NoSavedSettings | ImGuiWindowFlags_NoScrollbar))
    {
        drawConnectionBar(snapshot);
    }
    ImGui::End();

    std::vector<std::string> windowNames;
    if (snapshot.state == DeviceWorker::State::Connected)
    {
        for (std::size_t i = 0; i < snapshot.extensions.size(); ++i)
        {
            windowNames.push_back(controllerName(i) + "###controller" + std::to_string(i));
        }
    }

    ImGuiID dockSpace = ImGui::GetID("controllers");
    if (pendingLayout_ == Layout::None && !windowNames.empty() && windowNames.size() != arrangedControllers_)
    {
        pendingLayout_ = layoutStyle_; // different number of controllers than last time: rebuild the arrangement
    }
    if (pendingLayout_ != Layout::None && !windowNames.empty())
    {
        arrangeWindows(dockSpace, viewport->WorkSize, windowNames);
    }
    ImGui::DockSpaceOverViewport(dockSpace, viewport);

    if (windowNames.empty())
    {
        wasConnected_ = false;
        ImGui::SetNextWindowDockID(dockSpace, ImGuiCond_Always);
        if (ImGui::Begin("Not connected###placeholder"))
        {
            ImGui::TextDisabled("Choose the port (USB: /dev/ttyACM*, Bluetooth: /dev/rfcomm*, Windows: COMx)");
            ImGui::TextDisabled("and the number of extensions, then press Connect.");
        }
        ImGui::End();
        return;
    }

    if (!wasConnected_)
    {
        // Show the master first if the windows are tabs. Repeated for a few frames because the windows only
        // join their dock node after their first frame.
        focusMasterFrames_ = 3;
    }
    wasConnected_ = true;

    for (std::size_t i = 0; i < windowNames.size(); ++i)
    {
        ImGui::SetNextWindowDockID(dockSpace, ImGuiCond_FirstUseEver);
        if (i == 0 && focusMasterFrames_ > 0)
        {
            --focusMasterFrames_;
            ImGui::SetNextWindowFocus();
        }
        if (ImGui::Begin(windowNames[i].c_str()))
        {
            drawController(i, snapshot.extensions[i], i == 0 || snapshot.extensionsOnline[i - 1]);
        }
        ImGui::End();
    }
}

void App::arrangeWindows(unsigned int dockSpace, ImVec2 size, const std::vector<std::string>& windowNames)
{
    ImGui::DockBuilderRemoveNode(dockSpace);
    ImGui::DockBuilderAddNode(dockSpace, ImGuiDockNodeFlags_DockSpace);
    ImGui::DockBuilderSetNodeSize(dockSpace, size);

    if (pendingLayout_ == Layout::Tabs)
    {
        for (const std::string& name : windowNames)
        {
            ImGui::DockBuilderDockWindow(name.c_str(), dockSpace);
        }
    }
    else
    {
        // Split off one equal share per window, left to right or top to bottom.
        ImGuiDir direction = pendingLayout_ == Layout::SideBySide ? ImGuiDir_Left : ImGuiDir_Up;
        ImGuiID remaining = dockSpace;
        for (std::size_t i = 0; i + 1 < windowNames.size(); ++i)
        {
            ImGuiID node = 0;
            float share = 1.0f / static_cast<float>(windowNames.size() - i);
            ImGui::DockBuilderSplitNode(remaining, direction, share, &node, &remaining);
            ImGui::DockBuilderDockWindow(windowNames[i].c_str(), node);
        }
        ImGui::DockBuilderDockWindow(windowNames.back().c_str(), remaining);
    }

    ImGui::DockBuilderFinish(dockSpace);
    pendingLayout_ = Layout::None;
    arrangedControllers_ = windowNames.size();
    ImGui::MarkIniSettingsDirty();
}

void App::drawConnectionBar(const DeviceWorker::Snapshot& snapshot)
{
    using State = DeviceWorker::State;
    bool busy = snapshot.state == State::Connecting || snapshot.state == State::Connected;

    if (portScan_.valid() && portScan_.wait_for(std::chrono::seconds(0)) == std::future_status::ready)
    {
        ports_ = portScan_.get();
        if (port_.empty())
        {
            // Preselect the first port found, ROBO TX controllers come first.
            port_ = ports_.empty() ? "/dev/ttyACM0" : ports_.front().name;
        }
    }

    ImGui::BeginDisabled(busy);
    ImGui::AlignTextToFramePadding();
    ImGui::TextUnformatted("Port");
    ImGui::SameLine();
    ImGui::SetNextItemWidth(ImGui::GetFontSize() * 18);
    ImGui::InputText("##port", &port_);
    ImGui::SameLine(0, 0);
    if (ImGui::BeginCombo("##ports", nullptr, ImGuiComboFlags_NoPreview | ImGuiComboFlags_PopupAlignLeft))
    {
        if (ImGui::IsWindowAppearing())
        {
            startPortScan();
        }
        if (portScan_.valid())
        {
            ImGui::TextDisabled("Searching...");
        }
        else if (ports_.empty())
        {
            ImGui::TextDisabled("No ports found");
        }
        for (const ftrobotx::SerialPort::PortInfo& port : ports_)
        {
            std::string label = port.description + "##" + port.name;
            if (ImGui::Selectable(label.c_str(), port.name == port_))
            {
                port_ = port.name;
            }
            ImGui::SameLine();
            ImGui::TextDisabled("%s", port.name.c_str());
        }
        ImGui::EndCombo();
    }

    ImGui::SameLine();
    ImGui::TextUnformatted("Extensions");
    ImGui::SameLine();
    ImGui::SetNextItemWidth(ImGui::GetFontSize() * 6);
    if (ImGui::InputInt("##extensions", &numExtensions_))
    {
        numExtensions_ = std::clamp(numExtensions_, 0, 8);
    }
    ImGui::SetItemTooltip("Number of extension controllers (EXT 1 ... EXT n) behind the master");
    ImGui::EndDisabled();

    ImGui::SameLine();
    if (snapshot.state == State::Connected || snapshot.state == State::Connecting)
    {
        ImGui::BeginDisabled(snapshot.state == State::Connecting);
        if (ImGui::Button("Disconnect"))
        {
            worker_.disconnect();
        }
        ImGui::EndDisabled();
    }
    else if (ImGui::Button("Connect"))
    {
        worker_.connect(port_, static_cast<std::size_t>(numExtensions_) + 1);
    }

    if (snapshot.state == State::Connected && snapshot.extensions.size() > 1)
    {
        ImGui::SameLine();
        ImGui::SetNextItemWidth(ImGui::CalcTextSize("Side by side").x + ImGui::GetFrameHeight() * 2);
        if (ImGui::BeginCombo("##layout", "Layout"))
        {
            if (ImGui::Selectable("Tabs"))
            {
                pendingLayout_ = layoutStyle_ = Layout::Tabs;
            }
            if (ImGui::Selectable("Side by side"))
            {
                pendingLayout_ = layoutStyle_ = Layout::SideBySide;
            }
            if (ImGui::Selectable("Stacked"))
            {
                pendingLayout_ = layoutStyle_ = Layout::Stacked;
            }
            ImGui::EndCombo();
        }
        ImGui::SetItemTooltip("Arrange the controller windows. You can also drag a window's tab to dock it anywhere.");
    }

    ImGui::SameLine();
    switch (snapshot.state)
    {
    case State::Disconnected:
        ImGui::TextDisabled("Disconnected");
        break;
    case State::Connecting:
        ImGui::TextUnformatted("Connecting...");
        break;
    case State::Connected:
        ImGui::TextColored(kGreen, "Connected, %.0f cycles/s", snapshot.cyclesPerSecond);
        break;
    case State::Failed:
        ImGui::TextColored(kRed, "Error: %s", snapshot.error.c_str());
        break;
    }

    if (ImGui::GetTime() - stoppedAt_ < 2.0) // confirmation after STOP OUTPUTS
    {
        ImGui::SameLine();
        ImGui::TextColored(kYellow, "All outputs stopped");
    }

    // Emergency stop, right-aligned; Escape works from anywhere.
    const char* stopLabel = "STOP OUTPUTS (Esc)";
    float stopWidth = ImGui::CalcTextSize(stopLabel).x + ImGui::GetStyle().FramePadding.x * 2;
    ImGui::SameLine(ImGui::GetContentRegionMax().x - stopWidth);
    ImGui::BeginDisabled(snapshot.state != State::Connected);
    ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.65f, 0.15f, 0.15f, 1.0f));
    ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ImVec4(0.8f, 0.2f, 0.2f, 1.0f));
    ImGui::PushStyleColor(ImGuiCol_ButtonActive, ImVec4(0.9f, 0.25f, 0.25f, 1.0f));
    bool stopClicked = ImGui::Button(stopLabel);
    ImGui::PopStyleColor(3);
    ImGui::EndDisabled();

    if (snapshot.state == State::Connected &&
        (stopClicked || ImGui::Shortcut(ImGuiKey_Escape, ImGuiInputFlags_RouteGlobal)))
    {
        worker_.stopAllOutputs();
        stoppedAt_ = ImGui::GetTime();
    }
}

void App::drawController(std::size_t index, ftrobotx::Extension& extension, bool online)
{
    ImGui::PushID(static_cast<int>(index));

    ImGui::TextUnformatted(extension.name.empty() ? "(no name)" : extension.name.c_str());
    ImGui::SameLine();
    ImGui::TextDisabled("MAC %s", extension.macAddress.empty() ? "-" : extension.macAddress.c_str());
    if (extension.version)
    {
        ImGui::SameLine();
        ImGui::TextDisabled("Firmware %s, hardware %c, transfer area %d.%02d", extension.version->toString().c_str(),
                            extension.version->hardwareRevision, extension.version->transferAreaMajor,
                            extension.version->transferAreaMinor);
    }
    if (!online)
    {
        ImGui::PushStyleColor(ImGuiCol_Text, kYellow);
        ImGui::TextWrapped("Extension %zu is not connected to the master. Check the EXT cable and that its role is "
                           "set to Extension %zu.",
                           index, index);
        ImGui::PopStyleColor();
    }

    bool outputChanged = false;
    bool configChanged = false;

    auto drawLeft = [&] {
        drawInputs(index, extension, configChanged);
        drawCounters(extension, outputChanged, configChanged);
        ImGui::SeparatorText("Display");
        ImGui::Text("Buttons: left %u ms, right %u ms", extension.input.displayButtonLeft(),
                    extension.input.displayButtonRight());
        drawMessage(index);
    };

    // Inputs and outputs side by side if the window is wide enough, otherwise outputs below the inputs.
    bool twoColumns = ImGui::GetContentRegionAvail().x >= ImGui::GetFontSize() * 60;
    if (!twoColumns)
    {
        drawLeft();
        drawOutputs(index, extension, outputChanged, configChanged);
    }
    // The table spans the remaining height, so the divider between the columns reaches the bottom.
    else if (ImGui::BeginTable("layout", 2, ImGuiTableFlags_Resizable | ImGuiTableFlags_BordersInnerV,
                               ImVec2(0, ImGui::GetContentRegionAvail().y)))
    {
        ImGui::TableNextColumn();
        drawLeft();
        ImGui::TableNextColumn();
        drawOutputs(index, extension, outputChanged, configChanged);
        ImGui::EndTable();
    }

    if (outputChanged)
    {
        worker_.setOutputs(index, extension.output);
    }
    if (configChanged)
    {
        worker_.setConfig(index, extension.config);
    }

    ImGui::PopID();
}

App::ControllerUi& App::ui(std::size_t index)
{
    if (controllerUi_.size() <= index)
    {
        controllerUi_.resize(index + 1);
    }
    return controllerUi_[index];
}

void App::drawMessage(std::size_t index)
{
    std::string& text = ui(index).message;

    const ImGuiStyle& style = ImGui::GetStyle();
    float buttons = ImGui::CalcTextSize("ShowClear").x + style.FramePadding.x * 4 + style.ItemSpacing.x * 2;
    ImGui::SetNextItemWidth(-buttons);
    bool enter = ImGui::InputTextWithHint("##message", "Message for the display (max. 98 characters)", &text,
                                          ImGuiInputTextFlags_EnterReturnsTrue);
    if (text.size() > 98)
    {
        text.resize(98);
    }
    ImGui::SameLine();
    if (ImGui::Button("Show") || enter)
    {
        worker_.showMessage(index, text);
    }
    ImGui::SameLine();
    if (ImGui::Button("Clear"))
    {
        worker_.showMessage(index, "");
    }
}

void App::drawInputs(std::size_t index, ftrobotx::Extension& extension, bool& configChanged)
{
    std::array<bool, ftrobotx::kUniversalInputs>& ntc = ui(index).ntc;
    ImGui::SeparatorText("Universal inputs");
    if (!ImGui::BeginTable("inputs", 4, ImGuiTableFlags_SizingFixedFit | ImGuiTableFlags_RowBg))
    {
        return;
    }
    ImGui::TableSetupColumn("Input");
    ImGui::TableSetupColumn("Mode");
    ImGui::TableSetupColumn("Digital");
    ImGui::TableSetupColumn("Value", ImGuiTableColumnFlags_WidthStretch);
    ImGui::TableHeadersRow();

    for (std::size_t i = 0; i < ftrobotx::kUniversalInputs; ++i)
    {
        ImGui::PushID(static_cast<int>(i));
        ftrobotx::UniversalInputConfig config = extension.config.universalInputConfig(i);

        ImGui::TableNextColumn();
        ImGui::AlignTextToFramePadding();
        ImGui::Text("I%zu", i + 1);

        ImGui::TableNextColumn();
        bool showNtc = ntc[i] && config.mode == ftrobotx::InputMode::Resistance && !config.digital;
        auto isCurrent = [&](const ModeChoice& choice) { return choice.mode == config.mode && choice.ntc == showNtc; };
        const char* current = "?";
        for (const ModeChoice& choice : kInputModes)
        {
            current = isCurrent(choice) ? choice.label : current;
        }
        ImGui::SetNextItemWidth(ImGui::GetFontSize() * 8);
        if (ImGui::BeginCombo("##mode", current))
        {
            for (const ModeChoice& choice : kInputModes)
            {
                if (ImGui::Selectable(choice.label, isCurrent(choice)))
                {
                    config.mode = choice.mode;
                    config.digital = config.digital && choice.mode != ftrobotx::InputMode::Ultrasonic && !choice.ntc;
                    ntc[i] = choice.ntc;
                    extension.config.setUniversalInputConfig(i, config);
                    configChanged = true;
                }
            }
            ImGui::EndCombo();
        }

        ImGui::TableNextColumn();
        ImGui::BeginDisabled(config.mode == ftrobotx::InputMode::Ultrasonic || showNtc);
        if (ImGui::Checkbox("##digital", &config.digital))
        {
            extension.config.setUniversalInputConfig(i, config);
            configChanged = true;
        }
        ImGui::EndDisabled();

        ImGui::TableNextColumn();
        InputReading reading = readInput(config, showNtc, extension.input.universalInput(i));
        ImGui::ProgressBar(reading.fraction, ImVec2(-1, 0), reading.text.c_str());

        ImGui::PopID();
    }
    ImGui::EndTable();
}

void App::drawCounters(ftrobotx::Extension& extension, bool& outputChanged, bool& configChanged)
{
    ImGui::SeparatorText("Counters");
    if (!ImGui::BeginTable("counters", 5, ImGuiTableFlags_SizingFixedFit | ImGuiTableFlags_RowBg))
    {
        return;
    }
    ImGui::TableSetupColumn("Counter");
    ImGui::TableSetupColumn("Level");
    ImGui::TableSetupColumn("Value", ImGuiTableColumnFlags_WidthStretch);
    ImGui::TableSetupColumn("Counts on");
    ImGui::TableSetupColumn("");
    ImGui::TableHeadersRow();

    constexpr const char* kCounterModes[] = {"0 -> 1", "1 -> 0"};

    for (std::size_t i = 0; i < ftrobotx::kMotors; ++i)
    {
        ImGui::PushID(static_cast<int>(i));

        ImGui::TableNextColumn();
        ImGui::AlignTextToFramePadding();
        ImGui::Text("C%zu", i + 1);

        ImGui::TableNextColumn();
        ImGui::AlignTextToFramePadding();
        ImGui::Text("%u", extension.input.counterInput(i));

        ImGui::TableNextColumn();
        ImGui::AlignTextToFramePadding();
        ImGui::Text("%u", extension.input.counterValue(i));

        ImGui::TableNextColumn();
        int mode = std::min<int>(extension.config.counterConfig(i), 1);
        ImGui::SetNextItemWidth(ImGui::GetFontSize() * 6);
        if (ImGui::Combo("##mode", &mode, kCounterModes, 2))
        {
            extension.config.setCounterConfig(i, static_cast<std::uint8_t>(mode));
            configChanged = true;
        }

        ImGui::TableNextColumn();
        if (ImGui::Button("Reset"))
        {
            ++extension.output.counterResetCommandId[i];
            outputChanged = true;
        }

        ImGui::PopID();
    }
    ImGui::EndTable();
}

void App::drawOutputs(std::size_t index, ftrobotx::Extension& extension, bool& outputChanged, bool& configChanged)
{
    ftrobotx::Output& output = extension.output;
    const ImGuiStyle& style = ImGui::GetStyle();
    ControllerUi& settings = ui(index);

    for (std::size_t m = 0; m < ftrobotx::kMotors; ++m)
    {
        ImGui::PushID(static_cast<int>(m));
        bool motorMode = extension.config.motorConfig(m) != 0;

        char title[32];
        std::snprintf(title, sizeof(title), "M%zu  /  O%zu, O%zu", m + 1, 2 * m + 1, 2 * m + 2);
        ImGui::SeparatorText(title);

        // Switching ends a distance/synchronization command, so the firmware doesn't hold a partner motor.
        if (ImGui::RadioButton("Motor", motorMode) && !motorMode)
        {
            extension.config.setMotorConfig(m, 1);
            extension.setMotorOutput(m, 0);
            configChanged = outputChanged = true;
        }
        ImGui::SameLine();
        if (ImGui::RadioButton("Two outputs", !motorMode) && motorMode)
        {
            extension.config.setMotorConfig(m, 0);
            extension.setMotorOutput(m, 0);
            configChanged = outputChanged = true;
        }

        if (motorMode)
        {
            std::optional<std::size_t> partner = extension.syncPartner(m);
            if (partner)
            {
                ImGui::SameLine();
                ImGui::TextColored(kGreen, "synchronized with %s", kMotorNames[*partner]);
            }

            int speed = output.pwmOutputValues[2 * m] - output.pwmOutputValues[2 * m + 1];
            ImGui::SetNextItemWidth(-ImGui::CalcTextSize("Stop").x - style.ItemSpacing.x * 2 -
                                    style.FramePadding.x * 2);
            if (ImGui::SliderInt("##speed", &speed, -512, 512, "%d"))
            {
                extension.setMotorOutput(m, static_cast<std::int16_t>(speed));
                outputChanged = true;
            }
            ImGui::SetItemTooltip("Speed and direction, -512 ... 512. Ctrl+click to type a value.\n"
                                  "Ends a distance or synchronization command; a synchronized motor stops.");
            ImGui::SameLine();
            if (ImGui::Button("Stop"))
            {
                extension.stopMotor(m);
                outputChanged = true;
            }

            if (ImGui::TreeNode("Distance"))
            {
                ImGui::SetNextItemWidth(ImGui::GetFontSize() * 10);
                ImGui::SliderInt("Speed", &settings.distanceSpeed[m], -512, 512, "%d");
                ImGui::SetNextItemWidth(ImGui::GetFontSize() * 10);
                if (ImGui::InputInt("Counter pulses", &settings.distance[m]))
                {
                    settings.distance[m] = std::clamp(settings.distance[m], 1, 65535);
                }
                ImGui::BeginDisabled(settings.distanceSpeed[m] == 0);
                if (ImGui::Button("Start"))
                {
                    extension.startMotorDistance(m, static_cast<std::int16_t>(settings.distanceSpeed[m]),
                                                 static_cast<std::uint16_t>(settings.distance[m]));
                    outputChanged = true;
                }
                ImGui::EndDisabled();
                ImGui::SetItemTooltip("Runs the motor for the given number of pulses on C%zu, then the TX brakes it. "
                                      "Resets the counter.",
                                      m + 1);
                ImGui::SameLine();
                if (output.motorDistanceValues[m] == 0 || partner)
                {
                    ImGui::TextDisabled("No distance command");
                }
                else if (extension.motorCommandDone(m))
                {
                    ImGui::TextColored(kGreen, "Target reached");
                }
                else
                {
                    ImGui::TextUnformatted("Running...");
                }
                ImGui::TreePop();
            }
        }
        else
        {
            for (std::size_t o = 0; o < 2; ++o)
            {
                std::int16_t& duty = output.pwmOutputValues[2 * m + o];
                int value = duty;
                char label[16];
                std::snprintf(label, sizeof(label), "O%zu", 2 * m + o + 1);
                ImGui::SetNextItemWidth(-ImGui::CalcTextSize("O8").x - style.ItemInnerSpacing.x);
                if (ImGui::SliderInt(label, &value, 0, 512))
                {
                    duty = static_cast<std::int16_t>(value);
                    outputChanged = true;
                }
            }
        }

        ImGui::PopID();
    }

    drawSync(index, extension, outputChanged);
}

void App::drawSync(std::size_t index, ftrobotx::Extension& extension, bool& outputChanged)
{
    ControllerUi& settings = ui(index);
    ImGui::SeparatorText("Synchronized motors");
    ImGui::PushID("sync");

    float comboWidth = ImGui::GetFontSize() * 4;
    ImGui::AlignTextToFramePadding();
    ImGui::TextUnformatted("Master");
    ImGui::SameLine();
    ImGui::SetNextItemWidth(comboWidth);
    ImGui::Combo("##master", &settings.syncMaster, kMotorNames, 4);
    ImGui::SameLine();
    ImGui::TextUnformatted("Slave");
    ImGui::SameLine();
    ImGui::SetNextItemWidth(comboWidth);
    ImGui::Combo("##slave", &settings.syncSlave, kMotorNames, 4);
    ImGui::SameLine();
    ImGui::Checkbox("Opposite direction", &settings.syncOpposite);
    ImGui::SetItemTooltip("The slave turns the other way, e.g. to turn a robot on the spot");

    ImGui::SetNextItemWidth(ImGui::GetFontSize() * 10);
    ImGui::SliderInt("Speed", &settings.syncSpeed, -512, 512, "%d");
    ImGui::SetNextItemWidth(ImGui::GetFontSize() * 10);
    if (ImGui::InputInt("Counter pulses (0 = endless)", &settings.syncDistance))
    {
        settings.syncDistance = std::clamp(settings.syncDistance, 0, 65535);
    }

    auto master = static_cast<std::size_t>(settings.syncMaster);
    auto slave = static_cast<std::size_t>(settings.syncSlave);
    const char* problem = nullptr;
    if (master == slave)
    {
        problem = "Choose two different motors";
    }
    else if (extension.config.motorConfig(master) == 0 || extension.config.motorConfig(slave) == 0)
    {
        problem = "Both must be set to Motor";
    }
    else if (settings.syncSpeed == 0)
    {
        problem = "Choose a speed";
    }

    ImGui::BeginDisabled(problem != nullptr);
    if (ImGui::Button("Start"))
    {
        extension.startMotorSync(master, slave, static_cast<std::int16_t>(settings.syncSpeed),
                                 static_cast<std::uint16_t>(settings.syncDistance),
                                 settings.syncOpposite ? ftrobotx::SyncDirection::Opposite
                                                       : ftrobotx::SyncDirection::Same);
        outputChanged = true;
    }
    ImGui::EndDisabled();
    ImGui::SetItemTooltip("The TX keeps both counters equal by holding back the faster motor. Each motor needs its "
                          "encoder on the counter with the same number. Resets both counters.");
    ImGui::SameLine();
    bool running = problem == nullptr && extension.syncPartner(master) == slave;
    ImGui::BeginDisabled(!running);
    if (ImGui::Button("Stop"))
    {
        extension.stopMotor(master); // stops the slave too
        outputChanged = true;
    }
    ImGui::EndDisabled();

    ImGui::SameLine();
    if (problem != nullptr)
    {
        ImGui::TextDisabled("%s", problem);
    }
    else
    {
        int masterCount = extension.input.counterValue(master);
        int slaveCount = extension.input.counterValue(slave);
        if (!running)
        {
            ImGui::TextDisabled("Not running");
        }
        else if (extension.output.motorDistanceValues[master] == 0)
        {
            ImGui::TextUnformatted("Running (endless)");
        }
        else if (extension.motorCommandDone(master) && extension.motorCommandDone(slave))
        {
            ImGui::TextColored(kGreen, "Target reached");
        }
        else
        {
            ImGui::TextUnformatted("Running...");
        }
        ImGui::Text("C%zu = %d, C%zu = %d, difference %d", master + 1, masterCount, slave + 1, slaveCount,
                    masterCount - slaveCount);
    }

    ImGui::PopID();
}
