// The model TX behind a relay TX: commands and values over Bluetooth messages (protocol/relay_protocol.h).
#pragma once

#include <array>
#include <chrono>
#include <cstdint>
#include <deque>
#include <functional>
#include <map>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

#include "ftrobotx/Bluetooth.hpp"
#include "ftrobotx/Extension.hpp"
#include "ftrobotx/FtConnection.hpp"
#include "relay_protocol.h"

namespace ftrobotx
{

/// Bit masks for subscribe(): one bit per RelaySignal.
inline constexpr std::uint32_t kRelayInputs = 0xFFu << RELAY_SIG_I1;
inline constexpr std::uint32_t kRelayCounters = 0xFu << RELAY_SIG_C1;
inline constexpr std::uint32_t kRelayCounterLevels = 0xFu << RELAY_SIG_C1_LEVEL;
inline constexpr std::uint32_t kRelayButtons = 3u << RELAY_SIG_BUTTON_LEFT;
inline constexpr std::uint32_t kRelayMotorsDone = 0xFu << RELAY_SIG_M1_DONE;
inline constexpr std::uint32_t kRelayAll = (1u << RELAY_SIG_COUNT) - 1;

/// Talks to a model TX that runs hybrid/relay/tx/relay_model.c, through a relay TX in online mode. The functions
/// match FtConnection's; ExtensionIndex::Local is the model TX, Slave1 ... its extensions.
///
/// Commands are queued and sent one at a time; each is repeated until the model acknowledges it. Setting the same
/// output again before it went out only replaces the queued value, so calling setMotorOutput() in every loop is
/// fine. Values come from subscribe(). Call update() regularly: it runs the relay's online cycle too.
class RelayConnection
{
  public:
    struct Options
    {
        int channel = RELAY_CHANNEL;
        std::chrono::milliseconds ackTimeout{200}; // resend a command after this long without ACK
        int attempts = 5;
        std::chrono::milliseconds pingInterval{300}; // keeps the model's watchdog (1 s) quiet
        // Shortest time between two online cycles of the relay. Polled nonstop (~1000 cycles/s over USB), the relay
        // drops its Bluetooth connection to the model every half minute or so (measured); with 5 ms or more, never
        // in 9 minutes. Its inputs only change every 10 ms anyway.
        std::chrono::milliseconds relayPeriod{10};
    };

    struct Statistics
    {
        std::uint32_t commands = 0; // acknowledged
        std::uint32_t resends = 0;
        std::uint32_t failures = 0; // given up, or answered with an error status
        std::uint32_t valueFrames = 0;
        std::uint32_t lostValueFrames = 0; // from gaps in their sequence numbers
        double lastRoundTripMs = 0;        // command sent -> ACK received
    };

    /// Takes over the relay's Bluetooth callback.
    RelayConnection(FtConnection& relay, const BtAddress& model, Options options);
    RelayConnection(FtConnection& relay, const BtAddress& model) : RelayConnection(relay, model, Options{})
    {
    }
    RelayConnection(const RelayConnection&) = delete;
    RelayConnection& operator=(const RelayConnection&) = delete;

    /// Connects to the model over Bluetooth and checks its protocol version; blocks up to `timeout` (connecting
    /// takes a few seconds). Throws std::runtime_error on failure.
    void connect(std::chrono::milliseconds timeout = std::chrono::seconds(20));
    void disconnect();
    [[nodiscard]] bool connected() const
    {
        return connected_;
    }

    /// Controllers on the model's side (1 + extensions), and which extensions are connected (from connect()).
    [[nodiscard]] int controllers() const
    {
        return controllers_;
    }
    [[nodiscard]] const std::array<bool, 8>& extensionsOnline() const
    {
        return extensionsOnline_;
    }

    [[nodiscard]] FtConnection& relay()
    {
        return relay_;
    }

    // Outputs
    void setPwmOutput(ExtensionIndex index, PWM pwm, std::int16_t duty);
    void setMotorOutput(ExtensionIndex index, Motor motor, std::int16_t duty);
    void startMotorDistance(ExtensionIndex index, Motor motor, std::int16_t duty, std::uint16_t distance);
    void startMotorSync(ExtensionIndex index, Motor master, Motor slave, std::int16_t duty, std::uint16_t distance = 0,
                        SyncDirection direction = SyncDirection::Same);
    void stopMotor(ExtensionIndex index, Motor motor);
    void stopAllOutputs();
    void resetCounter(ExtensionIndex index, Counter counter);

    // Configuration
    void setInputMode(ExtensionIndex index, UniversalInput input, InputMode mode, bool digital = false);
    /// A pair of outputs as motor (true) or as two single outputs (false).
    void setMotorMode(ExtensionIndex index, Motor motor, bool motor_mode);
    void setCounterInverted(ExtensionIndex index, Counter counter, bool inverted);

    /// Shows a text (up to 98 characters) on the model's display.
    void showMessage(std::string_view text);

    /// Asks the model to send these signals (mask of kRelay... bits or 1 << RelaySignal) when they change, at most
    /// every `interval`, and all of them once a second. 0 stops it.
    void subscribe(ExtensionIndex index, std::uint32_t signals,
                   std::chrono::milliseconds interval = std::chrono::milliseconds(20));

    // Values from the subscriptions; 0 until the first report.
    [[nodiscard]] std::uint16_t value(ExtensionIndex index, RelaySignal signal) const;
    [[nodiscard]] std::uint16_t analogValue(ExtensionIndex index, UniversalInput input) const;
    [[nodiscard]] bool digitalValue(ExtensionIndex index, UniversalInput input) const;
    [[nodiscard]] std::uint16_t counterValue(ExtensionIndex index, Counter counter) const;
    [[nodiscard]] bool motorCommandDone(ExtensionIndex index, Motor motor) const;
    [[nodiscard]] std::optional<double> ntcTemperature(ExtensionIndex index, UniversalInput input, double r25 = kNtcR25,
                                                       double b = kNtcB) const;
    [[nodiscard]] std::optional<std::uint16_t> ultrasonicDistance(ExtensionIndex index, UniversalInput input) const;
    /// Whether a report for the controller arrived since subscribe().
    [[nodiscard]] bool hasValues(ExtensionIndex index) const;

    /// Runs the relay's online cycle, handles frames from the model, sends and repeats commands. Waits until
    /// Options::relayPeriod has passed since the last cycle.
    void update();

    /// True when no command is queued or waiting for its ACK.
    [[nodiscard]] bool idle() const
    {
        return queue_.empty() && !inFlight_;
    }

    /// Calls update() until every command is acknowledged; false after `timeout`.
    bool flush(std::chrono::milliseconds timeout = std::chrono::seconds(3));

    /// Called (from update()) when a command was refused by the model or given up after `attempts`.
    void setErrorCallback(std::function<void(const std::string&)> callback)
    {
        errorCallback_ = std::move(callback);
    }

    [[nodiscard]] const Statistics& statistics() const
    {
        return statistics_;
    }

  private:
    using Clock = std::chrono::steady_clock;

    struct Command
    {
        std::vector<std::uint8_t> frame;    // without seq: op, arguments (seq is inserted when sending)
        std::optional<std::uint32_t> key{}; // settings with the same key replace each other
        std::uint8_t seq = 0;
        int attempts = 0;
        Clock::time_point firstSent{};
        Clock::time_point lastSent{};
    };

    void queueSetting(std::uint32_t key, std::vector<std::uint8_t> frame);
    void queueCommand(std::vector<std::uint8_t> frame);
    /// Motor commands change motors beyond their own setting (sync partners): forget what is known to be in effect.
    void forgetMotorSettings(int ctrl);
    void sendNext();
    void transmit(Command& command);
    void handleFrame(const std::vector<std::uint8_t>& frame);
    void fail(const std::string& message);

    FtConnection& relay_;
    BtAddress model_;
    Options options_;
    bool connected_ = false;
    std::optional<BtStatus> connectResult_;
    int controllers_ = 0;
    std::array<bool, 8> extensionsOnline_{};

    std::deque<Command> queue_;
    std::optional<Command> inFlight_;
    std::map<std::uint32_t, std::vector<std::uint8_t>> inEffect_; // settings the model acknowledged, by key
    std::uint8_t nextSeq_ = 1;
    Clock::time_point lastActivity_;
    Clock::time_point nextRelayCycle_;
    std::optional<std::vector<std::uint8_t>> helloReply_;
    bool echoed_ = false; // a command came back unchanged (the model runs BtEcho)

    std::vector<std::vector<std::uint8_t>> received_;
    std::array<std::array<std::uint16_t, RELAY_SIG_COUNT>, 9> values_{};
    std::array<bool, 9> hasValues_{};
    std::optional<std::uint8_t> lastValuesSeq_;

    std::function<void(const std::string&)> errorCallback_;
    Statistics statistics_;
};

} // namespace ftrobotx
