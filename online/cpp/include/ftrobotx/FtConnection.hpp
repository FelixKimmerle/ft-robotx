#pragma once

#include <array>
#include <chrono>
#include <cstdint>
#include <functional>
#include <optional>
#include <span>
#include <string>
#include <string_view>
#include <vector>

#include "ftrobotx/Bluetooth.hpp"
#include "ftrobotx/Extension.hpp"
#include "ftrobotx/Sensors.hpp"
#include "ftrobotx/SerialPort.hpp"

namespace ftrobotx
{

class Packet;
class Response;

/// Serial connection to a fischertechnik ROBO TX Controller and its extensions.
class FtConnection
{
  public:
    explicit FtConnection(const std::string& portName, std::size_t numExtensions = 1,
                          std::chrono::milliseconds timeout = std::chrono::milliseconds(1000));

    [[nodiscard]] std::span<Extension> extensions()
    {
        return extensions_;
    }
    [[nodiscard]] std::span<const Extension> extensions() const
    {
        return extensions_;
    }
    [[nodiscard]] Extension& extension(ExtensionIndex index)
    {
        return extensions_.at(static_cast<std::size_t>(index));
    }
    [[nodiscard]] const Extension& extension(ExtensionIndex index) const
    {
        return extensions_.at(static_cast<std::size_t>(index));
    }

    void setPwmOutput(ExtensionIndex index, PWM pwm, std::int16_t duty);

    /// Runs a motor endlessly at the given duty (-512 ... 512, the sign gives the direction). Ends a distance or
    /// synchronization command of the motor; a motor it was synchronized with is stopped.
    void setMotorOutput(ExtensionIndex index, Motor motor, std::int16_t duty);

    /// Runs a motor with encoder for `distance` counter pulses, then the firmware brakes it (ftMscLib's
    /// StartMotorExCmd without slave). Resets the motor's counter. motorCommandDone() and the motor reached
    /// callback tell when it is there.
    void startMotorDistance(ExtensionIndex index, Motor motor, std::int16_t duty, std::uint16_t distance);

    /// Runs two motors with encoders synchronized (ftMscLib's StartMotorExCmd): the firmware keeps their counters
    /// equal by slowing down the faster one, in either direction. With a distance, both stop after `distance`
    /// pulses; with 0 they run until stopped. Resets both counters. Each motor's counter must be on the counter
    /// input with the same number (M1 on C1, ...).
    void startMotorSync(ExtensionIndex index, Motor master, Motor slave, std::int16_t duty, std::uint16_t distance = 0,
                        SyncDirection direction = SyncDirection::Same);

    /// Stops a motor (ftMscLib's StopMotorExCmd) and a motor synchronized with it. Cancelling a running distance or
    /// synchronization command resets the counters (it is a new motor command); otherwise they keep their values.
    void stopMotor(ExtensionIndex index, Motor motor);

    /// True once the motor finished its last distance command (startMotorDistance(), startMotorSync() with a
    /// distance); also true for a motor without one. Updated by update().
    [[nodiscard]] bool motorCommandDone(ExtensionIndex index, Motor motor) const;

    /// Requests a reset of the counter to 0. Takes effect with the next update().
    void resetCounter(ExtensionIndex index, Counter counter);

    /// True once the last counter reset was carried out and counterValue() shows it.
    [[nodiscard]] bool counterResetDone(ExtensionIndex index, Counter counter) const;

    /// Pulses counted since the last reset or motor command, in either direction. The encoder motors give 3
    /// pulses per turn of the motor, 63 1/3 per turn of the output shaft.
    [[nodiscard]] std::uint16_t counterValue(ExtensionIndex index, Counter counter) const;

    /// Called from update() when a motor finished a distance command (ftMscLib's SetCBMotorExReached); for a
    /// synchronized pair, once per motor. The callback may start the next command.
    void setMotorReachedCallback(std::function<void(ExtensionIndex, Motor)> callback)
    {
        motorReachedCallback_ = std::move(callback);
    }

    /// Called from update() when a counter reset was carried out (ftMscLib's SetCBCounterResetted).
    void setCounterResetCallback(std::function<void(ExtensionIndex, Counter)> callback)
    {
        counterResetCallback_ = std::move(callback);
    }

    /// Shows a text (up to 98 ASCII characters) on the controller's display right away; an empty text clears it.
    void showMessage(ExtensionIndex index, std::string_view text);

    /// Asks the master which extensions are connected: element 0 is extension 1, ..., element 7 extension 8.
    [[nodiscard]] std::array<bool, 8> extensionsOnline();

    /// Stops all motors and outputs of all extensions, including running distance and synchronization commands.
    void stopAllOutputs();

    /// Configures a universal input (ftMscLib's SetFtUniConfig); sent with the next update(). The value reads 0 until
    /// the first measurement in the new mode (a few hundred milliseconds for the ultrasonic sensor and the NTC).
    void setInputMode(ExtensionIndex index, UniversalInput input, InputMode mode, bool digital = false);

    [[nodiscard]] bool digitalValue(ExtensionIndex index, UniversalInput input);
    [[nodiscard]] std::uint16_t analogValue(ExtensionIndex index, UniversalInput input);

    /// Temperature in °C of an NTC on an input in analog resistance mode; see ntcCelsius() in Sensors.hpp. No value
    /// before the first measurement or without sensor.
    [[nodiscard]] std::optional<double> ntcTemperature(ExtensionIndex index, UniversalInput input, double r25 = kNtcR25,
                                                       double b = kNtcB);

    /// Distance in cm from an ultrasonic sensor on an input in ultrasonic mode; see ultrasonicCm() in Sensors.hpp.
    [[nodiscard]] std::optional<std::uint16_t> ultrasonicDistance(ExtensionIndex index, UniversalInput input);

    /// Sends the configuration if it changed, then the current output values, and reads back the inputs of all
    /// extensions. Callbacks (motor, counter reset, Bluetooth) are called from here.
    void update();

    // Bluetooth messages between ROBO TX Controllers (ftMscLib's Bluetooth Message API), see hybrid/relay/README.md.
    // The commands only send the request; results and everything the controller reports by itself (received
    // messages, incoming connections, scan results) come through the callbacks, called from update(). Channels are
    // 1 ... 8. Typical use: the receiver calls btListen() with the sender's address, the sender btConnect(); once
    // connected (events ConnectionIndication / Success), the receiver calls btStartReceive(). btConnect() to the
    // controller's own address gives a loopback connection, for testing with one controller.

    /// Searches for Bluetooth devices; results come through the scan callback (the controller itself first).
    void btStartScan();
    void btCancelScan();

    void btConnect(int channel, const BtAddress& address);
    void btDisconnect(int channel);

    /// Accepts a connection from `remote` on the channel.
    void btListen(int channel, const BtAddress& remote);
    void btStopListen(int channel);

    /// Reports received messages through the Bluetooth callback. Only works on a connected channel.
    void btStartReceive(int channel);
    void btStopReceive(int channel);

    /// Sends up to kBtMaxMessage (16) bytes; the receiving controller drops anything beyond that. The send callback
    /// event (status Success) confirms it.
    void btSend(int channel, std::span<const std::uint8_t> message);
    void btSend(int channel, std::string_view message);

    /// Called from update() for every Bluetooth event.
    void setBtCallback(std::function<void(const BtEvent&)> callback)
    {
        btCallback_ = std::move(callback);
    }

    /// Called from update() for every scan status and found device.
    void setBtScanCallback(std::function<void(const BtScanResult&)> callback)
    {
        btScanCallback_ = std::move(callback);
    }

  private:
    void sendConfig();
    void exchange();
    void notify();
    /// Sends a packet without waiting for a reply (Bluetooth commands; their replies come as events).
    void sendRequest(Packet& packet);
    /// Reads the reply (telecode `expectedCode`) for `response`, queueing Bluetooth packets that come before it.
    [[nodiscard]] std::vector<std::uint8_t> readReply(std::int32_t expectedCode, const Response& response);
    /// Reads the next packet from the controller, skipping bytes until the 02 55 start.
    [[nodiscard]] std::vector<std::uint8_t> readPacket(std::size_t expectedLength);
    /// Queues a Bluetooth reply/report for the callbacks.
    void queueBtReply(std::span<const std::uint8_t> bytes);
    /// Runs a motor command on the extension and forgets the pending reports of the motors it gave a new command.
    template <typename Command> void motorCommand(ExtensionIndex index, Command command);
    void sendPacket(Packet& packet, Response& response);

    SerialPort serialPort_;
    std::vector<Extension> extensions_;
    std::uint16_t sessionId_ = 0;
    std::uint16_t transactionId_ = 0;
    std::vector<std::uint8_t> messageIds_; // last message id per controller
    // commands whose completion the callbacks still have to report, per controller
    std::vector<std::array<bool, kMotors>> motorPending_;
    enum class ResetState : std::uint8_t
    {
        Done,
        Requested,
        Confirmed, // the firmware echoed the reset id; the counter value follows with the next exchange
    };
    std::vector<std::array<ResetState, kMotors>> counterResets_;
    std::function<void(ExtensionIndex, Motor)> motorReachedCallback_;
    std::function<void(ExtensionIndex, Counter)> counterResetCallback_;
    // Bluetooth reports read while waiting for other replies; the callbacks get them in update(). Only the master's
    // radio is reachable this way.
    std::vector<BtEvent> btEvents_;
    std::vector<BtScanResult> btScanResults_;
    std::function<void(const BtEvent&)> btCallback_;
    std::function<void(const BtScanResult&)> btScanCallback_;
    bool trace_ = false;
};

} // namespace ftrobotx
