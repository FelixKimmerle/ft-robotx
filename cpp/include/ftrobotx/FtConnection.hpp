#pragma once

#include <chrono>
#include <cstdint>
#include <span>
#include <string>
#include <vector>

#include "ftrobotx/Extension.hpp"
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

    [[nodiscard]] std::span<Extension> extensions() { return extensions_; }
    [[nodiscard]] std::span<const Extension> extensions() const { return extensions_; }
    [[nodiscard]] Extension& extension(ExtensionIndex index) { return extensions_.at(static_cast<std::size_t>(index)); }

    void setPwmOutput(ExtensionIndex index, PWM pwm, std::int16_t duty);
    void setMotorOutput(ExtensionIndex index, Motor motor, std::int16_t duty);
    void setMotorDistance(ExtensionIndex index, Motor motor, std::int16_t duty, std::uint16_t distance);
    void setMotorSync(ExtensionIndex index, Motor motor, Motor other, std::int16_t duty, std::uint16_t distance = 0);

    /// Requests a reset of the counter to 0. Takes effect with the next update().
    void resetCounter(ExtensionIndex index, Counter counter);

    /// Stops all motors and outputs of all extensions, including running distance commands. Takes effect with the
    /// next update().
    void stopAllOutputs();

    [[nodiscard]] bool digitalValue(ExtensionIndex index, UniversalInput input);
    [[nodiscard]] std::uint16_t analogValue(ExtensionIndex index, UniversalInput input);

    /// Sends the configuration if it changed, then the current output values, and reads back the inputs of all
    /// extensions.
    void update();

private:
    void sendConfig();
    void sendPacket(Packet& packet, Response& response);

    SerialPort serialPort_;
    std::vector<Extension> extensions_;
    std::uint16_t sessionId_ = 0;
    std::uint16_t transactionId_ = 0;
    bool trace_ = false;
};

} // namespace ftrobotx
