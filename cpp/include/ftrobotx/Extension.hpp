#pragma once

#include <array>
#include <cstdint>
#include <optional>
#include <string>

namespace ftrobotx
{

inline constexpr std::size_t kMotors = 4;
inline constexpr std::size_t kUniversalInputs = 8;

enum class ExtensionIndex
{
    Local,
    Slave1,
    Slave2,
    Slave3,
    Slave4,
    Slave5,
    Slave6,
    Slave7,
};

enum class Motor
{
    M1,
    M2,
    M3,
    M4,
};

enum class PWM
{
    O1,
    O2,
    O3,
    O4,
    O5,
    O6,
    O7,
    O8,
};

enum class Counter
{
    C1,
    C2,
    C3,
    C4,
};

enum class UniversalInput
{
    I1,
    I2,
    I3,
    I4,
    I5,
    I6,
    I7,
    I8,
};

enum class InputMode : std::uint8_t
{
    Voltage,
    Resistance,
    Resistance2,
    Ultrasonic,
    Invalid,
};

struct UniversalInputConfig
{
    InputMode mode = InputMode::Resistance;
    bool digital = true;

    [[nodiscard]] std::uint8_t toByte() const
    {
        return static_cast<std::uint8_t>((digital ? 0x80 : 0x00) | static_cast<std::uint8_t>(mode));
    }
};

/// Versions from the controller's info block (FT_VERSION in ROBO_TX_FW.h).
struct FirmwareVersion
{
    char hardwareRevision = '?'; // 'A', 'B' or 'C'
    std::uint8_t firmwareMajor = 0;
    std::uint8_t firmwareMinor = 0;
    std::uint8_t dllVersion = 0;
    std::uint8_t transferAreaMajor = 0;
    std::uint8_t transferAreaMinor = 0;

    /// Firmware version as shown by fischertechnik, e.g. "1.30".
    [[nodiscard]] std::string toString() const
    {
        std::string minor = std::to_string(firmwareMinor);
        return std::to_string(firmwareMajor) + "." + (minor.size() < 2 ? "0" + minor : minor);
    }
};

class Config
{
public:
    Config()
    {
        motorConfig_.fill(1);
        counterConfig_.fill(1);
    }

    [[nodiscard]] UniversalInputConfig universalInputConfig(std::size_t index) const { return universalInputConfigs_.at(index); }
    void setUniversalInputConfig(std::size_t index, UniversalInputConfig value)
    {
        universalInputConfigs_.at(index) = value;
        changed = true;
    }

    [[nodiscard]] std::uint8_t motorConfig(std::size_t index) const { return motorConfig_.at(index); }
    void setMotorConfig(std::size_t index, std::uint8_t value)
    {
        motorConfig_.at(index) = value;
        changed = true;
    }

    [[nodiscard]] std::uint8_t counterConfig(std::size_t index) const { return counterConfig_.at(index); }
    void setCounterConfig(std::size_t index, std::uint8_t value)
    {
        counterConfig_.at(index) = value;
        changed = true;
    }

    bool changed = true;

private:
    std::array<UniversalInputConfig, kUniversalInputs> universalInputConfigs_{};
    std::array<std::uint8_t, kMotors> motorConfig_{};
    std::array<std::uint8_t, kMotors> counterConfig_{};
};

class Input
{
public:
    [[nodiscard]] std::uint16_t universalInput(std::size_t index) const { return universalInputs_.at(index); }
    void setUniversalInput(std::size_t index, std::uint16_t value) { set(universalInputs_.at(index), value); }

    [[nodiscard]] std::uint8_t counterInput(std::size_t index) const { return counterInputs_.at(index); }
    void setCounterInput(std::size_t index, std::uint8_t value) { set(counterInputs_.at(index), value); }

    [[nodiscard]] std::uint16_t counterValue(std::size_t index) const { return counterValues_.at(index); }
    void setCounterValue(std::size_t index, std::uint16_t value) { set(counterValues_.at(index), value); }

    [[nodiscard]] std::uint16_t displayButtonLeft() const { return displayButtonLeft_; }
    void setDisplayButtonLeft(std::uint16_t value) { set(displayButtonLeft_, value); }

    [[nodiscard]] std::uint16_t displayButtonRight() const { return displayButtonRight_; }
    void setDisplayButtonRight(std::uint16_t value) { set(displayButtonRight_, value); }

    [[nodiscard]] std::uint16_t motorReachedDestination(std::size_t index) const { return motorReachedDestination_.at(index); }
    void setMotorReachedDestination(std::size_t index, std::uint16_t value) { set(motorReachedDestination_.at(index), value); }

    [[nodiscard]] std::uint16_t resetCounter(std::size_t index) const { return resetCounters_.at(index); }
    void setResetCounter(std::size_t index, std::uint16_t value) { set(resetCounters_.at(index), value); }

    /// Set whenever an input value changes; reset it after handling the change.
    bool changed = true;

private:
    template <typename T>
    void set(T& field, T value)
    {
        changed |= field != value;
        field = value;
    }

    std::array<std::uint16_t, kUniversalInputs> universalInputs_{};
    std::array<std::uint8_t, kMotors> counterInputs_{};
    std::array<std::uint16_t, kMotors> counterValues_{};
    std::array<std::uint16_t, kMotors> motorReachedDestination_{};
    std::array<std::uint16_t, kMotors> resetCounters_{};
    std::uint16_t displayButtonLeft_ = 0;
    std::uint16_t displayButtonRight_ = 0;
};

struct Output
{
    std::array<std::uint16_t, kMotors> counterResetCommandId{};
    std::array<std::uint8_t, kMotors> motorMasterValues{};
    std::array<std::int16_t, kMotors * 2> pwmOutputValues{};
    std::array<std::uint16_t, kMotors> motorDistanceValues{};
    std::array<std::uint16_t, kMotors> motorCommandId{};

    /// Stops a motor like ftMscLib's StopMotorExCmd: duty and distance to 0, and a new motor command, so a
    /// running distance/synchronization command is cancelled too.
    void stopMotor(std::size_t motor)
    {
        pwmOutputValues.at(2 * motor) = 0;
        pwmOutputValues.at(2 * motor + 1) = 0;
        motorDistanceValues.at(motor) = 0;
        ++motorCommandId.at(motor);
    }

    /// Stops all motors and outputs.
    void stopAll()
    {
        for (std::size_t motor = 0; motor < kMotors; ++motor)
        {
            stopMotor(motor);
        }
    }
};

/// A TX controller (local master or one of the extension slaves) and its transfer area.
struct Extension
{
    explicit Extension(std::uint32_t index) : transferAreaIndex(index) {}

    std::uint32_t transferAreaIndex;

    std::string name;
    std::optional<FirmwareVersion> version;
    std::string macAddress;

    Config config;
    Output output;
    Input input;
};

} // namespace ftrobotx
