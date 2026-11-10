// Conversions for the values of fischertechnik sensors on the universal inputs.
#pragma once

#include <cmath>
#include <cstdint>
#include <optional>

namespace ftrobotx
{

/// Analog resistance mode (InputMode::Resistance, not digital): 10 ... 4999 Ohm; 5000 means overload (open input).
inline constexpr std::uint16_t kResistanceOverload = 5000;

/// Ultrasonic mode (InputMode::Ultrasonic): 2 ... 1023 cm.
inline constexpr std::uint16_t kUltrasonicMin = 2;
inline constexpr std::uint16_t kUltrasonicMax = 1023;
inline constexpr std::uint16_t kUltrasonicOverload = 1024; // nothing in range
inline constexpr std::uint16_t kUltrasonicNotPresent = 4096;

/// The fischertechnik NTC: 1.5 kOhm at 25 °C, B constant 3900 K. Each sensor differs a little; pass your own
/// values to ntcKelvin()/ntcCelsius() to calibrate.
inline constexpr double kNtcR25 = 1500.0;
inline constexpr double kNtcB = 3900.0;

inline constexpr double kZeroCelsius = 273.15; // K

/// Temperature in Kelvin of an NTC from its resistance (B parameter equation). No value for 0 (right after switching
/// the input mode there is no measurement yet) and for an overload (no sensor connected).
[[nodiscard]] inline std::optional<double> ntcKelvin(std::uint16_t ohm, double r25 = kNtcR25, double b = kNtcB)
{
    if (ohm == 0 || ohm >= kResistanceOverload)
    {
        return std::nullopt;
    }
    return 1.0 / (1.0 / (25.0 + kZeroCelsius) + std::log(ohm / r25) / b);
}

/// Temperature in °C of an NTC from its resistance; see ntcKelvin().
[[nodiscard]] inline std::optional<double> ntcCelsius(std::uint16_t ohm, double r25 = kNtcR25, double b = kNtcB)
{
    std::optional<double> kelvin = ntcKelvin(ohm, r25, b);
    return kelvin ? std::optional<double>(*kelvin - kZeroCelsius) : std::nullopt;
}

/// Distance in cm from the ultrasonic sensor's value. No value if there is no measurement yet (0, right after
/// switching the input mode), nothing is in range (1024) or no sensor is connected (4096).
[[nodiscard]] inline std::optional<std::uint16_t> ultrasonicCm(std::uint16_t value)
{
    if (value < kUltrasonicMin || value > kUltrasonicMax)
    {
        return std::nullopt;
    }
    return value;
}

} // namespace ftrobotx
