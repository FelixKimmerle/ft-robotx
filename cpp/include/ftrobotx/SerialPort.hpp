#pragma once

#include <chrono>
#include <cstdint>
#include <memory>
#include <span>
#include <string>
#include <vector>

namespace ftrobotx
{

/// Raw 8N1 serial port. USB (/dev/ttyACM*, COMx) and Bluetooth SPP (/dev/rfcomm*, COMx) look the same here.
/// On Linux, "bt:XX:XX:XX:XX:XX:XX" connects to a paired controller directly over Bluetooth, without rfcomm bind.
class SerialPort
{
public:
    struct PortInfo
    {
        std::string name;        // what to pass to the constructor
        std::string description; // e.g. "ROBO TX Controller (USB)" or "ROBO TX-A (Bluetooth)"
    };

    SerialPort(const std::string& name, std::chrono::milliseconds timeout);
    ~SerialPort();

    SerialPort(const SerialPort&) = delete;
    SerialPort& operator=(const SerialPort&) = delete;

    void setTimeout(std::chrono::milliseconds timeout);

    void write(std::span<const std::uint8_t> data);

    /// Reads until the buffer is full or the timeout expires; returns the number of bytes read.
    std::size_t read(std::span<std::uint8_t> buffer);

    /// Ports a TX controller may be connected to, ROBO TX controllers first: USB serial ports, bound rfcomm
    /// devices, and (Linux) paired ROBO TX controllers for a direct Bluetooth connection. Can take a moment, as it
    /// asks BlueZ for the paired devices.
    static std::vector<PortInfo> listPorts();

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

} // namespace ftrobotx
