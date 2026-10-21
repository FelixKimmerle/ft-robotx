#include "ftrobotx/SerialPort.hpp"

#include <algorithm>
#include <cstring>
#include <stdexcept>
#include <string_view>
#include <system_error>

#define WIN32_LEAN_AND_MEAN
#define NOMINMAX
#include <windows.h>

#include "ftrobotx/Errors.hpp"

namespace ftrobotx
{

namespace
{

[[noreturn]] void throwLastError(const std::string& what)
{
    throw std::system_error(static_cast<int>(::GetLastError()), std::system_category(), what);
}

DWORD remainingMs(std::chrono::steady_clock::time_point deadline)
{
    auto remaining = std::chrono::ceil<std::chrono::milliseconds>(deadline - std::chrono::steady_clock::now());
    return remaining.count() > 0 ? static_cast<DWORD>(remaining.count()) : 0;
}

void setTimeouts(HANDLE handle, DWORD readMs, DWORD writeMs)
{
    COMMTIMEOUTS timeouts{};
    timeouts.ReadTotalTimeoutConstant = readMs;
    timeouts.WriteTotalTimeoutConstant = writeMs;
    if (!::SetCommTimeouts(handle, &timeouts))
    {
        throwLastError("SetCommTimeouts");
    }
}

} // namespace

struct SerialPort::Impl
{
    HANDLE handle = INVALID_HANDLE_VALUE;
    std::chrono::milliseconds timeout;
};

SerialPort::SerialPort(const std::string& name, std::chrono::milliseconds timeout) : impl_(std::make_unique<Impl>())
{
    impl_->timeout = timeout;
    if (name.starts_with("bt:"))
    {
        throw std::invalid_argument("Direct Bluetooth connections (bt:...) are only supported on Linux; "
                                    "use the COM port Windows created when pairing");
    }

    // The \\.\ prefix is required for COM10 and above.
    std::string path = name.starts_with(R"(\\.\)") ? name : R"(\\.\)" + name;
    impl_->handle = ::CreateFileA(path.c_str(), GENERIC_READ | GENERIC_WRITE, 0, nullptr, OPEN_EXISTING, 0, nullptr);
    if (impl_->handle == INVALID_HANDLE_VALUE)
    {
        throwLastError("Cannot open " + name);
    }

    try
    {
        DCB dcb{};
        dcb.DCBlength = sizeof(dcb);
        if (!::GetCommState(impl_->handle, &dcb))
        {
            throwLastError("GetCommState");
        }
        // Same as the .NET SerialPort defaults: 9600 8N1, no handshake, DTR and RTS off.
        dcb.BaudRate = CBR_9600;
        dcb.ByteSize = 8;
        dcb.Parity = NOPARITY;
        dcb.StopBits = ONESTOPBIT;
        dcb.fBinary = TRUE;
        dcb.fParity = FALSE;
        dcb.fOutxCtsFlow = FALSE;
        dcb.fOutxDsrFlow = FALSE;
        dcb.fDtrControl = DTR_CONTROL_DISABLE;
        dcb.fRtsControl = RTS_CONTROL_DISABLE;
        dcb.fDsrSensitivity = FALSE;
        dcb.fOutX = FALSE;
        dcb.fInX = FALSE;
        dcb.fAbortOnError = FALSE;
        if (!::SetCommState(impl_->handle, &dcb))
        {
            throwLastError("SetCommState");
        }
        ::PurgeComm(impl_->handle, PURGE_RXCLEAR | PURGE_TXCLEAR);
    }
    catch (...)
    {
        ::CloseHandle(impl_->handle);
        throw;
    }
}

SerialPort::~SerialPort()
{
    ::CloseHandle(impl_->handle);
}

void SerialPort::setTimeout(std::chrono::milliseconds timeout)
{
    impl_->timeout = timeout;
}

void SerialPort::write(std::span<const std::uint8_t> data)
{
    auto deadline = std::chrono::steady_clock::now() + impl_->timeout;
    while (!data.empty())
    {
        DWORD timeLeft = remainingMs(deadline);
        if (timeLeft == 0)
        {
            throw TimeoutError("Write to serial port timed out");
        }
        setTimeouts(impl_->handle, 0, timeLeft);

        DWORD written = 0;
        if (!::WriteFile(impl_->handle, data.data(), static_cast<DWORD>(data.size()), &written, nullptr))
        {
            throwLastError("WriteFile");
        }
        data = data.subspan(written);
    }
}

std::size_t SerialPort::read(std::span<std::uint8_t> buffer)
{
    auto deadline = std::chrono::steady_clock::now() + impl_->timeout;
    std::span<std::uint8_t> data = buffer;
    while (!data.empty())
    {
        DWORD timeLeft = remainingMs(deadline);
        if (timeLeft == 0)
        {
            break;
        }
        setTimeouts(impl_->handle, timeLeft, 0);

        DWORD received = 0;
        if (!::ReadFile(impl_->handle, data.data(), static_cast<DWORD>(data.size()), &received, nullptr))
        {
            throwLastError("ReadFile");
        }
        data = data.subspan(received);
    }
    return buffer.size() - data.size();
}

std::vector<SerialPort::PortInfo> SerialPort::listPorts()
{
    std::vector<char> buffer(16 * 1024);
    DWORD length = 0;
    while ((length = ::QueryDosDeviceA(nullptr, buffer.data(), static_cast<DWORD>(buffer.size()))) == 0)
    {
        if (::GetLastError() != ERROR_INSUFFICIENT_BUFFER)
        {
            throwLastError("QueryDosDevice");
        }
        buffer.resize(buffer.size() * 2);
    }

    std::vector<PortInfo> ports;
    for (const char* name = buffer.data(); *name != '\0'; name += std::strlen(name) + 1)
    {
        std::string_view device(name);
        if (device.size() > 3 && device.starts_with("COM") &&
            std::ranges::all_of(device.substr(3), [](char c) { return c >= '0' && c <= '9'; }))
        {
            // Paired Bluetooth controllers appear as COM ports too.
            ports.push_back({std::string(device), "Serial port"});
        }
    }
    std::ranges::sort(ports, [](const PortInfo& a, const PortInfo& b) {
        return a.name.size() != b.name.size() ? a.name.size() < b.name.size() : a.name < b.name;
    });
    return ports;
}

} // namespace ftrobotx
