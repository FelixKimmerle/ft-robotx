#include "ftrobotx/SerialPort.hpp"

#include <algorithm>
#include <array>
#include <cerrno>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <optional>
#include <string_view>
#include <system_error>

#include <fcntl.h>
#include <poll.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <termios.h>
#include <unistd.h>

#include "ftrobotx/Errors.hpp"

namespace ftrobotx
{

namespace
{

[[noreturn]] void throwErrno(const std::string& what)
{
    throw std::system_error(errno, std::generic_category(), what);
}

/// Waits until fd is ready for events or the deadline passes. Returns false on timeout.
bool waitFor(int fd, short events, std::chrono::steady_clock::time_point deadline)
{
    while (true)
    {
        auto remaining = std::chrono::ceil<std::chrono::milliseconds>(deadline - std::chrono::steady_clock::now());
        if (remaining.count() <= 0)
        {
            return false;
        }

        pollfd pfd{fd, events, 0};
        int result = ::poll(&pfd, 1, static_cast<int>(remaining.count()));
        if (result < 0)
        {
            if (errno == EINTR)
            {
                continue;
            }
            throwErrno("poll");
        }
        if (result == 0)
        {
            return false;
        }
        if ((pfd.revents & events) == 0 && (pfd.revents & (POLLERR | POLLHUP | POLLNVAL)) != 0)
        {
            throw std::runtime_error("Serial port disconnected");
        }
        return true;
    }
}

int openDevice(const std::string& name)
{
    // No O_NONBLOCK: for Bluetooth (/dev/rfcomm*) a blocking open waits until the link is up, while a non-blocking
    // one returns early and the device then reports a hang-up. Reads and writes never block anyway, because they
    // only happen after poll() and with VMIN = VTIME = 0.
    int fd = ::open(name.c_str(), O_RDWR | O_NOCTTY | O_CLOEXEC);
    if (fd < 0)
    {
        throwErrno("Cannot open " + name);
    }

    try
    {
        if (::ioctl(fd, TIOCEXCL) < 0)
        {
            throwErrno("Cannot get exclusive access to " + name);
        }

        termios tty{};
        if (::tcgetattr(fd, &tty) < 0)
        {
            throwErrno("tcgetattr");
        }
        ::cfmakeraw(&tty);
        tty.c_cflag |= CLOCAL | CREAD;
        tty.c_cflag &= ~static_cast<tcflag_t>(CRTSCTS | CSTOPB | PARENB);
        tty.c_iflag &= ~static_cast<tcflag_t>(IXON | IXOFF | IXANY);
        tty.c_cc[VMIN] = 0;
        tty.c_cc[VTIME] = 0;
        // Ignored by USB and Bluetooth, but a real UART needs something; 9600 is the .NET default.
        ::cfsetispeed(&tty, B9600);
        ::cfsetospeed(&tty, B9600);
        if (::tcsetattr(fd, TCSANOW, &tty) < 0)
        {
            throwErrno("tcsetattr");
        }
        ::tcflush(fd, TCIOFLUSH);
    }
    catch (...)
    {
        ::close(fd);
        throw;
    }
    return fd;
}

std::string readLine(const std::filesystem::path& path)
{
    std::ifstream file(path);
    std::string line;
    std::getline(file, line);
    return line;
}

/// USB vendor:product ID of the ROBO TX Controller.
constexpr std::string_view kRoboTxUsbId = "221d:1000";

/// Describes a /dev/ttyACM* or /dev/ttyUSB* device from its USB attributes in sysfs.
std::string describeUsbDevice(const std::string& name)
{
    std::error_code ec;
    std::filesystem::path device = std::filesystem::canonical("/sys/class/tty/" + name + "/device", ec);
    // Walk up from the tty's interface to the USB device, which has the IDs.
    for (int up = 0; !ec && up < 3 && device.has_parent_path(); ++up, device = device.parent_path())
    {
        std::string vendor = readLine(device / "idVendor");
        if (vendor.empty())
        {
            continue;
        }
        if (vendor + ":" + readLine(device / "idProduct") == kRoboTxUsbId)
        {
            return "ROBO TX Controller (USB)";
        }
        std::string product = readLine(device / "product");
        return (product.empty() ? "USB serial port" : product) + " (USB)";
    }
    return "Serial port";
}

#ifdef __linux__

constexpr int kBtProtoRfcomm = 3; // BTPROTO_RFCOMM from <bluetooth/bluetooth.h>

/// The TX offers its "Serial Port" service on RFCOMM channel 1 (its SDP record says so).
constexpr std::uint8_t kRoboTxRfcommChannel = 1;

/// struct sockaddr_rc from <bluetooth/rfcomm.h>, so building doesn't need the BlueZ headers.
struct SockaddrRc
{
    sa_family_t family;
    std::array<std::uint8_t, 6> address; // byte order reversed compared to the "XX:XX:..." notation
    std::uint8_t channel;
};

std::optional<std::array<std::uint8_t, 6>> parseMac(std::string_view text)
{
    std::array<std::uint8_t, 6> address{};
    if (text.size() != 17)
    {
        return std::nullopt;
    }
    for (std::size_t i = 0; i < 6; ++i)
    {
        unsigned value = 0;
        if (std::sscanf(std::string(text.substr(i * 3, 2)).c_str(), "%2x", &value) != 1 ||
            (i < 5 && text[i * 3 + 2] != ':'))
        {
            return std::nullopt;
        }
        address[5 - i] = static_cast<std::uint8_t>(value);
    }
    return address;
}

/// Connects to a paired controller directly; unlike /dev/rfcomm* this needs neither root nor the uucp group.
int connectBluetooth(std::string_view mac)
{
    std::optional<std::array<std::uint8_t, 6>> address = parseMac(mac);
    if (!address)
    {
        throw std::invalid_argument("Invalid Bluetooth address \"" + std::string(mac) + "\", expected bt:XX:XX:XX:XX:XX:XX");
    }

    int fd = ::socket(AF_BLUETOOTH, SOCK_STREAM | SOCK_CLOEXEC, kBtProtoRfcomm);
    if (fd < 0)
    {
        throwErrno("Cannot create Bluetooth socket");
    }
    SockaddrRc target{AF_BLUETOOTH, *address, kRoboTxRfcommChannel};
    // Blocks until the link is up (about 3.5 s) or the Bluetooth stack gives up.
    if (::connect(fd, reinterpret_cast<const sockaddr*>(&target), sizeof(target)) < 0)
    {
        int error = errno;
        ::close(fd);
        throw std::system_error(error, std::generic_category(), "Cannot connect to " + std::string(mac));
    }
    return fd;
}

/// Paired Bluetooth devices that look like a ROBO TX, as reported by BlueZ.
std::vector<SerialPort::PortInfo> pairedControllers()
{
    std::vector<SerialPort::PortInfo> ports;
    FILE* pipe = ::popen("timeout 3 bluetoothctl devices Paired 2>/dev/null", "r");
    if (pipe == nullptr)
    {
        return ports;
    }
    char line[256];
    while (std::fgets(line, sizeof(line), pipe) != nullptr)
    {
        // "Device 00:13:7B:53:DB:5E ROBO TX-A"
        std::string_view entry(line);
        if (!entry.starts_with("Device ") || entry.size() < 25)
        {
            continue;
        }
        std::string mac(entry.substr(7, 17));
        std::string name(entry.substr(25));
        while (!name.empty() && (name.back() == '\n' || name.back() == '\r'))
        {
            name.pop_back();
        }
        if (name.starts_with("ROBO TX") && parseMac(mac))
        {
            ports.push_back({"bt:" + mac, name + " (Bluetooth)"});
        }
    }
    ::pclose(pipe);
    return ports;
}

#endif

} // namespace

struct SerialPort::Impl
{
    int fd = -1;
    bool socket = false;
    std::chrono::milliseconds timeout;
};

SerialPort::SerialPort(const std::string& name, std::chrono::milliseconds timeout) : impl_(std::make_unique<Impl>())
{
    impl_->timeout = timeout;
    if (name.starts_with("bt:"))
    {
#ifdef __linux__
        impl_->fd = connectBluetooth(std::string_view(name).substr(3));
        impl_->socket = true;
#else
        throw std::invalid_argument("Direct Bluetooth connections (bt:...) are only supported on Linux");
#endif
    }
    else
    {
        impl_->fd = openDevice(name);
    }
}

SerialPort::~SerialPort()
{
    ::close(impl_->fd);
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
        if (!waitFor(impl_->fd, POLLOUT, deadline))
        {
            throw TimeoutError("Write to serial port timed out");
        }
        // send() with MSG_NOSIGNAL, so a dropped Bluetooth link is an error instead of a SIGPIPE.
        ssize_t written = impl_->socket ? ::send(impl_->fd, data.data(), data.size(), MSG_NOSIGNAL)
                                        : ::write(impl_->fd, data.data(), data.size());
        if (written < 0)
        {
            if (errno == EINTR || errno == EAGAIN)
            {
                continue;
            }
            throwErrno("write");
        }
        data = data.subspan(static_cast<std::size_t>(written));
    }
}

std::size_t SerialPort::read(std::span<std::uint8_t> buffer)
{
    auto deadline = std::chrono::steady_clock::now() + impl_->timeout;
    std::span<std::uint8_t> data = buffer;
    while (!data.empty())
    {
        if (!waitFor(impl_->fd, POLLIN, deadline))
        {
            break;
        }
        ssize_t received = ::read(impl_->fd, data.data(), data.size());
        if (received < 0)
        {
            if (errno == EINTR || errno == EAGAIN)
            {
                continue;
            }
            throwErrno("read");
        }
        if (received == 0)
        {
            throw std::runtime_error("Serial port disconnected");
        }
        data = data.subspan(static_cast<std::size_t>(received));
    }
    return buffer.size() - data.size();
}

std::vector<SerialPort::PortInfo> SerialPort::listPorts()
{
#ifdef __APPLE__
    constexpr std::string_view prefixes[] = {"cu.usbmodem", "cu.usbserial", "cu.Bluetooth"};
#else
    constexpr std::string_view prefixes[] = {"ttyACM", "ttyUSB", "rfcomm"};
#endif

    std::vector<PortInfo> ports;
    std::error_code ec;
    for (const auto& entry : std::filesystem::directory_iterator("/dev", ec))
    {
        std::string name = entry.path().filename().string();
        if (std::ranges::any_of(prefixes, [&](std::string_view prefix) { return name.starts_with(prefix); }))
        {
            std::string description = name.starts_with("rfcomm") ? "Bluetooth (rfcomm)" : describeUsbDevice(name);
            ports.push_back({entry.path().string(), description});
        }
    }
    std::ranges::sort(ports, {}, &PortInfo::name);

#ifdef __linux__
    std::vector<PortInfo> bluetooth = pairedControllers();
    ports.insert(ports.end(), bluetooth.begin(), bluetooth.end());
#endif

    // ROBO TX controllers first.
    std::ranges::stable_partition(ports, [](const PortInfo& port) { return port.description.starts_with("ROBO TX"); });
    return ports;
}

} // namespace ftrobotx
