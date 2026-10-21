#include "ftrobotx/FtConnection.hpp"

#include <algorithm>
#include <cstdlib>
#include <iostream>
#include <random>
#include <string_view>

#include "ftrobotx/Errors.hpp"
#include "ftrobotx/Packets.hpp"

namespace ftrobotx
{

namespace
{

std::string toHex(std::span<const std::uint8_t> bytes)
{
    constexpr char digits[] = "0123456789ABCDEF";
    std::string result;
    for (std::uint8_t b : bytes)
    {
        result += digits[b >> 4];
        result += digits[b & 0xF];
    }
    return result;
}

/// Set the environment variable FT_TRACE=1 to log every packet to stderr.
bool traceEnabled()
{
    const char* value = std::getenv("FT_TRACE");
    return value != nullptr && *value != '\0' && std::string_view(value) != "0";
}

/// Over Bluetooth the link is only established with the first packet, which takes a few seconds (about 3.5 s
/// measured), so the first reply gets more time than the rest.
constexpr std::chrono::milliseconds kFirstReplyTimeout{10000};

} // namespace

FtConnection::FtConnection(const std::string& portName, std::size_t numExtensions, std::chrono::milliseconds timeout)
    : serialPort_(portName, std::max(timeout, kFirstReplyTimeout)), trace_(traceEnabled())
{
    EchoPacket echoPacket;
    EchoResponse echoResponse;
    sendPacket(echoPacket, echoResponse);
    if (!echoResponse.success())
    {
        throw ProtocolError("Echo failed!");
    }
    serialPort_.setTimeout(timeout);

    // The firmware ignores a message with the same id as the previous one, which may come from an earlier
    // connection; start at a random id so that's unlikely.
    std::uint8_t firstId = static_cast<std::uint8_t>(std::random_device{}() % 255 + 1);
    messageIds_.assign(numExtensions, firstId);
    extensions_.reserve(numExtensions);
    for (std::size_t i = 0; i < numExtensions; ++i)
    {
        extensions_.emplace_back(static_cast<std::uint32_t>(i));
    }

    sendConfig();

    RequestInfoPacket infoPacket(extensions_);
    RequestInfoResponse infoResponse(extensions_);
    sendPacket(infoPacket, infoResponse);
}

void FtConnection::setPwmOutput(ExtensionIndex index, PWM pwm, std::int16_t duty)
{
    extension(index).output.pwmOutputValues.at(static_cast<std::size_t>(pwm)) = duty;
}

void FtConnection::setMotorOutput(ExtensionIndex index, Motor motor, std::int16_t duty)
{
    auto& pwm = extension(index).output.pwmOutputValues;
    auto m = static_cast<std::size_t>(motor);
    pwm.at(2 * m) = duty > 0 ? duty : std::int16_t{0};
    pwm.at(2 * m + 1) = duty > 0 ? std::int16_t{0} : static_cast<std::int16_t>(-duty);
}

void FtConnection::setMotorDistance(ExtensionIndex index, Motor motor, std::int16_t duty, std::uint16_t distance)
{
    setMotorOutput(index, motor, duty);
    Output& output = extension(index).output;
    auto m = static_cast<std::size_t>(motor);
    output.motorDistanceValues.at(m) = distance;
    ++output.motorCommandId.at(m);
}

void FtConnection::setMotorSync(ExtensionIndex index, Motor motor, Motor other, std::int16_t duty,
                                std::uint16_t /*distance*/)
{
    setMotorOutput(index, motor, duty);
    setMotorOutput(index, other, duty);

    extension(index).output.motorMasterValues.at(static_cast<std::size_t>(motor)) =
        static_cast<std::uint8_t>(static_cast<int>(other) + 1);
}

void FtConnection::showMessage(ExtensionIndex index, std::string_view text)
{
    auto controller = static_cast<std::size_t>(index);
    std::uint8_t& id = messageIds_.at(controller);
    id = static_cast<std::uint8_t>(id == 255 ? 1 : id + 1); // the firmware only shows a message with a new id

    MessagePacket packet(extensions_.at(controller).transferAreaIndex, id, text);
    MessageResponse response;
    sendPacket(packet, response);
    if (!response.success())
    {
        throw ProtocolError("Message not accepted");
    }
}

std::array<bool, 8> FtConnection::extensionsOnline()
{
    StatePacket packet;
    StateResponse response;
    sendPacket(packet, response);
    if (!response.success())
    {
        throw ProtocolError("State request not accepted");
    }
    return response.extensionsOnline();
}

void FtConnection::resetCounter(ExtensionIndex index, Counter counter)
{
    ++extension(index).output.counterResetCommandId.at(static_cast<std::size_t>(counter));
}

void FtConnection::stopAllOutputs()
{
    for (Extension& extension : extensions_)
    {
        extension.output.stopAll();
    }
}

bool FtConnection::digitalValue(ExtensionIndex index, UniversalInput input)
{
    return analogValue(index, input) != 0;
}

std::uint16_t FtConnection::analogValue(ExtensionIndex index, UniversalInput input)
{
    return extension(index).input.universalInput(static_cast<std::size_t>(input));
}

void FtConnection::update()
{
    if (std::ranges::any_of(extensions_, [](const Extension& extension) { return extension.config.changed; }))
    {
        sendConfig();
    }

    OutputPacket outputPacket(extensions_);
    InputResponse inputResponse(extensions_);
    sendPacket(outputPacket, inputResponse);
}

void FtConnection::sendConfig()
{
    ConfigPacket configPacket(extensions_);
    ConfigResponse configResponse(extensions_.size());
    sendPacket(configPacket, configResponse);
    if (!configResponse.success())
    {
        throw ProtocolError("Config failed!");
    }

    for (Extension& extension : extensions_)
    {
        extension.config.changed = false;
    }
}

void FtConnection::sendPacket(Packet& packet, Response& response)
{
    packet.setSessionId(sessionId_);
    packet.setTransactionId(transactionId_);
    std::vector<std::uint8_t> data = packet.bytes();
    if (trace_)
    {
        std::cerr << "> " << toHex(data) << '\n';
    }
    serialPort_.write(data);

    std::vector<std::uint8_t> receivedData(response.length());
    std::size_t received = serialPort_.read(receivedData);
    if (received < receivedData.size())
    {
        receivedData.resize(received);
        throw TimeoutError("No response from controller (got " + std::to_string(received) + " of " +
                           std::to_string(response.length()) + " bytes: " + toHex(receivedData) + ")");
    }
    if (trace_)
    {
        std::cerr << "< " << toHex(receivedData) << '\n';
    }

    response.load(receivedData);
    sessionId_ = response.sessionId();
    transactionId_ = static_cast<std::uint16_t>(response.transactionId() + 1);
}

} // namespace ftrobotx
