#include "ftrobotx/FtConnection.hpp"

#include <algorithm>
#include <cstdlib>
#include <iostream>
#include <random>
#include <stdexcept>
#include <string_view>
#include <utility>

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

/// Whether the firmware echoes the current counter reset id.
bool resetConfirmed(const Extension& extension, std::size_t counter)
{
    return extension.input.resetCounter(counter) == extension.output.counterResetCommandId.at(counter);
}

/// Whether the firmware echoes all current motor command and counter reset ids.
bool idsTakenOver(const Extension& extension)
{
    for (std::size_t i = 0; i < kMotors; ++i)
    {
        if (!extension.motorCommandDone(i) || !resetConfirmed(extension, i))
        {
            return false;
        }
    }
    return true;
}

int checkChannel(int channel)
{
    if (channel < 1 || channel > kBtChannels)
    {
        throw std::out_of_range("Bluetooth channel " + std::to_string(channel) + " (must be 1 ... 8)");
    }
    return channel;
}

/// Longest wait for the firmware to take over the command ids of a new connection.
constexpr std::chrono::milliseconds kSettleTimeout{500};

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

    // The firmware confirms motor commands and counter resets by echoing their ids. Until it has seen our ids
    // (starting at 0), it still echoes those of an earlier connection, which could pass for a confirmation of our
    // first command. It takes a few milliseconds; this also stops motors still running from an earlier connection.
    motorPending_.assign(numExtensions, {});
    counterResets_.assign(numExtensions, {});
    auto settleEnd = std::chrono::steady_clock::now() + kSettleTimeout;
    do
    {
        exchange();
    } while (!std::ranges::all_of(extensions_, idsTakenOver) && std::chrono::steady_clock::now() < settleEnd);
}

void FtConnection::setPwmOutput(ExtensionIndex index, PWM pwm, std::int16_t duty)
{
    extension(index).output.pwmOutputValues.at(static_cast<std::size_t>(pwm)) = duty;
}

template <typename Command> void FtConnection::motorCommand(ExtensionIndex index, Command command)
{
    Extension& ext = extension(index);
    std::array<std::uint16_t, kMotors> ids = ext.output.motorCommandId;
    command(ext);
    // A motor with a new command no longer has the old one to report; a started one is marked by the caller.
    auto& pending = motorPending_.at(static_cast<std::size_t>(index));
    for (std::size_t m = 0; m < kMotors; ++m)
    {
        pending[m] = pending[m] && ext.output.motorCommandId[m] == ids[m];
    }
}

void FtConnection::setMotorOutput(ExtensionIndex index, Motor motor, std::int16_t duty)
{
    motorCommand(index, [&](Extension& ext) { ext.setMotorOutput(static_cast<std::size_t>(motor), duty); });
}

void FtConnection::startMotorDistance(ExtensionIndex index, Motor motor, std::int16_t duty, std::uint16_t distance)
{
    auto m = static_cast<std::size_t>(motor);
    motorCommand(index, [&](Extension& ext) { ext.startMotorDistance(m, duty, distance); });
    motorPending_.at(static_cast<std::size_t>(index)).at(m) = distance != 0;
}

void FtConnection::startMotorSync(ExtensionIndex index, Motor master, Motor slave, std::int16_t duty,
                                  std::uint16_t distance, SyncDirection direction)
{
    auto m = static_cast<std::size_t>(master);
    auto s = static_cast<std::size_t>(slave);
    motorCommand(index, [&](Extension& ext) { ext.startMotorSync(m, s, duty, distance, direction); });
    auto& pending = motorPending_.at(static_cast<std::size_t>(index));
    pending.at(m) = pending.at(s) = distance != 0;
}

void FtConnection::stopMotor(ExtensionIndex index, Motor motor)
{
    motorCommand(index, [&](Extension& ext) { ext.stopMotor(static_cast<std::size_t>(motor)); });
}

bool FtConnection::motorCommandDone(ExtensionIndex index, Motor motor) const
{
    return extension(index).motorCommandDone(static_cast<std::size_t>(motor));
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
    auto c = static_cast<std::size_t>(counter);
    ++extension(index).output.counterResetCommandId.at(c);
    counterResets_.at(static_cast<std::size_t>(index)).at(c) = ResetState::Requested;
}

bool FtConnection::counterResetDone(ExtensionIndex index, Counter counter) const
{
    return counterResets_.at(static_cast<std::size_t>(index)).at(static_cast<std::size_t>(counter)) == ResetState::Done;
}

std::uint16_t FtConnection::counterValue(ExtensionIndex index, Counter counter) const
{
    return extension(index).input.counterValue(static_cast<std::size_t>(counter));
}

void FtConnection::stopAllOutputs()
{
    for (Extension& extension : extensions_)
    {
        extension.output.stopAll();
    }
    for (auto& pending : motorPending_)
    {
        pending.fill(false);
    }
}

void FtConnection::setInputMode(ExtensionIndex index, UniversalInput input, InputMode mode, bool digital)
{
    extension(index).config.setUniversalInputConfig(static_cast<std::size_t>(input), {mode, digital});
}

std::optional<double> FtConnection::ntcTemperature(ExtensionIndex index, UniversalInput input, double r25, double b)
{
    return ntcCelsius(analogValue(index, input), r25, b);
}

std::optional<std::uint16_t> FtConnection::ultrasonicDistance(ExtensionIndex index, UniversalInput input)
{
    return ultrasonicCm(analogValue(index, input));
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

    exchange();
    notify();
}

void FtConnection::exchange()
{
    OutputPacket outputPacket(extensions_);
    InputResponse inputResponse(extensions_);
    sendPacket(outputPacket, inputResponse);
}

void FtConnection::notify()
{
    // Taken out first: a callback may send further commands, whose replies are queued anew.
    std::vector<BtScanResult> scanResults = std::exchange(btScanResults_, {});
    for (const BtScanResult& result : scanResults)
    {
        if (btScanCallback_)
        {
            btScanCallback_(result);
        }
    }
    std::vector<BtEvent> events = std::exchange(btEvents_, {});
    for (const BtEvent& event : events)
    {
        if (btCallback_)
        {
            btCallback_(event);
        }
    }

    for (std::size_t e = 0; e < extensions_.size(); ++e)
    {
        auto index = static_cast<ExtensionIndex>(e);
        // Cleared before the call, so the callback can start the next command.
        for (std::size_t m = 0; m < kMotors; ++m)
        {
            if (motorPending_[e][m] && extensions_[e].motorCommandDone(m))
            {
                motorPending_[e][m] = false;
                if (motorReachedCallback_)
                {
                    motorReachedCallback_(index, static_cast<Motor>(m));
                }
            }
        }
        for (std::size_t c = 0; c < kMotors; ++c)
        {
            ResetState& state = counterResets_[e][c];
            if (state == ResetState::Confirmed)
            {
                state = ResetState::Done;
                if (counterResetCallback_)
                {
                    counterResetCallback_(index, static_cast<Counter>(c));
                }
            }
            else if (state == ResetState::Requested && resetConfirmed(extensions_[e], c))
            {
                state = ResetState::Confirmed;
            }
        }
    }
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

void FtConnection::sendRequest(Packet& packet)
{
    packet.setSessionId(sessionId_);
    packet.setTransactionId(0); // like ftMscLib for Bluetooth commands; the reply doesn't advance the ids
    std::vector<std::uint8_t> data = packet.bytes();
    if (trace_)
    {
        std::cerr << "> " << toHex(data) << '\n';
    }
    serialPort_.write(data);
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

    // Bluetooth replies and reports may come first; they are queued for the callbacks.
    std::vector<std::uint8_t> receivedData;
    for (;;)
    {
        receivedData = readPacket(response.length());
        std::int32_t code = receivedData[16] | receivedData[17] << 8 | receivedData[18] << 16 | receivedData[19] << 24;
        if (!BtPacket::isReply(code))
        {
            break;
        }
        queueBtReply(receivedData);
    }
    if (receivedData.size() != response.length())
    {
        throw ProtocolError("Unexpected response of " + std::to_string(receivedData.size()) + " bytes (expected " +
                            std::to_string(response.length()) + "): " + toHex(receivedData));
    }

    response.load(receivedData);
    sessionId_ = response.sessionId();
    transactionId_ = static_cast<std::uint16_t>(response.transactionId() + 1);
}

std::vector<std::uint8_t> FtConnection::readPacket(std::size_t expectedLength)
{
    auto timeout = [&](std::span<const std::uint8_t> got) {
        return TimeoutError("No response from controller (got " + std::to_string(got.size()) + " of " +
                            std::to_string(expectedLength) + " bytes: " + toHex(got) + ")");
    };

    // Start bytes 02 55 and the length (big endian) of what follows up to the checksum.
    std::vector<std::uint8_t> packet(4);
    std::size_t received = serialPort_.read(packet);
    if (received < packet.size())
    {
        throw timeout(std::span(packet).first(received));
    }
    while (packet[0] != 0x02 || packet[1] != 0x55)
    {
        if (trace_)
        {
            std::cerr << "skipping " << toHex(std::span(packet).first(1)) << '\n';
        }
        packet.erase(packet.begin());
        packet.resize(4);
        if (serialPort_.read(std::span(packet).last(1)) == 0)
        {
            throw timeout(std::span(packet).first(3));
        }
    }

    std::size_t length = static_cast<std::size_t>(packet[2] << 8 | packet[3]);
    if (length < 20)
    {
        throw ProtocolError("Invalid packet length " + std::to_string(length) + ": " + toHex(packet));
    }
    packet.resize(4 + length + 3); // + checksum and end byte
    received = serialPort_.read(std::span(packet).subspan(4));
    if (received < packet.size() - 4)
    {
        throw timeout(std::span(packet).first(4 + received));
    }
    if (trace_)
    {
        std::cerr << "< " << toHex(packet) << '\n';
    }
    return packet;
}

void FtConnection::queueBtReply(std::span<const std::uint8_t> bytes)
{
    BtReply reply;
    reply.load(bytes);
    if (reply.isScan())
    {
        btScanResults_.push_back(reply.scanResult());
    }
    else
    {
        btEvents_.push_back(reply.event());
    }
}

void FtConnection::btStartScan()
{
    BtPacket packet(BtPacket::kScan);
    sendRequest(packet);
}

void FtConnection::btCancelScan()
{
    BtPacket packet(BtPacket::kCancelScan);
    sendRequest(packet);
}

void FtConnection::btConnect(int channel, const BtAddress& address)
{
    BtPacket packet(BtPacket::kConnect, checkChannel(channel), address);
    sendRequest(packet);
}

void FtConnection::btDisconnect(int channel)
{
    BtPacket packet(BtPacket::kDisconnect, checkChannel(channel));
    sendRequest(packet);
}

void FtConnection::btListen(int channel, const BtAddress& remote)
{
    BtPacket packet(BtPacket::kListen, checkChannel(channel), remote);
    sendRequest(packet);
}

void FtConnection::btStopListen(int channel)
{
    BtPacket packet(BtPacket::kStopListen, checkChannel(channel));
    sendRequest(packet);
}

void FtConnection::btStartReceive(int channel)
{
    BtPacket packet(BtPacket::kStartReceive, checkChannel(channel));
    sendRequest(packet);
}

void FtConnection::btStopReceive(int channel)
{
    BtPacket packet(BtPacket::kStopReceive, checkChannel(channel));
    sendRequest(packet);
}

void FtConnection::btSend(int channel, std::span<const std::uint8_t> message)
{
    if (message.size() > kBtMaxMessage)
    {
        throw std::invalid_argument("Bluetooth message of " + std::to_string(message.size()) +
                                    " bytes; the receiving controller keeps only " + std::to_string(kBtMaxMessage));
    }
    BtPacket packet(checkChannel(channel), message);
    sendRequest(packet);
}

void FtConnection::btSend(int channel, std::string_view message)
{
    btSend(channel, std::span(reinterpret_cast<const std::uint8_t*>(message.data()), message.size()));
}

} // namespace ftrobotx
