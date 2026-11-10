#include "ftrobotx/Bluetooth.hpp"

#include <cctype>

namespace ftrobotx
{

std::string toString(const BtAddress& address)
{
    constexpr char digits[] = "0123456789ABCDEF";
    std::string text;
    for (std::uint8_t b : address)
    {
        if (!text.empty())
        {
            text += ':';
        }
        text += digits[b >> 4];
        text += digits[b & 0xF];
    }
    return text;
}

std::optional<BtAddress> parseBtAddress(std::string_view text)
{
    BtAddress address{};
    std::size_t count = 0;
    int high = -1;
    for (char c : text)
    {
        if (c == ':' || c == '-')
        {
            continue;
        }
        if (!std::isxdigit(static_cast<unsigned char>(c)) || count == address.size())
        {
            return std::nullopt;
        }
        int digit = std::isdigit(static_cast<unsigned char>(c))
                        ? c - '0'
                        : std::toupper(static_cast<unsigned char>(c)) - 'A' + 10;
        if (high < 0)
        {
            high = digit;
        }
        else
        {
            address[count++] = static_cast<std::uint8_t>(high << 4 | digit);
            high = -1;
        }
    }
    if (count != address.size() || high >= 0)
    {
        return std::nullopt;
    }
    return address;
}

const char* toString(BtStatus status)
{
    switch (status)
    {
    case BtStatus::Success:
        return "success";
    case BtStatus::ConnectionExists:
        return "already connected";
    case BtStatus::ConnectionSetup:
        return "connection is being set up";
    case BtStatus::SwitchedOff:
        return "Bluetooth is switched off";
    case BtStatus::AllChannelsBusy:
        return "all channels busy";
    case BtStatus::NotRoboTx:
        return "not a ROBO TX Controller";
    case BtStatus::ConnectionTimeout:
        return "timeout, no device with this address";
    case BtStatus::ConnectionInvalid:
        return "no connection on this channel";
    case BtStatus::ConnectionRelease:
        return "disconnecting";
    case BtStatus::ListenActive:
        return "already listening";
    case BtStatus::ReceiveActive:
        return "already receiving";
    case BtStatus::ConnectionIndication:
        return "connected (incoming)";
    case BtStatus::DisconnectIndication:
        return "disconnected by the remote side";
    case BtStatus::MessageIndication:
        return "message received";
    case BtStatus::ChannelBusy:
        return "channel busy";
    case BtStatus::AddressBusy:
        return "address already used on another channel";
    case BtStatus::NoListenActive:
        return "the remote controller doesn't listen";
    }
    return "unknown status";
}

const char* toString(BtCommand command)
{
    switch (command)
    {
    case BtCommand::Connect:
        return "connect";
    case BtCommand::Disconnect:
        return "disconnect";
    case BtCommand::Send:
        return "send";
    case BtCommand::Receive:
        return "receive";
    case BtCommand::StopReceive:
        return "stop receive";
    case BtCommand::Listen:
        return "listen";
    case BtCommand::StopListen:
        return "stop listen";
    }
    return "unknown command";
}

} // namespace ftrobotx
