// Bluetooth messages between ROBO TX Controllers (ftMscLib's Bluetooth Message API), driven in online mode.
// The protocol is described in hybrid/relay/README.md.
#pragma once

#include <array>
#include <cstdint>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

namespace ftrobotx
{

/// Bluetooth channels (ftMscLib: chanIdx) are numbered 1 ... 8.
inline constexpr int kBtChannels = 8;

/// The receiving controller keeps only the first 16 bytes of a message (measured), so longer ones are refused.
inline constexpr std::size_t kBtMaxMessage = 16;

/// Bluetooth address, most significant byte first, as printed: 00:13:7B:53:DB:5E is {0x00, 0x13, 0x7B, ...}.
using BtAddress = std::array<std::uint8_t, 6>;

[[nodiscard]] std::string toString(const BtAddress& address);

/// Parses "00:13:7B:53:DB:5E" (also with '-' or without separators).
[[nodiscard]] std::optional<BtAddress> parseBtAddress(std::string_view text);

/// Result of a Bluetooth command or an event (CB_BtStatus in ROBO_TX_FW.h).
enum class BtStatus : std::uint16_t
{
    Success = 0,
    ConnectionExists = 1,
    ConnectionSetup = 2,
    SwitchedOff = 3, // Bluetooth is switched off in the controller's menu
    AllChannelsBusy = 4,
    NotRoboTx = 5,         // the remote device is no ROBO TX Controller
    ConnectionTimeout = 6, // no device with this address answered
    ConnectionInvalid = 7, // no connection on this channel
    ConnectionRelease = 8,
    ListenActive = 9,
    ReceiveActive = 10,
    ConnectionIndication = 11, // incoming connection (on a listening channel)
    DisconnectIndication = 12, // the remote side closed the connection
    MessageIndication = 13,    // a message arrived
    ChannelBusy = 14,
    AddressBusy = 15,
    NoListenActive = 16, // the remote controller doesn't listen for this controller
};

[[nodiscard]] const char* toString(BtStatus status);

/// The command an event belongs to, in the order of the telecodes (12 ... 18).
enum class BtCommand
{
    Connect,
    Disconnect,
    Send,
    Receive, // btStartReceive() and every message received afterwards
    StopReceive,
    Listen, // btListen() and incoming connections/disconnections afterwards
    StopListen,
};

[[nodiscard]] const char* toString(BtCommand command);

/// A report from the controller about a Bluetooth channel.
struct BtEvent
{
    BtCommand command = BtCommand::Connect;
    int channel = 0; // 1 ... 8
    BtStatus status = BtStatus::Success;
    std::vector<std::uint8_t> message; // for BtStatus::MessageIndication
};

/// Status of a scan for Bluetooth devices (BtInquiryScanStatus in ROBO_TX_FW.h).
enum class BtScanStatus : std::uint16_t
{
    NotPossible = 0,
    Started = 1,
    Result = 2, // a device was found; the controller's own address always comes first
    Busy = 3,
    Timeout = 4,
    Ended = 5,
};

struct BtScanResult
{
    BtScanStatus status = BtScanStatus::NotPossible;
    BtAddress address{}; // for BtScanStatus::Result
    std::string name;
};

} // namespace ftrobotx
