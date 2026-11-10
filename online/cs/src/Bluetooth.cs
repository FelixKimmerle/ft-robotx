// Bluetooth messages between ROBO TX Controllers (ftMscLib's Bluetooth Message API), driven in online mode.
// The protocol is described in hybrid/relay/README.md.
using System.Globalization;

namespace FtRoboTx;

/// <summary>Result of a Bluetooth command or an event (CB_BtStatus in ROBO_TX_FW.h).</summary>
public enum BtStatus : ushort
{
    Success = 0,
    ConnectionExists = 1,
    ConnectionSetup = 2,
    SwitchedOff = 3,         // Bluetooth is switched off in the controller's menu
    AllChannelsBusy = 4,
    NotRoboTx = 5,           // the remote device is no ROBO TX Controller
    ConnectionTimeout = 6,   // no device with this address answered
    ConnectionInvalid = 7,   // no connection on this channel
    ConnectionRelease = 8,
    ListenActive = 9,
    ReceiveActive = 10,
    ConnectionIndication = 11, // incoming connection (on a listening channel)
    DisconnectIndication = 12, // the remote side closed the connection
    MessageIndication = 13,    // a message arrived
    ChannelBusy = 14,
    AddressBusy = 15,
    NoListenActive = 16,       // the remote controller doesn't listen for this controller
}

/// <summary>The command an event belongs to, in the order of the telecodes (12 ... 18).</summary>
public enum BtCommand
{
    Connect,
    Disconnect,
    Send,
    Receive, // StartBtReceive and every message received afterwards
    StopReceive,
    Listen,  // BtListen and incoming connections/disconnections afterwards
    StopListen,
}

/// <summary>A report from the controller about a Bluetooth channel.</summary>
public sealed record BtEvent(BtCommand Command, int Channel, BtStatus Status, byte[] Message);

/// <summary>Status of a scan for Bluetooth devices (BtInquiryScanStatus in ROBO_TX_FW.h).</summary>
public enum BtScanStatus : ushort
{
    NotPossible = 0,
    Started = 1,
    Result = 2, // a device was found; the controller's own address always comes first
    Busy = 3,
    Timeout = 4,
    Ended = 5,
}

public sealed record BtScanResult(BtScanStatus Status, BtAddress Address, string Name);

/// <summary>Bluetooth address, most significant byte first, as printed: 00:13:7B:53:DB:5E.</summary>
public readonly record struct BtAddress(byte A0, byte A1, byte A2, byte A3, byte A4, byte A5)
{
    public byte[] ToBytes() => [A0, A1, A2, A3, A4, A5];

    public static BtAddress FromBytes(ReadOnlySpan<byte> b) => new(b[0], b[1], b[2], b[3], b[4], b[5]);

    /// <summary>Parses "00:13:7B:53:DB:5E" (also with '-' or without separators); null if invalid.</summary>
    public static BtAddress? Parse(string? text)
    {
        string hex = (text ?? "").Replace(":", "").Replace("-", "");
        if (hex.Length != 12 || !ulong.TryParse(hex, NumberStyles.HexNumber, CultureInfo.InvariantCulture, out _))
        {
            return null;
        }
        return FromBytes(Convert.FromHexString(hex));
    }

    public override string ToString() => string.Join(":", ToBytes().Select(b => b.ToString("X2")));
}
