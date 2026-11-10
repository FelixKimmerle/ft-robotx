namespace FtRoboTx.Packets;

/// <summary>
/// Bluetooth message commands (telecodes 10 ... 18, found in ftMscLib). The controller answers with telecode + 100,
/// some right away, some seconds later (connect), and also sends such packets by itself (received messages, incoming
/// connections, scan results); FtConnection treats all of them as events.
/// </summary>
public class BtPacket : Packet
{
    public const int Scan = 10;
    public const int CancelScan = 11;
    public const int Connect = 12;
    public const int Disconnect = 13;
    public const int Send = 14;
    public const int StartReceive = 15;
    public const int StopReceive = 16;
    public const int Listen = 17;
    public const int StopListen = 18;

    private readonly byte[] payload = [];

    /// <summary>Whether a reply telecode belongs to a Bluetooth command.</summary>
    public static bool IsReply(int commandCode) => commandCode is >= Scan + 100 and <= StopListen + 100;

    /// <summary>Scan or cancel scan: no payload.</summary>
    public BtPacket(int commandCode)
    {
        Header.CommandCode = commandCode;
        Header.NumPayload = 1;
    }

    /// <summary>Disconnect, start/stop receive, stop listen: channel, 0, 0, 0.</summary>
    public BtPacket(int commandCode, int channel) : this(commandCode)
    {
        payload = [(byte)channel, 0, 0, 0];
    }

    /// <summary>Connect, listen: channel, address, 0.</summary>
    public BtPacket(int commandCode, int channel, BtAddress address) : this(commandCode)
    {
        payload = [(byte)channel, .. address.ToBytes(), 0];
    }

    /// <summary>Send: channel, 0, length (16 bit), message, 2 bytes padding (as ftMscLib does).</summary>
    public BtPacket(int channel, ReadOnlySpan<byte> message) : this(Send)
    {
        payload = [(byte)channel, 0, (byte)message.Length, (byte)(message.Length >> 8), .. message, 0, 0];
    }

    protected override void WritePayload(BinaryWriter writer) => writer.Write(payload);
}

/// <summary>Any packet with a Bluetooth reply telecode (110 ... 118).</summary>
public class BtReply : Response
{
    /// <summary>The message length the controller reports (it keeps 16 bytes).</summary>
    public const int MaxMessage = 16;

    /// <summary>Scan replies (110, 111) carry a scan status, the others a channel event.</summary>
    public bool IsScan => Header.CommandCode is BtPacket.Scan + 100 or BtPacket.CancelScan + 100;

    public BtEvent? Event { get; private set; }
    public BtScanResult? ScanResult { get; private set; }

    protected override void LoadPayload(ReadOnlySpan<byte> payload)
    {
        int offset = 0;
        if (IsScan)
        {
            // BT_SCAN_STATUS: status, address, 2 bytes padding, name
            var status = (BtScanStatus)ReadUInt16(payload, ref offset);
            if (payload.Length < 10)
            {
                ScanResult = new BtScanResult(status, default, "");
                return;
            }
            ReadOnlySpan<byte> name = payload[10..];
            int end = name.IndexOf((byte)0);
            ScanResult = new BtScanResult(status, BtAddress.FromBytes(payload[2..]),
                System.Text.Encoding.ASCII.GetString(end < 0 ? name : name[..end]));
            return;
        }

        // BT_CB: channel, status; for received messages BT_RECV_CB: channel, status, length, message (16 bytes)
        var command = (BtCommand)(Header.CommandCode - 100 - BtPacket.Connect);
        int channel = ReadUInt16(payload, ref offset);
        var eventStatus = (BtStatus)ReadUInt16(payload, ref offset);
        byte[] message = [];
        if (eventStatus == BtStatus.MessageIndication)
        {
            int length = Math.Min((int)ReadUInt16(payload, ref offset), MaxMessage);
            message = payload.Slice(offset, length).ToArray();
        }
        Event = new BtEvent(command, channel, eventStatus, message);
    }
}
