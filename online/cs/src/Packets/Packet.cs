using System.Buffers.Binary;

namespace FtRoboTx.Packets;

public abstract class Packet
{
    protected bool SendPayload { get; init; } = true;
    protected Header Header { get; } = new();

    public ushort SessionId
    {
        set => Header.SessionId = value;
    }

    public ushort TransactionId
    {
        set => Header.TransactionId = value;
    }

    protected virtual void WritePayload(BinaryWriter writer) { }

    public byte[] GetBytes()
    {
        using MemoryStream payloadStream = new();
        using (BinaryWriter payloadWriter = new(payloadStream))
        {
            WritePayload(payloadWriter);
        }
        byte[] payload = payloadStream.ToArray();

        Header.Length = (short)(20 + payload.Length);

        using MemoryStream stream = new();
        using BinaryWriter writer = new(stream);
        writer.Write((byte)0x02);
        writer.Write((byte)0x55);
        Header.Write(writer);
        if (SendPayload)
        {
            writer.Write(payload);
        }
        writer.Flush();

        short crc = CalculateCrc(stream.GetBuffer().AsSpan(2, (int)stream.Length - 2));
        writer.Write(BinaryPrimitives.ReverseEndianness(crc));
        writer.Write((byte)0x03);
        writer.Flush();

        return stream.ToArray();
    }

    /// <summary>Two's complement of the 16-bit sum of all bytes.</summary>
    private static short CalculateCrc(ReadOnlySpan<byte> data)
    {
        short sum = 0;
        foreach (byte b in data)
        {
            sum += b;
        }
        return (short)-sum;
    }

    public override string ToString() => Convert.ToHexString(GetBytes());
}
