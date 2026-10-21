using System.Buffers.Binary;

namespace FtRoboTxOnline.Packets;

public class Header
{
    /// <summary>Size of the header on the wire, including the 0x02 0x55 start bytes.</summary>
    public const int Size = 24;

    public int From { get; set; } = 0x02;
    public int To { get; set; } = 0x01;
    public ushort TransactionId { get; set; }
    public ushort SessionId { get; set; }
    public int CommandCode { get; set; }
    public int NumPayload { get; set; } = 1;
    public short Length { get; set; } = 27;

    public void Load(ReadOnlySpan<byte> bytes)
    {
        Length = BinaryPrimitives.ReadInt16BigEndian(bytes[2..]);
        From = BinaryPrimitives.ReadInt32LittleEndian(bytes[4..]);
        To = BinaryPrimitives.ReadInt32LittleEndian(bytes[8..]);
        TransactionId = BinaryPrimitives.ReadUInt16LittleEndian(bytes[12..]);
        SessionId = BinaryPrimitives.ReadUInt16LittleEndian(bytes[14..]);
        CommandCode = BinaryPrimitives.ReadInt32LittleEndian(bytes[16..]);
        if (bytes.Length > Response.MinLength)
        {
            NumPayload = BinaryPrimitives.ReadInt32LittleEndian(bytes[20..]);
        }
    }

    /// <summary>Writes the header without the start bytes.</summary>
    public void Write(BinaryWriter writer)
    {
        // BinaryWriter is always little-endian; the length field is big-endian.
        writer.Write(BinaryPrimitives.ReverseEndianness(Length));
        writer.Write(From);
        writer.Write(To);
        writer.Write(TransactionId);
        writer.Write(SessionId);
        writer.Write(CommandCode);
        writer.Write(NumPayload);
    }
}
