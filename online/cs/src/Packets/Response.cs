using System.Buffers.Binary;

namespace FtRoboTx.Packets;

public class Response
{
    /// <summary>Length of a response without payload: header, CRC and end byte.</summary>
    public const int MinLength = 27;

    protected Header Header { get; } = new();

    public ushort SessionId => Header.SessionId;
    public ushort TransactionId => Header.TransactionId;

    /// <summary>Total number of bytes this response occupies on the wire.</summary>
    public virtual int Length => MinLength;

    public void Load(ReadOnlySpan<byte> bytes)
    {
        Header.Load(bytes);
        if (bytes.Length > MinLength)
        {
            LoadPayload(bytes.Slice(Header.Size, Header.Length - 20));
        }
        else
        {
            LoadPayload([]);
        }
    }

    protected virtual void LoadPayload(ReadOnlySpan<byte> payload) { }

    protected static int ReadInt32(ReadOnlySpan<byte> bytes, ref int offset)
    {
        int value = BinaryPrimitives.ReadInt32LittleEndian(bytes[offset..]);
        offset += sizeof(int);
        return value;
    }

    protected static ushort ReadUInt16(ReadOnlySpan<byte> bytes, ref int offset)
    {
        ushort value = BinaryPrimitives.ReadUInt16LittleEndian(bytes[offset..]);
        offset += sizeof(ushort);
        return value;
    }
}
