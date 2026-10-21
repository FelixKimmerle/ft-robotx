using System.Text;

namespace FtRoboTxOnline.Packets;

/// <summary>
/// Shows a text on a controller's display, or clears it with an empty text (telecode 8, like ftMscLib's
/// SetRoboTxMessage). The firmware only shows a message whose id differs from the previous one.
/// </summary>
public class MessagePacket : Packet
{
    /// <summary>Longest text the firmware shows.</summary>
    public const int MaxLength = 98;

    private readonly uint transferAreaIndex;
    private readonly byte messageId;
    private readonly byte[] text;

    public MessagePacket(uint transferAreaIndex, byte messageId, string text)
    {
        this.transferAreaIndex = transferAreaIndex;
        this.messageId = messageId;
        this.text = Encoding.ASCII.GetBytes(text.Length > MaxLength ? text[..MaxLength] : text);
        Header.CommandCode = 0x08;
        Header.NumPayload = 1;
    }

    protected override void WritePayload(BinaryWriter writer)
    {
        writer.Write(transferAreaIndex);

        // DISPLAY_MSG: id, then the text, zero-padded to 99 bytes
        writer.Write(messageId);
        writer.Write(text);
        writer.Write(new byte[99 - text.Length]);
    }
}
