using FtRoboTx.Extensions;

namespace FtRoboTx.Packets;

public class RequestInfoPacket : Packet
{
    private readonly IReadOnlyList<Extension> extensions;

    public RequestInfoPacket(IReadOnlyList<Extension> extensions)
    {
        this.extensions = extensions;
        Header.CommandCode = 0x06;
        Header.NumPayload = extensions.Count;
    }

    protected override void WritePayload(BinaryWriter writer)
    {
        foreach (Extension extension in extensions)
        {
            writer.Write(extension.TransferAreaIndex);
        }
    }
}
