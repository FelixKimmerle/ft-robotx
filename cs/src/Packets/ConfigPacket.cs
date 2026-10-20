using FtRoboTxOnline.Extensions;

namespace FtRoboTxOnline.Packets;

public class ConfigPacket : Packet
{
    private readonly IReadOnlyList<Extension> extensions;

    public ConfigPacket(IReadOnlyList<Extension> extensions)
    {
        this.extensions = extensions;
        Header.CommandCode = 0x05;
        Header.NumPayload = extensions.Count;
    }

    protected override void WritePayload(BinaryWriter writer)
    {
        foreach (Extension extension in extensions)
        {
            writer.Write(extension.TransferAreaIndex);

            for (int i = 0; i < Extension.Motors; i++)
            {
                writer.Write(extension.Config.GetMotorConfig(i));
            }

            for (int i = 0; i < Extension.UniversalInputs; i++)
            {
                writer.Write(extension.Config.GetUniversalInputConfig(i).ToByte());
            }

            for (int i = 0; i < Extension.Motors; i++)
            {
                writer.Write(extension.Config.GetCounterConfig(i));
            }

            // Reserved
            writer.Write(new byte[32]);
        }
    }
}
