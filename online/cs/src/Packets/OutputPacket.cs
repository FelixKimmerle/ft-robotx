using FtRoboTx.Extensions;

namespace FtRoboTx.Packets;

public class OutputPacket : Packet
{
    private readonly IReadOnlyList<Extension> extensions;

    public OutputPacket(IReadOnlyList<Extension> extensions)
    {
        this.extensions = extensions;
        Header.CommandCode = 0x02;
        Header.NumPayload = extensions.Count;
    }

    protected override void WritePayload(BinaryWriter writer)
    {
        foreach (Extension extension in extensions)
        {
            Output output = extension.Output;

            writer.Write(extension.TransferAreaIndex);

            foreach (ushort value in output.CounterResetCommandId)
            {
                writer.Write(value);
            }

            writer.Write(output.MotorMasterValues);

            foreach (short value in output.PwmOutputValues)
            {
                writer.Write(value);
            }

            foreach (ushort value in output.MotorDistanceValues)
            {
                writer.Write(value);
            }

            foreach (ushort value in output.MotorCommandId)
            {
                writer.Write(value);
            }
        }
    }
}
