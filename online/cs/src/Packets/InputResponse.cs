using FtRoboTx.Extensions;

namespace FtRoboTx.Packets;

public class InputResponse(IReadOnlyList<Extension> extensions) : Response
{
    public override int Length => MinLength + 52 * extensions.Count;

    protected override void LoadPayload(ReadOnlySpan<byte> payload)
    {
        int offset = 0;

        for (int e = 0; e < Header.NumPayload; e++)
        {
            Input input = extensions[ReadInt32(payload, ref offset)].Input;

            for (int i = 0; i < Extension.UniversalInputs; i++)
            {
                input.SetUniversalInput(i, ReadUInt16(payload, ref offset));
            }

            for (int i = 0; i < Extension.Motors; i++)
            {
                input.SetCounterInput(i, payload[offset++]);
            }

            for (int i = 0; i < Extension.Motors; i++)
            {
                input.SetCounterValue(i, ReadUInt16(payload, ref offset));
            }

            input.SetDisplayButtonLeft(ReadUInt16(payload, ref offset));
            input.SetDisplayButtonRight(ReadUInt16(payload, ref offset));

            for (int i = 0; i < Extension.Motors; i++)
            {
                input.SetResetCounter(i, ReadUInt16(payload, ref offset));
            }

            for (int i = 0; i < Extension.Motors; i++)
            {
                input.SetMotorReachedDestination(i, ReadUInt16(payload, ref offset));
            }
        }
    }
}
