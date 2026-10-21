namespace FtRoboTxOnline.Packets;

/// <summary>
/// Reads the master's state (telecode 7, ftMscLib's STATE_RD_REQUEST). Of the 24 bytes per controller, only the
/// first 8 are known: whether extension 1 ... 8 is connected.
/// </summary>
public class StatePacket : Packet
{
    public StatePacket()
    {
        Header.CommandCode = 0x07;
        Header.NumPayload = 1;
    }

    protected override void WritePayload(BinaryWriter writer)
    {
        writer.Write(0u); // the master knows which extensions are connected
    }
}
