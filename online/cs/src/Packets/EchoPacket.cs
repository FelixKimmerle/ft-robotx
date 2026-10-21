namespace FtRoboTx.Packets;

public class EchoPacket : Packet
{
    public EchoPacket()
    {
        Header.CommandCode = 0x01;
        SendPayload = false;
    }
}
