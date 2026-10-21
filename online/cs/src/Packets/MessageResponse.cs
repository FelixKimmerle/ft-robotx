namespace FtRoboTxOnline.Packets;

public class MessageResponse : Response
{
    public override int Length => MinLength + 4;

    public bool Success => Header.CommandCode == 108;
}
