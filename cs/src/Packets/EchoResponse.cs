namespace FtRoboTxOnline.Packets;

public class EchoResponse : Response
{
    public bool Success => Header.CommandCode == 0x65;
}
