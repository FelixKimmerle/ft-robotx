namespace FtRoboTx.Packets;

public class ConfigResponse(int numExtensions) : Response
{
    public override int Length => MinLength + 4 * numExtensions;

    public bool Success => Header.CommandCode == 0x69;
}
