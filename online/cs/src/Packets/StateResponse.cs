namespace FtRoboTx.Packets;

public class StateResponse : Response
{
    public override int Length => MinLength + 4 + 24;

    public bool Success => Header.CommandCode == 107;

    /// <summary>Extension 1 ... 8 connected to the master.</summary>
    public bool[] ExtensionsOnline { get; } = new bool[8];

    protected override void LoadPayload(ReadOnlySpan<byte> payload)
    {
        for (int i = 0; i < ExtensionsOnline.Length; i++)
        {
            ExtensionsOnline[i] = payload[4 + i] != 0;
        }
    }
}
