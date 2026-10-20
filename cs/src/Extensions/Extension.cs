namespace FtRoboTxOnline.Extensions;

/// <summary>A TX controller (local master or one of the extension slaves) and its transfer area.</summary>
public class Extension(uint transferAreaIndex)
{
    public const int Motors = 4;
    public const int UniversalInputs = 8;

    public uint TransferAreaIndex { get; } = transferAreaIndex;

    public string? Name { get; set; }
    public FirmwareVersion? Version { get; set; }
    public string? MacAddress { get; set; }

    public Config Config { get; } = new();
    public Output Output { get; } = new();
    public Input Input { get; } = new();
}
