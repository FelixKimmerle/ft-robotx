namespace FtRoboTx.Extensions;

public readonly record struct UniversalInputConfig(InputMode Mode, bool Digital)
{
    public UniversalInputConfig() : this(InputMode.Resistance, true) { }

    public byte ToByte() => (byte)((Digital ? 0x80 : 0x00) | (byte)Mode);
}
