namespace FtRoboTx.Extensions;

/// <summary>Versions from the controller's info block (FT_VERSION in ROBO_TX_FW.h).</summary>
public readonly record struct FirmwareVersion(
    char HardwareRevision,
    byte FirmwareMajor,
    byte FirmwareMinor,
    byte DllVersion,
    byte TransferAreaMajor,
    byte TransferAreaMinor)
{
    /// <summary>Firmware version as shown by fischertechnik, e.g. "1.30".</summary>
    public override string ToString() => $"{FirmwareMajor}.{FirmwareMinor:00}";
}
