using System.Text;
using FtRoboTxOnline.Extensions;

namespace FtRoboTxOnline.Packets;

public class RequestInfoResponse(IReadOnlyList<Extension> extensions) : Response
{
    public override int Length => MinLength + 68 * extensions.Count;

    protected override void LoadPayload(ReadOnlySpan<byte> payload)
    {
        int offset = 0;
        for (int i = 0; i < extensions.Count; i++)
        {
            Extension extension = extensions[ReadInt32(payload, ref offset)];

            extension.Name = ReadString(payload.Slice(offset, 17));
            offset += 17;
            extension.MacAddress = ReadString(payload.Slice(offset, 18));
            offset += 18;
            offset += 1;
            // ta_array_start_addr, pgm_area_start_addr, pgm_area_size
            offset += 3 * 4;

            // FT_VERSION: hardware, firmware, transfer area, reserved; each FT_VER is the bytes a, b, c, d
            ReadOnlySpan<byte> version = payload.Slice(offset, 16);
            extension.Version = new FirmwareVersion(
                HardwareRevision: (char)version[0],
                FirmwareMajor: version[6],
                FirmwareMinor: version[7],
                DllVersion: version[5],
                TransferAreaMajor: version[10],
                TransferAreaMinor: version[11]);
            offset += 16;
        }
    }

    private static string ReadString(ReadOnlySpan<byte> bytes) => Encoding.ASCII.GetString(bytes.TrimEnd((byte)0));
}
