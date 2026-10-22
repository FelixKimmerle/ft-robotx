namespace FtRoboTx;

/// <summary>Command-line handling for the example programs: -h/--help and -p/--port, in the usual --help style.</summary>
public static class CommandLine
{
    public const string PortHelp =
        "Port of the ROBO TX Controller: /dev/ttyACM0 (USB), /dev/rfcomm0 (Bluetooth via rfcomm)\n" +
        "                        or COM3 (Windows). Default: the first ROBO TX Controller found on USB (Linux).";

    /// <summary>
    /// Parses [-p PORT] and -h/--help. Returns the port to use, or null if the program should exit with exitCode
    /// (after the help, an error, or when no controller was found).
    /// </summary>
    public static string? ParsePort(string[] args, string program, string description, out int exitCode)
    {
        exitCode = 0;
        string? port = null;
        for (int i = 0; i < args.Length; i++)
        {
            switch (args[i])
            {
                case "-h" or "--help":
                    Console.WriteLine($"Usage: {program} [-p PORT]\n\n{description}\n\nOptions:");
                    Console.WriteLine("  -h, --help            Show this help and exit.");
                    Console.WriteLine($"  -p, --port PORT       {PortHelp}");
                    return null;
                case "-p" or "--port" when i + 1 < args.Length:
                    port = args[++i];
                    break;
                default:
                    Console.Error.WriteLine($"{program}: unexpected argument {args[i]}; see --help");
                    exitCode = 2;
                    return null;
            }
        }

        port ??= FindUsbController();
        if (port is null)
        {
            Console.Error.WriteLine("No ROBO TX Controller found on USB; choose the port with -p (see --help)");
            exitCode = 1;
        }
        return port;
    }

    /// <summary>The first ROBO TX Controller on USB (USB ID 221d:1000), found through /sys on Linux.</summary>
    public static string? FindUsbController()
    {
        if (!OperatingSystem.IsLinux())
        {
            return null;
        }
        foreach (string tty in Directory.GetDirectories("/sys/class/tty", "ttyACM*").Order())
        {
            // From the tty's USB interface up to the USB device, which has the IDs.
            DirectoryInfo? device = new DirectoryInfo(Path.Combine(tty, "device")).ResolveLinkTarget(true) as DirectoryInfo;
            for (int up = 0; device is not null && up < 3; up++, device = device.Parent)
            {
                string vendor = Path.Combine(device.FullName, "idVendor");
                if (File.Exists(vendor))
                {
                    string id = File.ReadAllText(vendor).Trim() + ":" +
                                File.ReadAllText(Path.Combine(device.FullName, "idProduct")).Trim();
                    if (id == "221d:1000")
                    {
                        string port = "/dev/" + Path.GetFileName(tty);
                        Console.WriteLine($"Using {port} (ROBO TX Controller on USB)");
                        return port;
                    }
                    break;
                }
            }
        }
        return null;
    }
}
