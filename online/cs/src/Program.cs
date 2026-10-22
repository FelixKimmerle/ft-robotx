using System.IO.Ports;
using FtRoboTx;
using FtRoboTx.Extensions;

// Usage: dotnet run -- [-p PORT]; --help explains the options.
string? portName = CommandLine.ParsePort(args, "FtRoboTx",
    "The original demo: master and extension 1; a motor on M3 of the extension (M1 synchronized to it), and the\n" +
    "button on I8 of the master starts a distance run of M1. Lists the serial ports found. Ctrl+C quits.",
    out int exitCode);
if (portName is null)
{
    return exitCode;
}
foreach (string port in SerialPort.GetPortNames())
{
    Console.WriteLine(port);
}

using CancellationTokenSource cts = new();
Console.CancelKeyPress += (_, e) =>
{
    e.Cancel = true;
    cts.Cancel();
};

using FtConnection ft = new(portName, numExtensions: 2);
Console.WriteLine("Config success");

foreach (Extension extension in ft.Extensions)
{
    Console.WriteLine(extension.Name);
    Console.WriteLine(extension.Version);
    Console.WriteLine(extension.MacAddress);
}

//ft.SetMotorSync(ExtensionIndex.Slave1, Motor.M1, Motor.M3, 512);
//ft.SetMotorDistance(ExtensionIndex.Slave1, Motor.M1, -512, 200);

ft.Extensions[(int)ExtensionIndex.Slave1].Output.MotorMasterValues[(int)Motor.M1] = (byte)(Motor.M3 + 1);
ft.SetMotorOutput(ExtensionIndex.Slave1, Motor.M3, 512);

try
{
    while (!cts.IsCancellationRequested)
    {
        ft.Update();

        foreach (Extension extension in ft.Extensions)
        {
            if (extension.Input.Changed)
            {
                Console.WriteLine(extension.Input.GetMotorReachedDestination((int)Motor.M1));
                extension.Input.Changed = false;
            }
        }

        if (ft.GetDigitalValue(ExtensionIndex.Local, UniversalInput.I8))
        {
            ft.SetMotorDistance(ExtensionIndex.Slave1, Motor.M1, -512, 200);
        }
    }
}
finally
{
    // Don't leave the motors running after Ctrl+C.
    ft.StopAllOutputs();
    ft.Update();
}

return 0;
