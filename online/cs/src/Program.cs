using System.IO.Ports;
using FtRoboTxOnline;
using FtRoboTxOnline.Extensions;

// Usage: dotnet run [port], e.g. /dev/ttyACM0 (USB), /dev/rfcomm0 (Bluetooth) or COM3
string[] ports = SerialPort.GetPortNames();
foreach (string port in ports)
{
    Console.WriteLine(port);
}

string? portName = args.Length > 0 ? args[0] : ports.FirstOrDefault();
if (portName is null)
{
    Console.Error.WriteLine("No serial port found.");
    return 1;
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
