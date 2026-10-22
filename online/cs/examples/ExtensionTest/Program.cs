// Master and one extension, each with a button on I1 and a lamp on O1, wired crosswise:
// the master's button switches the extension's lamp and the other way round.
// Usage: dotnet run -- [-p PORT]; --help explains the options.
using FtRoboTx;
using FtRoboTx.Extensions;

const short On = 512;
ExtensionIndex[] controllers = [ExtensionIndex.Local, ExtensionIndex.Slave1];

string? portName = CommandLine.ParsePort(args, "ExtensionTest",
    "Master and extension 1, each with a button on I1 and a lamp on O1, crosswise: each button switches the\n" +
    "other controller's lamp. Ctrl+C quits and switches the lamps off.", out int exitCode);
if (portName is null)
{
    return exitCode;
}

using CancellationTokenSource cts = new();
Console.CancelKeyPress += (_, e) =>
{
    e.Cancel = true;
    cts.Cancel();
};

using FtConnection ft = new(portName, numExtensions: controllers.Length);
foreach (ExtensionIndex index in controllers)
{
    Extension extension = ft.Extensions[(int)index];
    Console.WriteLine($"{index,-6}: {extension.Name} {extension.Version} {extension.MacAddress}");
}
Console.WriteLine("Master button -> extension lamp, extension button -> master lamp. Press Ctrl+C to quit.");

bool?[] wasPressed = new bool?[controllers.Length];
try
{
    while (!cts.IsCancellationRequested)
    {
        ft.Update();

        for (int i = 0; i < controllers.Length; i++)
        {
            ExtensionIndex button = controllers[i];
            ExtensionIndex lamp = controllers[(i + 1) % controllers.Length];

            bool pressed = ft.GetDigitalValue(button, UniversalInput.I1);
            if (pressed != wasPressed[i])
            {
                Console.WriteLine($"{button} button {(pressed ? "pressed" : "released")} -> {lamp} lamp {(pressed ? "on" : "off")}");
                ft.SetPWMOutput(lamp, PWM.O1, pressed ? On : (short)0);
                wasPressed[i] = pressed;
            }
        }
    }
}
finally
{
    ft.StopAllOutputs();
    ft.Update();
}

return 0;
