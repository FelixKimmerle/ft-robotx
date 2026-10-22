// Lamp on O1 is on while the button on I1 is pressed.
// Usage: dotnet run -- [-p PORT]; --help explains the options.
using FtRoboTx;
using FtRoboTx.Extensions;

const short On = 512;

string? portName = CommandLine.ParsePort(args, "ButtonLamp",
    "Lamp on O1 lights while the button on I1 is pressed. Ctrl+C quits and switches the lamp off.", out int exitCode);
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

using FtConnection ft = new(portName);
Console.WriteLine($"Connected to {ft.Extensions[0].Name} {ft.Extensions[0].Version}. Press Ctrl+C to quit.");

bool? wasPressed = null;
try
{
    while (!cts.IsCancellationRequested)
    {
        ft.Update();

        bool pressed = ft.GetDigitalValue(ExtensionIndex.Local, UniversalInput.I1);
        if (pressed != wasPressed)
        {
            Console.WriteLine(pressed ? "Button pressed, lamp on" : "Button released, lamp off");
            ft.SetPWMOutput(ExtensionIndex.Local, PWM.O1, pressed ? On : (short)0);
            wasPressed = pressed;
        }
    }
}
finally
{
    ft.StopAllOutputs();
    ft.Update();
}

return 0;
