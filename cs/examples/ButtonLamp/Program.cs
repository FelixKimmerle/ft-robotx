// Lamp on O1 is on while the button on I1 is pressed.
// Usage: dotnet run [port], default /dev/ttyACM0 (USB)
using FtRoboTxOnline;
using FtRoboTxOnline.Extensions;

const short On = 512;

string portName = args.Length > 0 ? args[0] : "/dev/ttyACM0";

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
