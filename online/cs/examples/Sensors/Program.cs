// Ultrasonic distance sensor on I2 and NTC (temperature) on I3.
// Usage: dotnet run -- [-p PORT]; --help explains the options.
using System.Diagnostics;
using FtRoboTx;
using FtRoboTx.Extensions;

const ExtensionIndex Tx = ExtensionIndex.Local;

string? portName = CommandLine.ParsePort(args, "Sensors",
    "Shows the distance from the ultrasonic sensor on I2 and the temperature from the NTC on I3 twice a second.\n" +
    "Ctrl+C quits.", out int exitCode);
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
ft.SetInputMode(Tx, UniversalInput.I2, InputMode.Ultrasonic);
ft.SetInputMode(Tx, UniversalInput.I3, InputMode.Resistance); // analog, in Ohm
Console.WriteLine("Ultrasonic sensor on I2, NTC on I3. Press Ctrl+C to quit.");

Stopwatch report = Stopwatch.StartNew();
bool first = true;
while (!cts.IsCancellationRequested)
{
    ft.Update();
    if (!first && report.ElapsedMilliseconds < 500)
    {
        continue;
    }
    first = false;
    report.Restart();

    ushort? distance = ft.GetUltrasonicDistance(Tx, UniversalInput.I2);
    double? temperature = ft.GetNtcTemperature(Tx, UniversalInput.I3);
    // For your own sensor: ft.GetNtcTemperature(Tx, UniversalInput.I3, r25: 1500.0, b: 3900.0)
    string distanceText = distance is { } cm ? $"{cm,4} cm" : "   - cm";
    string temperatureText = temperature is { } celsius
        ? $"{celsius,5:F1} °C ({ft.GetAnalogValue(Tx, UniversalInput.I3)} Ohm)"
        : "    - °C";
    Console.WriteLine($"Distance {distanceText}   Temperature {temperatureText}");
}

return 0;
