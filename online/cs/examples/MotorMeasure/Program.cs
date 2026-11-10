// Measures the speed of the encoder motors on M1/C1 and M2/C2 at several duties, each on its own and synchronized.
// Usage: dotnet run -- [-p PORT]; --help explains the options.
using System.Diagnostics;
using FtRoboTx;
using FtRoboTx.Extensions;

const double PulsesPerTurn = 63.0 + 1.0 / 3.0; // pulses per turn of the encoder motor's output shaft
const int SpinUpMs = 500;
const int MeasureMs = 1000;
const ExtensionIndex Tx = ExtensionIndex.Local;

string? portName = CommandLine.ParsePort(args, "MotorMeasure",
    "Measures the encoder motors on M1 (counter C1) and M2 (counter C2): at duties 128, 256, 384 and 512, both\n" +
    "run on their own, then synchronized, for about 1.5 s each. Prints counter pulses per second, turns of the\n" +
    "output shaft per minute and, when synchronized, the largest difference between the counters. Ctrl+C stops.",
    out int exitCode);
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

// Calls Update() for the given time; returns the largest counter difference seen.
int Run(int milliseconds)
{
    int maxDifference = 0;
    Stopwatch watch = Stopwatch.StartNew();
    while (watch.ElapsedMilliseconds < milliseconds && !cts.IsCancellationRequested)
    {
        ft.Update();
        maxDifference = Math.Max(maxDifference,
            Math.Abs(ft.GetCounterValue(Tx, Counter.C1) - ft.GetCounterValue(Tx, Counter.C2)));
    }
    return maxDifference;
}

// Starts the motors, lets them spin up and counts the pulses for a second.
(double M1, double M2, int MaxDifference) Measure(Action start)
{
    start();
    Run(SpinUpMs);
    ushort first1 = ft.GetCounterValue(Tx, Counter.C1), first2 = ft.GetCounterValue(Tx, Counter.C2);
    Stopwatch watch = Stopwatch.StartNew();
    int maxDifference = Run(MeasureMs);
    double seconds = watch.Elapsed.TotalSeconds;
    ushort last1 = ft.GetCounterValue(Tx, Counter.C1), last2 = ft.GetCounterValue(Tx, Counter.C2);

    ft.StopMotor(Tx, Motor.M1);
    ft.StopMotor(Tx, Motor.M2);
    Run(300);
    return ((ushort)(last1 - first1) / seconds, (ushort)(last2 - first2) / seconds, maxDifference); // wraps at 65536
}

static double Rpm(double pulsesPerSecond) => pulsesPerSecond * 60.0 / PulsesPerTurn;

Console.WriteLine("        |       on their own       |       synchronized");
Console.WriteLine("  duty  |  M1 pulses/s rpm  M2 pulses/s rpm  |  pulses/s  rpm  max. difference");
try
{
    foreach (short duty in new short[] { 128, 256, 384, 512 })
    {
        var single = Measure(() =>
        {
            ft.SetMotorOutput(Tx, Motor.M1, duty);
            ft.SetMotorOutput(Tx, Motor.M2, duty);
        });
        var synced = Measure(() => ft.StartMotorSync(Tx, Motor.M1, Motor.M2, duty));
        if (cts.IsCancellationRequested)
        {
            break;
        }
        Console.WriteLine($"  {duty,4}  |  {single.M1,8:F1} {Rpm(single.M1),5:F1}    {single.M2,8:F1} {Rpm(single.M2),5:F1}    |" +
                          $"  {synced.M1,7:F1} {Rpm(synced.M1),5:F1}  {synced.MaxDifference,4}");
    }
}
finally
{
    ft.StopAllOutputs();
    ft.Update();
}

return 0;
