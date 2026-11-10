// Two encoder motors on M1/C1 and M2/C2 run synchronized back and forth; the button on I1 starts and stops them.
// Usage: dotnet run -- [-p PORT]; --help explains the options.
using System.Diagnostics;
using FtRoboTx;
using FtRoboTx.Extensions;

const short Duty = 400;
const ushort Pulses = 190; // 3 turns of the output shaft
const ExtensionIndex Tx = ExtensionIndex.Local;

string? portName = CommandLine.ParsePort(args, "MotorSync",
    "Encoder motors on M1 (counter C1) and M2 (counter C2) run synchronized back and forth, 190 counter pulses\n" +
    "each way; the button on I1 starts and stops them. Shows the counters while they run. Ctrl+C quits.",
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
Console.WriteLine($"Connected to {ft.Extensions[0].Name} {ft.Extensions[0].Version}. " +
                  "Press the button on I1 to start and stop, Ctrl+C to quit.");

bool running = false;
int sign = 1;
int legs = 0;
void StartLeg() => ft.StartMotorSync(Tx, Motor.M1, Motor.M2, (short)(sign * Duty), Pulses);

// Raised by Update() once per motor; the next leg starts when both arrived.
ft.MotorReached += (_, _) =>
{
    if (!running || !ft.IsMotorCommandDone(Tx, Motor.M1) || !ft.IsMotorCommandDone(Tx, Motor.M2))
    {
        return;
    }
    legs++;
    Console.WriteLine($"Leg {legs} done: C1 = {ft.GetCounterValue(Tx, Counter.C1)}, C2 = {ft.GetCounterValue(Tx, Counter.C2)}");
    sign = -sign;
    StartLeg();
};

bool wasPressed = false;
Stopwatch report = Stopwatch.StartNew();
try
{
    while (!cts.IsCancellationRequested)
    {
        ft.Update();

        bool pressed = ft.GetDigitalValue(Tx, UniversalInput.I1);
        if (pressed && !wasPressed)
        {
            running = !running;
            if (running)
            {
                Console.WriteLine("Start");
                StartLeg();
            }
            else
            {
                Console.WriteLine("Stop");
                ft.StopMotor(Tx, Motor.M1); // stops M2 as well
            }
        }
        wasPressed = pressed;

        if (running && report.ElapsedMilliseconds >= 500)
        {
            int c1 = ft.GetCounterValue(Tx, Counter.C1);
            int c2 = ft.GetCounterValue(Tx, Counter.C2);
            Console.WriteLine($"  C1 = {c1}, C2 = {c2}, difference {c1 - c2}");
            report.Restart();
        }
    }
}
finally
{
    ft.StopAllOutputs();
    ft.Update();
}

return 0;
