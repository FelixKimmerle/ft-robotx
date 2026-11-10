using System.IO.Ports;
using FtRoboTx.Extensions;
using FtRoboTx.Packets;

namespace FtRoboTx;

/// <summary>Serial connection to a fischertechnik ROBO TX Controller and its extensions.</summary>
public sealed class FtConnection : IDisposable
{
    private readonly SerialPort serialPort;
    private readonly List<Extension> extensions = [];
    private ushort sessionId;
    private ushort transactionId;
    private readonly byte[] messageIds; // last message id per controller
    // commands whose completion the events still have to report, per controller and motor/counter
    private readonly bool[,] motorPending;
    private readonly ResetState[,] counterResets;

    private enum ResetState : byte
    {
        Done,
        Requested,
        Confirmed, // the firmware echoed the reset id; the counter value follows with the next exchange
    }

    /// <summary>Longest wait for the firmware to take over the command ids of a new connection.</summary>
    private const int SettleTimeoutMs = 500;

    /// <summary>Set the environment variable FT_TRACE=1 to log every packet to stderr.</summary>
    private readonly bool trace = Environment.GetEnvironmentVariable("FT_TRACE") is { Length: > 0 } and not "0";

    public IReadOnlyList<Extension> Extensions => extensions;

    public Extension GetExtension(ExtensionIndex index) => Get(index);

    public FtConnection(string portName, int numExtensions = 1, int timeoutMs = 1000)
    {
        // Over Bluetooth the link is only established with the first packet, which takes a few seconds (about
        // 3.5 s measured), so the first reply gets more time than the rest.
        int firstReplyTimeoutMs = Math.Max(timeoutMs, 10000);
        serialPort = new SerialPort(portName)
        {
            ReadTimeout = firstReplyTimeoutMs,
            WriteTimeout = firstReplyTimeoutMs,
        };
        serialPort.Open();

        try
        {
            EchoResponse echoResponse = new();
            SendPacket(new EchoPacket(), echoResponse);
            if (!echoResponse.Success)
            {
                throw new InvalidOperationException("Echo failed!");
            }
            serialPort.ReadTimeout = timeoutMs;
            serialPort.WriteTimeout = timeoutMs;

            // The firmware ignores a message with the same id as the previous one, which may come from an earlier
            // connection; start at a random id so that's unlikely.
            messageIds = new byte[numExtensions];
            Array.Fill(messageIds, (byte)Random.Shared.Next(1, 256));

            for (uint i = 0; i < numExtensions; i++)
            {
                extensions.Add(new Extension(i));
            }

            SendConfig();

            SendPacket(new RequestInfoPacket(extensions), new RequestInfoResponse(extensions));

            // The firmware confirms motor commands and counter resets by echoing their ids. Until it has seen our ids
            // (starting at 0), it still echoes those of an earlier connection, which could pass for a confirmation of
            // our first command. It takes a few milliseconds; this also stops motors still running from an earlier
            // connection.
            motorPending = new bool[numExtensions, Extension.Motors];
            counterResets = new ResetState[numExtensions, Extension.Motors];
            long settleEnd = Environment.TickCount64 + SettleTimeoutMs;
            do
            {
                Exchange();
            } while (!extensions.All(extension => Enumerable.Range(0, Extension.Motors).All(i =>
                         extension.IsMotorCommandDone(i) && ResetConfirmed(extension, i))) &&
                     Environment.TickCount64 < settleEnd);
        }
        catch
        {
            serialPort.Dispose();
            throw;
        }
    }

    public void SetPWMOutput(ExtensionIndex index, PWM pwm, short duty)
    {
        Get(index).Output.PwmOutputValues[(int)pwm] = duty;
    }

    /// <summary>
    /// Runs a motor endlessly at the given duty (-512 ... 512, the sign gives the direction). Ends a distance or
    /// synchronization command of the motor; a motor it was synchronized with is stopped.
    /// </summary>
    public void SetMotorOutput(ExtensionIndex index, Motor motor, short duty) =>
        MotorCommand(index, extension => extension.SetMotorOutput((int)motor, duty));

    /// <summary>
    /// Runs a motor with encoder for <paramref name="distance"/> counter pulses, then the firmware brakes it
    /// (ftMscLib's StartMotorExCmd without slave). Resets the motor's counter. <see cref="IsMotorCommandDone"/> and
    /// <see cref="MotorReached"/> tell when it is there.
    /// </summary>
    public void StartMotorDistance(ExtensionIndex index, Motor motor, short duty, ushort distance)
    {
        MotorCommand(index, extension => extension.StartMotorDistance((int)motor, duty, distance));
        motorPending[(int)index, (int)motor] = distance != 0;
    }

    /// <summary>
    /// Runs two motors with encoders synchronized (ftMscLib's StartMotorExCmd): the firmware keeps their counters
    /// equal by slowing down the faster one, in either direction. With a distance, both stop after
    /// <paramref name="distance"/> pulses; with 0 they run until stopped. Resets both counters. Each motor's counter
    /// must be on the counter input with the same number (M1 on C1, ...).
    /// </summary>
    public void StartMotorSync(ExtensionIndex index, Motor master, Motor slave, short duty, ushort distance = 0,
                               SyncDirection direction = SyncDirection.Same)
    {
        MotorCommand(index, extension => extension.StartMotorSync((int)master, (int)slave, duty, distance, direction));
        motorPending[(int)index, (int)master] = motorPending[(int)index, (int)slave] = distance != 0;
    }

    /// <summary>
    /// Stops a motor (ftMscLib's StopMotorExCmd) and a motor synchronized with it. Cancelling a running distance or
    /// synchronization command resets the counters (it is a new motor command); otherwise they keep their values.
    /// </summary>
    public void StopMotor(ExtensionIndex index, Motor motor) =>
        MotorCommand(index, extension => extension.StopMotor((int)motor));

    /// <summary>
    /// True once the motor finished its last distance command (<see cref="StartMotorDistance"/>,
    /// <see cref="StartMotorSync"/> with a distance); also true for a motor without one. Updated by <see cref="Update"/>.
    /// </summary>
    public bool IsMotorCommandDone(ExtensionIndex index, Motor motor) => Get(index).IsMotorCommandDone((int)motor);

    /// <summary>
    /// Raised by <see cref="Update"/> when a motor finished a distance command (ftMscLib's SetCBMotorExReached); for
    /// a synchronized pair, once per motor. The handler may start the next command.
    /// </summary>
    public event Action<ExtensionIndex, Motor>? MotorReached;

    /// <summary>Raised by <see cref="Update"/> when a counter reset was carried out (ftMscLib's SetCBCounterResetted).</summary>
    public event Action<ExtensionIndex, Counter>? CounterReset;

    /// <summary>
    /// Shows a text (up to 98 ASCII characters) on the controller's display right away; an empty text clears it.
    /// </summary>
    public void ShowMessage(ExtensionIndex index, string text)
    {
        ref byte id = ref messageIds[(int)index];
        id = (byte)(id == 255 ? 1 : id + 1); // the firmware only shows a message with a new id

        MessageResponse response = new();
        SendPacket(new MessagePacket(Get(index).TransferAreaIndex, id, text), response);
        if (!response.Success)
        {
            throw new InvalidOperationException("Message not accepted");
        }
    }

    /// <summary>
    /// Asks the master which extensions are connected: element 0 is extension 1, ..., element 7 extension 8.
    /// </summary>
    public bool[] GetExtensionsOnline()
    {
        StateResponse response = new();
        SendPacket(new StatePacket(), response);
        if (!response.Success)
        {
            throw new InvalidOperationException("State request not accepted");
        }
        return response.ExtensionsOnline;
    }

    /// <summary>Requests a reset of the counter to 0. Takes effect with the next <see cref="Update"/>.</summary>
    public void ResetCounter(ExtensionIndex index, Counter counter)
    {
        Get(index).Output.CounterResetCommandId[(int)counter]++;
        counterResets[(int)index, (int)counter] = ResetState.Requested;
    }

    /// <summary>True once the last counter reset was carried out and <see cref="GetCounterValue"/> shows it.</summary>
    public bool IsCounterResetDone(ExtensionIndex index, Counter counter) =>
        counterResets[(int)index, (int)counter] == ResetState.Done;

    /// <summary>
    /// Pulses counted since the last reset or motor command, in either direction. The encoder motors give 3 pulses
    /// per turn of the motor, 63 1/3 per turn of the output shaft.
    /// </summary>
    public ushort GetCounterValue(ExtensionIndex index, Counter counter) => Get(index).Input.GetCounterValue((int)counter);

    /// <summary>
    /// Stops all motors and outputs of all extensions, including running distance and synchronization commands.
    /// Takes effect with the next <see cref="Update"/>.
    /// </summary>
    public void StopAllOutputs()
    {
        foreach (Extension extension in extensions)
        {
            extension.Output.StopAll();
        }
        Array.Clear(motorPending);
    }

    /// <summary>
    /// Configures a universal input (ftMscLib's SetFtUniConfig); sent with the next <see cref="Update"/>. The value
    /// reads 0 until the first measurement in the new mode (a few hundred milliseconds for the ultrasonic sensor and
    /// the NTC).
    /// </summary>
    public void SetInputMode(ExtensionIndex index, UniversalInput input, InputMode mode, bool digital = false) =>
        Get(index).Config.SetUniversalInputConfig((int)input, new UniversalInputConfig(mode, digital));

    /// <summary>
    /// Temperature in °C of an NTC on an input in analog resistance mode; see <see cref="Sensors.NtcCelsius"/>. Null
    /// before the first measurement or without sensor.
    /// </summary>
    public double? GetNtcTemperature(ExtensionIndex index, UniversalInput input, double r25 = Sensors.NtcR25,
                                     double b = Sensors.NtcB) =>
        Sensors.NtcCelsius(GetAnalogValue(index, input), r25, b);

    /// <summary>
    /// Distance in cm from an ultrasonic sensor on an input in ultrasonic mode; see <see cref="Sensors.UltrasonicCm"/>.
    /// </summary>
    public ushort? GetUltrasonicDistance(ExtensionIndex index, UniversalInput input) =>
        Sensors.UltrasonicCm(GetAnalogValue(index, input));

    public bool GetDigitalValue(ExtensionIndex index, UniversalInput input) => GetAnalogValue(index, input) != 0;

    public ushort GetAnalogValue(ExtensionIndex index, UniversalInput input) => Get(index).Input.GetUniversalInput((int)input);

    /// <summary>
    /// Sends the configuration if it changed, then the current output values, and reads back the inputs of all extensions.
    /// </summary>
    public void Update()
    {
        if (extensions.Any(extension => extension.Config.Changed))
        {
            SendConfig();
        }

        Exchange();
        Notify();
    }

    public void Dispose() => serialPort.Dispose();

    private void Exchange() => SendPacket(new OutputPacket(extensions), new InputResponse(extensions));

    private void Notify()
    {
        for (int e = 0; e < extensions.Count; e++)
        {
            // Cleared before the call, so the handler can start the next command.
            for (int m = 0; m < Extension.Motors; m++)
            {
                if (motorPending[e, m] && extensions[e].IsMotorCommandDone(m))
                {
                    motorPending[e, m] = false;
                    MotorReached?.Invoke((ExtensionIndex)e, (Motor)m);
                }
            }
            for (int c = 0; c < Extension.Motors; c++)
            {
                if (counterResets[e, c] == ResetState.Confirmed)
                {
                    counterResets[e, c] = ResetState.Done;
                    CounterReset?.Invoke((ExtensionIndex)e, (Counter)c);
                }
                else if (counterResets[e, c] == ResetState.Requested && ResetConfirmed(extensions[e], c))
                {
                    counterResets[e, c] = ResetState.Confirmed;
                }
            }
        }
    }

    /// <summary>Whether the firmware echoes the current counter reset id.</summary>
    private static bool ResetConfirmed(Extension extension, int counter) =>
        extension.Input.GetResetCounter(counter) == extension.Output.CounterResetCommandId[counter];

    /// <summary>
    /// Runs a motor command on the extension and forgets the pending reports of the motors it gave a new command.
    /// </summary>
    private void MotorCommand(ExtensionIndex index, Action<Extension> command)
    {
        Extension extension = Get(index);
        ushort[] ids = (ushort[])extension.Output.MotorCommandId.Clone();
        command(extension);
        for (int m = 0; m < Extension.Motors; m++)
        {
            motorPending[(int)index, m] &= extension.Output.MotorCommandId[m] == ids[m];
        }
    }

    private Extension Get(ExtensionIndex index) => extensions[(int)index];

    private void SendConfig()
    {
        ConfigResponse configResponse = new(extensions.Count);
        SendPacket(new ConfigPacket(extensions), configResponse);
        if (!configResponse.Success)
        {
            throw new InvalidOperationException("Config failed!");
        }

        foreach (Extension extension in extensions)
        {
            extension.Config.Changed = false;
        }
    }

    private void SendPacket(Packet packet, Response response)
    {
        packet.SessionId = sessionId;
        packet.TransactionId = transactionId;
        byte[] data = packet.GetBytes();
        if (trace)
        {
            Console.Error.WriteLine($"> {Convert.ToHexString(data)}");
        }

        serialPort.Write(data, 0, data.Length);

        byte[] receivedData = new byte[response.Length];
        int received = 0;
        try
        {
            while (received < receivedData.Length)
            {
                received += serialPort.Read(receivedData, received, receivedData.Length - received);
            }
        }
        catch (TimeoutException)
        {
            throw new TimeoutException(
                $"No response from controller (got {received} of {receivedData.Length} bytes: {Convert.ToHexString(receivedData, 0, received)})");
        }
        if (trace)
        {
            Console.Error.WriteLine($"< {Convert.ToHexString(receivedData)}");
        }

        response.Load(receivedData);
        sessionId = response.SessionId;
        transactionId = (ushort)(response.TransactionId + 1);
    }
}
