using System.Buffers.Binary;
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

    // Bluetooth reports read while waiting for other replies; the events get them in Update(). Only the master's
    // radio is reachable this way.
    private readonly List<BtEvent> btEvents = [];
    private readonly List<BtScanResult> btScanResults = [];

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

    // Bluetooth messages between ROBO TX Controllers (ftMscLib's Bluetooth Message API), see hybrid/relay/README.md.
    // The commands only send the request; results and everything the controller reports by itself (received
    // messages, incoming connections, scan results) come through the events, raised by Update(). Channels are 1 ... 8.
    // Typical use: the receiver calls BtListen with the sender's address, the sender BtConnect; once connected (events
    // ConnectionIndication / Success), the receiver calls BtStartReceive. BtConnect to the controller's own address
    // gives a loopback connection, for testing with one controller.

    /// <summary>Bluetooth channels are numbered 1 ... 8.</summary>
    public const int BtChannels = 8;

    /// <summary>The receiving controller keeps only the first 16 bytes of a message (measured).</summary>
    public const int BtMaxMessage = 16;

    /// <summary>Raised by <see cref="Update"/> for every Bluetooth event.</summary>
    public event Action<BtEvent>? BtEventReceived;

    /// <summary>Raised by <see cref="Update"/> for every scan status and found device.</summary>
    public event Action<BtScanResult>? BtScanResultReceived;

    /// <summary>Searches for Bluetooth devices; results come through <see cref="BtScanResultReceived"/>.</summary>
    public void BtStartScan() => SendRequest(new BtPacket(BtPacket.Scan));
    public void BtCancelScan() => SendRequest(new BtPacket(BtPacket.CancelScan));

    public void BtConnect(int channel, BtAddress address) =>
        SendRequest(new BtPacket(BtPacket.Connect, CheckChannel(channel), address));
    public void BtDisconnect(int channel) => SendRequest(new BtPacket(BtPacket.Disconnect, CheckChannel(channel)));

    /// <summary>Accepts a connection from <paramref name="remote"/> on the channel.</summary>
    public void BtListen(int channel, BtAddress remote) =>
        SendRequest(new BtPacket(BtPacket.Listen, CheckChannel(channel), remote));
    public void BtStopListen(int channel) => SendRequest(new BtPacket(BtPacket.StopListen, CheckChannel(channel)));

    /// <summary>Reports received messages through <see cref="BtEventReceived"/>. Only works on a connected channel.</summary>
    public void BtStartReceive(int channel) => SendRequest(new BtPacket(BtPacket.StartReceive, CheckChannel(channel)));
    public void BtStopReceive(int channel) => SendRequest(new BtPacket(BtPacket.StopReceive, CheckChannel(channel)));

    /// <summary>
    /// Sends up to <see cref="BtMaxMessage"/> (16) bytes; the receiving controller drops anything beyond that. The send
    /// event (status Success) confirms it.
    /// </summary>
    public void BtSend(int channel, ReadOnlySpan<byte> message)
    {
        if (message.Length > BtMaxMessage)
        {
            throw new ArgumentException(
                $"Bluetooth message of {message.Length} bytes; the receiving controller keeps only {BtMaxMessage}",
                nameof(message));
        }
        SendRequest(new BtPacket(CheckChannel(channel), message));
    }

    public void BtSend(int channel, string message) => BtSend(channel, System.Text.Encoding.ASCII.GetBytes(message));

    public void Dispose() => serialPort.Dispose();

    private void Exchange() => SendPacket(new OutputPacket(extensions), new InputResponse(extensions));

    private void Notify()
    {
        // Taken out first: a handler may send further commands, whose replies are queued anew.
        BtScanResult[] scanResults = [.. btScanResults];
        btScanResults.Clear();
        foreach (BtScanResult result in scanResults)
        {
            BtScanResultReceived?.Invoke(result);
        }
        BtEvent[] events = [.. btEvents];
        btEvents.Clear();
        foreach (BtEvent btEvent in events)
        {
            BtEventReceived?.Invoke(btEvent);
        }

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

    /// <summary>Sends a packet without waiting for a reply (Bluetooth commands; their replies come as events).</summary>
    private void SendRequest(Packet packet)
    {
        packet.SessionId = sessionId;
        packet.TransactionId = 0; // like ftMscLib for Bluetooth commands; the reply doesn't advance the ids
        byte[] data = packet.GetBytes();
        if (trace)
        {
            Console.Error.WriteLine($"> {Convert.ToHexString(data)}");
        }
        serialPort.Write(data, 0, data.Length);
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

        // Bluetooth replies and reports may come first; they are queued for the events.
        byte[] receivedData;
        while (true)
        {
            receivedData = ReadPacket(response.Length);
            if (!BtPacket.IsReply(BinaryPrimitives.ReadInt32LittleEndian(receivedData.AsSpan(16))))
            {
                break;
            }
            QueueBtReply(receivedData);
        }
        if (receivedData.Length != response.Length)
        {
            throw new InvalidOperationException(
                $"Unexpected response of {receivedData.Length} bytes (expected {response.Length}): {Convert.ToHexString(receivedData)}");
        }

        response.Load(receivedData);
        sessionId = response.SessionId;
        transactionId = (ushort)(response.TransactionId + 1);
    }

    /// <summary>Reads the next packet from the controller, skipping bytes until the 02 55 start.</summary>
    private byte[] ReadPacket(int expectedLength)
    {
        byte[] start = new byte[4]; // 02 55 and the length (big endian) of what follows up to the checksum
        int received = ReadFully(start, 0, 4, expectedLength);
        while (start[0] != 0x02 || start[1] != 0x55)
        {
            if (trace)
            {
                Console.Error.WriteLine($"skipping {start[0]:X2}");
            }
            Array.Copy(start, 1, start, 0, 3);
            received = 3 + ReadFully(start, 3, 1, expectedLength);
        }

        int length = (start[2] << 8) | start[3];
        if (length < 20)
        {
            throw new InvalidOperationException($"Invalid packet length {length}: {Convert.ToHexString(start)}");
        }
        byte[] packet = new byte[4 + length + 3]; // + checksum and end byte
        start.CopyTo(packet, 0);
        ReadFully(packet, 4, packet.Length - 4, expectedLength);
        if (trace)
        {
            Console.Error.WriteLine($"< {Convert.ToHexString(packet)}");
        }
        return packet;
    }

    private int ReadFully(byte[] buffer, int offset, int count, int expectedLength)
    {
        int received = 0;
        try
        {
            while (received < count)
            {
                received += serialPort.Read(buffer, offset + received, count - received);
            }
        }
        catch (TimeoutException)
        {
            throw new TimeoutException(
                $"No response from controller (got {offset + received} of {expectedLength} bytes: {Convert.ToHexString(buffer, 0, offset + received)})");
        }
        return received;
    }

    private void QueueBtReply(byte[] bytes)
    {
        BtReply reply = new();
        reply.Load(bytes);
        if (reply.IsScan)
        {
            btScanResults.Add(reply.ScanResult!);
        }
        else
        {
            btEvents.Add(reply.Event!);
        }
    }

    private static int CheckChannel(int channel) =>
        channel is >= 1 and <= BtChannels
            ? channel
            : throw new ArgumentOutOfRangeException(nameof(channel), channel, "Bluetooth channel must be 1 ... 8");

}
