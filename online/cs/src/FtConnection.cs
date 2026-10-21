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

    /// <summary>Set the environment variable FT_TRACE=1 to log every packet to stderr.</summary>
    private readonly bool trace = Environment.GetEnvironmentVariable("FT_TRACE") is { Length: > 0 } and not "0";

    public IReadOnlyList<Extension> Extensions => extensions;

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

    public void SetMotorOutput(ExtensionIndex index, Motor motor, short duty)
    {
        short[] pwm = Get(index).Output.PwmOutputValues;
        pwm[2 * (int)motor] = duty > 0 ? duty : (short)0;
        pwm[2 * (int)motor + 1] = duty > 0 ? (short)0 : (short)-duty;
    }

    public void SetMotorDistance(ExtensionIndex index, Motor motor, short duty, ushort distance)
    {
        SetMotorOutput(index, motor, duty);
        Output output = Get(index).Output;
        output.MotorDistanceValues[(int)motor] = distance;
        output.MotorCommandId[(int)motor]++;
    }

    public void SetMotorSync(ExtensionIndex index, Motor motor, Motor other, short duty, ushort distance = 0)
    {
        SetMotorOutput(index, motor, duty);
        SetMotorOutput(index, other, duty);

        Get(index).Output.MotorMasterValues[(int)motor] = (byte)(other + 1);
    }

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
    }

    /// <summary>
    /// Stops all motors and outputs of all extensions, including running distance commands. Takes effect with the
    /// next <see cref="Update"/>.
    /// </summary>
    public void StopAllOutputs()
    {
        foreach (Extension extension in extensions)
        {
            extension.Output.StopAll();
        }
    }

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

        SendPacket(new OutputPacket(extensions), new InputResponse(extensions));
    }

    public void Dispose() => serialPort.Dispose();

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
