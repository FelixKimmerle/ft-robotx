namespace FtRoboTxOnline.Extensions;

public class Input
{
    private readonly ushort[] universalInputs = new ushort[Extension.UniversalInputs];
    private readonly byte[] counterInputs = new byte[Extension.Motors];
    private readonly ushort[] counterValues = new ushort[Extension.Motors];
    private readonly ushort[] motorReachedDestination = new ushort[Extension.Motors];
    private readonly ushort[] resetCounters = new ushort[Extension.Motors];
    private ushort displayButtonLeft;
    private ushort displayButtonRight;

    /// <summary>Set whenever an input value changes; reset it after handling the change.</summary>
    public bool Changed { get; set; } = true;

    public ushort GetUniversalInput(int index) => universalInputs[index];
    public void SetUniversalInput(int index, ushort value) => Set(ref universalInputs[index], value);

    public byte GetCounterInput(int index) => counterInputs[index];
    public void SetCounterInput(int index, byte value) => Set(ref counterInputs[index], value);

    public ushort GetCounterValue(int index) => counterValues[index];
    public void SetCounterValue(int index, ushort value) => Set(ref counterValues[index], value);

    public ushort GetDisplayButtonLeft() => displayButtonLeft;
    public void SetDisplayButtonLeft(ushort value) => Set(ref displayButtonLeft, value);

    public ushort GetDisplayButtonRight() => displayButtonRight;
    public void SetDisplayButtonRight(ushort value) => Set(ref displayButtonRight, value);

    public ushort GetMotorReachedDestination(int index) => motorReachedDestination[index];
    public void SetMotorReachedDestination(int index, ushort value) => Set(ref motorReachedDestination[index], value);

    public ushort GetResetCounter(int index) => resetCounters[index];
    public void SetResetCounter(int index, ushort value) => Set(ref resetCounters[index], value);

    private void Set<T>(ref T field, T value) where T : IEquatable<T>
    {
        Changed |= !field.Equals(value);
        field = value;
    }
}
