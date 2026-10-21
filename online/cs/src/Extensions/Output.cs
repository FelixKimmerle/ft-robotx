namespace FtRoboTx.Extensions;

public class Output
{
    public ushort[] CounterResetCommandId { get; } = new ushort[Extension.Motors];
    public byte[] MotorMasterValues { get; } = new byte[Extension.Motors];
    public short[] PwmOutputValues { get; } = new short[Extension.Motors * 2];
    public ushort[] MotorDistanceValues { get; } = new ushort[Extension.Motors];
    public ushort[] MotorCommandId { get; } = new ushort[Extension.Motors];

    /// <summary>
    /// Stops a motor like ftMscLib's StopMotorExCmd: duty and distance to 0, and a new motor command, so a running
    /// distance/synchronization command is cancelled too.
    /// </summary>
    public void StopMotor(int motor)
    {
        PwmOutputValues[2 * motor] = 0;
        PwmOutputValues[2 * motor + 1] = 0;
        MotorDistanceValues[motor] = 0;
        MotorCommandId[motor]++;
    }

    /// <summary>Stops all motors and outputs.</summary>
    public void StopAll()
    {
        for (int motor = 0; motor < Extension.Motors; motor++)
        {
            StopMotor(motor);
        }
    }
}
