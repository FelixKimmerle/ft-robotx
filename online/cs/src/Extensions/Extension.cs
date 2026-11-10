namespace FtRoboTx.Extensions;

/// <summary>A TX controller (local master or one of the extension slaves) and its transfer area.</summary>
public class Extension(uint transferAreaIndex)
{
    public const int Motors = 4;
    public const int UniversalInputs = 8;

    public uint TransferAreaIndex { get; } = transferAreaIndex;

    public string? Name { get; set; }
    public FirmwareVersion? Version { get; set; }
    public string? MacAddress { get; set; }

    public Config Config { get; } = new();
    public Output Output { get; } = new();
    public Input Input { get; } = new();

    // Motor commands on the output values; FtConnection has the same functions by ExtensionIndex and Motor, and
    // documents them. These are for code that works on an Extension directly.

    public void SetMotorOutput(int motor, short duty)
    {
        ReleaseMotor(motor);
        Output.PwmOutputValues[2 * motor] = duty > 0 ? duty : (short)0;
        Output.PwmOutputValues[2 * motor + 1] = duty > 0 ? (short)0 : (short)-duty;
    }

    public void StartMotorDistance(int motor, short duty, ushort distance)
    {
        SetMotorOutput(motor, duty);
        Output.MotorDistanceValues[motor] = distance;
        Output.MotorCommandId[motor]++;
    }

    public void StartMotorSync(int master, int slave, short duty, ushort distance = 0,
                               SyncDirection direction = SyncDirection.Same)
    {
        if (master == slave)
        {
            throw new ArgumentException("A motor can't be synchronized with itself", nameof(slave));
        }
        SetMotorOutput(master, duty);
        SetMotorOutput(slave, direction == SyncDirection.Same ? duty : (short)-duty);

        // The slave's entry names the master (index + 1). Both get the same distance: if they differ, the pair stops
        // at the shorter one and the other never reports reaching its target.
        Output.MotorMasterValues[slave] = (byte)(master + 1);
        foreach (int motor in new[] { master, slave })
        {
            Output.MotorDistanceValues[motor] = distance;
            Output.MotorCommandId[motor]++;
        }
    }

    public void StopMotor(int motor)
    {
        // A motor that finished its distance command stands already; a new command would only reset the counters.
        bool Finished(int i) => Output.MotorDistanceValues[i] != 0 && IsMotorCommandDone(i);
        int? partner = GetSyncPartner(motor);
        if (Finished(motor) && (partner is null || Finished(partner.Value)))
        {
            Output.PwmOutputValues[2 * motor] = 0;
            Output.PwmOutputValues[2 * motor + 1] = 0;
            return;
        }
        SetMotorOutput(motor, 0); // a plain motor keeps its counter, a running command is cancelled
    }

    /// <summary>The motor synchronized with this one (as master or slave), if any.</summary>
    public int? GetSyncPartner(int motor)
    {
        for (int other = 0; other < Motors; other++)
        {
            if (other != motor && (Output.MotorMasterValues[other] == motor + 1 || Output.MotorMasterValues[motor] == other + 1))
            {
                return other;
            }
        }
        return null;
    }

    /// <summary>
    /// Whether the motor finished its last distance command, i.e. the firmware confirmed the current motor command
    /// id. Also true for commands without distance (they are confirmed right away) and before the first command.
    /// </summary>
    public bool IsMotorCommandDone(int motor) => Input.GetMotorReachedDestination(motor) == Output.MotorCommandId[motor];

    /// <summary>Ends the distance/synchronization command of a motor and stops the motors synchronized with it.</summary>
    private void ReleaseMotor(int motor)
    {
        bool commandRunning = Output.MotorDistanceValues[motor] != 0 || Output.MotorMasterValues[motor] != 0;
        while (GetSyncPartner(motor) is int partner)
        {
            Output.StopMotor(partner); // the firmware would hold it at our counter otherwise; clears its master entry
            if (Output.MotorMasterValues[motor] == partner + 1)
            {
                Output.MotorMasterValues[motor] = 0;
            }
            commandRunning = true;
        }
        if (commandRunning)
        {
            Output.MotorDistanceValues[motor] = 0;
            Output.MotorMasterValues[motor] = 0;
            Output.MotorCommandId[motor]++; // the firmware only takes over the new settings with a new command
        }
    }
}
