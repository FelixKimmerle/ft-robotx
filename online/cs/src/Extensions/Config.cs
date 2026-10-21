namespace FtRoboTx.Extensions;

public class Config
{
    private readonly UniversalInputConfig[] universalInputConfigs = new UniversalInputConfig[Extension.UniversalInputs];
    private readonly byte[] motorConfig = new byte[Extension.Motors];
    private readonly byte[] counterConfig = new byte[Extension.Motors];

    public bool Changed { get; set; } = true;

    public Config()
    {
        Array.Fill(universalInputConfigs, new UniversalInputConfig());
        Array.Fill(motorConfig, (byte)1);
        Array.Fill(counterConfig, (byte)1);
    }

    public UniversalInputConfig GetUniversalInputConfig(int index) => universalInputConfigs[index];

    public void SetUniversalInputConfig(int index, UniversalInputConfig value)
    {
        universalInputConfigs[index] = value;
        Changed = true;
    }

    public byte GetMotorConfig(int index) => motorConfig[index];

    public void SetMotorConfig(int index, byte value)
    {
        motorConfig[index] = value;
        Changed = true;
    }

    public byte GetCounterConfig(int index) => counterConfig[index];

    public void SetCounterConfig(int index, byte value)
    {
        counterConfig[index] = value;
        Changed = true;
    }
}
