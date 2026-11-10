namespace FtRoboTx;

/// <summary>Conversions for the values of fischertechnik sensors on the universal inputs.</summary>
public static class Sensors
{
    /// <summary>Analog resistance mode (not digital): 10 ... 4999 Ohm; 5000 means overload (open input).</summary>
    public const ushort ResistanceOverload = 5000;

    /// <summary>Ultrasonic mode: 2 ... 1023 cm.</summary>
    public const ushort UltrasonicMin = 2;
    public const ushort UltrasonicMax = 1023;
    public const ushort UltrasonicOverload = 1024; // nothing in range
    public const ushort UltrasonicNotPresent = 4096;

    /// <summary>
    /// The fischertechnik NTC: 1.5 kOhm at 25 °C, B constant 3900 K. Each sensor differs a little; pass your own
    /// values to <see cref="NtcKelvin"/>/<see cref="NtcCelsius"/> to calibrate.
    /// </summary>
    public const double NtcR25 = 1500.0;
    public const double NtcB = 3900.0;

    public const double ZeroCelsius = 273.15; // K

    /// <summary>
    /// Temperature in Kelvin of an NTC from its resistance (B parameter equation). Null for 0 (right after switching
    /// the input mode there is no measurement yet) and for an overload (no sensor connected).
    /// </summary>
    public static double? NtcKelvin(ushort ohm, double r25 = NtcR25, double b = NtcB)
    {
        if (ohm == 0 || ohm >= ResistanceOverload)
        {
            return null;
        }
        return 1.0 / (1.0 / (25.0 + ZeroCelsius) + Math.Log(ohm / r25) / b);
    }

    /// <summary>Temperature in °C of an NTC from its resistance; see <see cref="NtcKelvin"/>.</summary>
    public static double? NtcCelsius(ushort ohm, double r25 = NtcR25, double b = NtcB) =>
        NtcKelvin(ohm, r25, b) - ZeroCelsius;

    /// <summary>
    /// Distance in cm from the ultrasonic sensor's value. Null if there is no measurement yet (0, right after
    /// switching the input mode), nothing is in range (1024) or no sensor is connected (4096).
    /// </summary>
    public static ushort? UltrasonicCm(ushort value) => value is >= UltrasonicMin and <= UltrasonicMax ? value : null;
}
