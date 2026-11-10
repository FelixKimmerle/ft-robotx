// Ultrasonic distance sensor on I2 and NTC (temperature) on I3.
// Usage: ftrobotx_sensors [-p PORT]; --help explains the options.
#include <chrono>
#include <csignal>
#include <cstdio>
#include <exception>
#include <iostream>
#include <optional>

#include "Cli.hpp"
#include "ftrobotx/FtConnection.hpp"

#define DESCRIPTION                                                                                                    \
    "Shows the distance from the ultrasonic sensor on I2 and the temperature from the NTC on I3 twice a second.\n"     \
    "Ctrl+C quits."

namespace
{

volatile std::sig_atomic_t stopRequested = 0;

void onSignal(int /*signal*/)
{
    stopRequested = 1;
}

} // namespace

int main(int argc, char* argv[])
{
    using namespace ftrobotx;

    std::string portOption;
    int exitCode = 0;
    if (!cli::parse(argc, argv, "[-p PORT]", DESCRIPTION, {{"-p", "--port", "PORT", cli::kPortHelp, &portOption}},
                    nullptr, exitCode))
    {
        return exitCode;
    }

    try
    {
        std::string portName = cli::choosePort(portOption);
        if (portName.empty())
        {
            return 1;
        }
        std::signal(SIGINT, onSignal);

        FtConnection ft(portName);
        constexpr auto tx = ExtensionIndex::Local;
        ft.setInputMode(tx, UniversalInput::I2, InputMode::Ultrasonic);
        ft.setInputMode(tx, UniversalInput::I3, InputMode::Resistance); // analog, in Ohm
        std::cout << "Ultrasonic sensor on I2, NTC on I3. Press Ctrl+C to quit.\n";

        auto nextReport = std::chrono::steady_clock::now();
        while (stopRequested == 0)
        {
            ft.update();
            if (std::chrono::steady_clock::now() < nextReport)
            {
                continue;
            }
            nextReport += std::chrono::milliseconds(500);

            std::optional<std::uint16_t> distance = ft.ultrasonicDistance(tx, UniversalInput::I2);
            std::optional<double> temperature = ft.ntcTemperature(tx, UniversalInput::I3);
            // For your own sensor: ft.ntcTemperature(tx, UniversalInput::I3, 1500.0 /* Ohm at 25 °C */, 3900.0 /* B */)
            if (distance)
            {
                std::printf("Distance %4u cm", *distance);
            }
            else
            {
                std::printf("Distance    - cm");
            }
            if (temperature)
            {
                std::printf("   Temperature %5.1f C (%u Ohm)\n", *temperature, ft.analogValue(tx, UniversalInput::I3));
            }
            else
            {
                std::printf("   Temperature     - C\n");
            }
            std::fflush(stdout);
        }
    }
    catch (const std::exception& e)
    {
        std::cerr << "Error: " << e.what() << '\n';
        return 1;
    }
    return 0;
}
