// Measures the speed of the encoder motors on M1/C1 and M2/C2 at several duties, each on its own and synchronized.
// Usage: ftrobotx_motor_measure [-p PORT]; --help explains the options.
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <exception>
#include <iostream>
#include <string>

#include "Cli.hpp"
#include "ftrobotx/FtConnection.hpp"

#define DESCRIPTION                                                                                                    \
    "Measures the encoder motors on M1 (counter C1) and M2 (counter C2): at duties 128, 256, 384 and 512, both\n"      \
    "run on their own, then synchronized, for about 1.5 s each. Prints counter pulses per second, turns of the\n"      \
    "output shaft per minute and, when synchronized, the largest difference between the counters. Ctrl+C stops."

namespace
{

using Clock = std::chrono::steady_clock;

/// Pulses per turn of the encoder motor's output shaft.
constexpr double kPulsesPerTurn = 63.0 + 1.0 / 3.0;

constexpr std::chrono::milliseconds kSpinUp{500};
constexpr std::chrono::milliseconds kMeasure{1000};

volatile std::sig_atomic_t stopRequested = 0;

void onSignal(int /*signal*/)
{
    stopRequested = 1;
}

struct Result
{
    double pulsesPerSecond[2] = {};
    int maxDifference = 0;
};

/// Calls update() for `duration`. Tracks the largest counter difference if `difference` is given.
void run(ftrobotx::FtConnection& ft, std::chrono::milliseconds duration, int* difference = nullptr)
{
    using namespace ftrobotx;
    auto end = Clock::now() + duration;
    while (Clock::now() < end && stopRequested == 0)
    {
        ft.update();
        if (difference != nullptr)
        {
            int d = std::abs(ft.counterValue(ExtensionIndex::Local, Counter::C1) -
                             ft.counterValue(ExtensionIndex::Local, Counter::C2));
            *difference = std::max(*difference, d);
        }
    }
}

/// Starts the motors with `start`, lets them spin up and counts the pulses for a second.
template <typename Start> Result measure(ftrobotx::FtConnection& ft, bool synchronized, Start start)
{
    using namespace ftrobotx;
    constexpr auto tx = ExtensionIndex::Local;
    Result result;

    start();
    run(ft, kSpinUp);
    std::uint16_t first[2] = {ft.counterValue(tx, Counter::C1), ft.counterValue(tx, Counter::C2)};
    auto begin = Clock::now();
    run(ft, kMeasure, synchronized ? &result.maxDifference : nullptr);
    double seconds = std::chrono::duration<double>(Clock::now() - begin).count();
    std::uint16_t last[2] = {ft.counterValue(tx, Counter::C1), ft.counterValue(tx, Counter::C2)};
    for (int i = 0; i < 2; ++i)
    {
        result.pulsesPerSecond[i] = static_cast<std::uint16_t>(last[i] - first[i]) / seconds; // wraps at 65536
    }

    ft.stopMotor(tx, Motor::M1);
    ft.stopMotor(tx, Motor::M2);
    run(ft, std::chrono::milliseconds(300));
    return result;
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

        std::printf("        |       on their own       |       synchronized\n");
        std::printf("  duty  |  M1 pulses/s rpm  M2 pulses/s rpm  |  pulses/s  rpm  max. difference\n");
        try
        {
            for (std::int16_t duty : {128, 256, 384, 512})
            {
                Result single = measure(ft, false, [&] {
                    ft.setMotorOutput(tx, Motor::M1, duty);
                    ft.setMotorOutput(tx, Motor::M2, duty);
                });
                Result synced = measure(ft, true, [&] { ft.startMotorSync(tx, Motor::M1, Motor::M2, duty); });
                if (stopRequested != 0)
                {
                    break;
                }
                auto rpm = [](double pulsesPerSecond) { return pulsesPerSecond * 60.0 / kPulsesPerTurn; };
                std::printf("  %4d  |  %8.1f %5.1f    %8.1f %5.1f    |  %7.1f %5.1f  %4d\n", duty,
                            single.pulsesPerSecond[0], rpm(single.pulsesPerSecond[0]), single.pulsesPerSecond[1],
                            rpm(single.pulsesPerSecond[1]), synced.pulsesPerSecond[0], rpm(synced.pulsesPerSecond[0]),
                            synced.maxDifference);
                std::fflush(stdout);
            }
        }
        catch (...)
        {
            ft.stopAllOutputs();
            ft.update();
            throw;
        }

        ft.stopAllOutputs();
        ft.update();
    }
    catch (const std::exception& e)
    {
        std::cerr << "Error: " << e.what() << '\n';
        return 1;
    }
    return 0;
}
