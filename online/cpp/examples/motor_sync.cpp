// Two encoder motors on M1/C1 and M2/C2 run synchronized back and forth; the button on I1 starts and stops them.
// Usage: ftrobotx_motor_sync [-p PORT] [-d DUTY] [-n PULSES] [-o]; --help explains the options.
#include <chrono>
#include <csignal>
#include <cstdlib>
#include <exception>
#include <iostream>
#include <string>

#include "Cli.hpp"
#include "ftrobotx/FtConnection.hpp"

#define DESCRIPTION                                                                                                    \
    "Encoder motors on M1 (counter C1) and M2 (counter C2) run synchronized back and forth, PULSES counter pulses\n"   \
    "each way; the button on I1 starts and stops them. Shows the counters while they run. Ctrl+C quits."

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
    using Clock = std::chrono::steady_clock;

    std::string portOption;
    std::string dutyOption = "400";
    std::string pulsesOption = "190";
    std::string oppositeOption;
    int exitCode = 0;
    if (!cli::parse(argc, argv, "[-p PORT] [-d DUTY] [-n PULSES] [-o]", DESCRIPTION,
                    {{"-p", "--port", "PORT", cli::kPortHelp, &portOption},
                     {"-d", "--duty", "DUTY", "Speed, 1 ... 512. Default: 400.", &dutyOption},
                     {"-n", "--pulses", "PULSES", "Counter pulses each way. Default: 190 (3 turns of the shaft).",
                      &pulsesOption},
                     {"-o", "--opposite", nullptr, "M2 turns the other way than M1 (a robot turning on the spot).",
                      &oppositeOption}},
                    nullptr, exitCode))
    {
        return exitCode;
    }

    try
    {
        auto duty = static_cast<std::int16_t>(std::stoi(dutyOption));
        auto pulses = static_cast<std::uint16_t>(std::stoi(pulsesOption));
        SyncDirection direction = oppositeOption.empty() ? SyncDirection::Same : SyncDirection::Opposite;
        if (duty < 1 || duty > 512 || pulses < 1)
        {
            std::cerr << "DUTY must be 1 ... 512 and PULSES at least 1; see --help\n";
            return 2;
        }

        std::string portName = cli::choosePort(portOption);
        if (portName.empty())
        {
            return 1;
        }
        std::signal(SIGINT, onSignal);

        FtConnection ft(portName);
        const Extension& master = ft.extensions().front();
        std::cout << "Connected to " << master.name << ' ' << (master.version ? master.version->toString() : "")
                  << ". Press the button on I1 to start and stop, Ctrl+C to quit.\n";

        constexpr auto tx = ExtensionIndex::Local;
        bool running = false;
        int sign = 1;
        int legs = 0;
        auto startLeg = [&] {
            ft.startMotorSync(tx, Motor::M1, Motor::M2, static_cast<std::int16_t>(sign * duty), pulses, direction);
        };

        // Called from update() once per motor; the next leg starts when both arrived.
        ft.setMotorReachedCallback([&](ExtensionIndex /*index*/, Motor /*motor*/) {
            if (!running || !ft.motorCommandDone(tx, Motor::M1) || !ft.motorCommandDone(tx, Motor::M2))
            {
                return;
            }
            ++legs;
            std::cout << "Leg " << legs << " done: C1 = " << ft.counterValue(tx, Counter::C1)
                      << ", C2 = " << ft.counterValue(tx, Counter::C2) << '\n';
            sign = -sign;
            startLeg();
        });

        bool wasPressed = false;
        auto nextReport = Clock::now();
        try
        {
            while (stopRequested == 0)
            {
                ft.update();

                bool pressed = ft.digitalValue(tx, UniversalInput::I1);
                if (pressed && !wasPressed)
                {
                    running = !running;
                    if (running)
                    {
                        std::cout << "Start\n";
                        startLeg();
                    }
                    else
                    {
                        std::cout << "Stop\n";
                        ft.stopMotor(tx, Motor::M1); // stops M2 as well
                    }
                }
                wasPressed = pressed;

                if (running && Clock::now() >= nextReport)
                {
                    int c1 = ft.counterValue(tx, Counter::C1);
                    int c2 = ft.counterValue(tx, Counter::C2);
                    std::cout << "  C1 = " << c1 << ", C2 = " << c2 << ", difference " << c1 - c2 << '\n';
                    nextReport = Clock::now() + std::chrono::milliseconds(500);
                }
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
