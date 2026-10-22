// Lamp on O1 is on while the button on I1 is pressed.
// Usage: ftrobotx_button_lamp [-p PORT]; --help explains the options.
#include <csignal>
#include <exception>
#include <iostream>
#include <optional>

#include "Cli.hpp"
#include "ftrobotx/FtConnection.hpp"

#define DESCRIPTION "Lamp on O1 lights while the button on I1 is pressed. Ctrl+C quits and switches the lamp off."

namespace
{

constexpr std::int16_t kOn = 512;

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
        const Extension& master = ft.extensions().front();
        std::cout << "Connected to " << master.name << ' ' << (master.version ? master.version->toString() : "")
                  << ". Press Ctrl+C to quit.\n";

        std::optional<bool> wasPressed;
        try
        {
            while (stopRequested == 0)
            {
                ft.update();

                bool pressed = ft.digitalValue(ExtensionIndex::Local, UniversalInput::I1);
                if (pressed != wasPressed)
                {
                    std::cout << (pressed ? "Button pressed, lamp on" : "Button released, lamp off") << '\n';
                    ft.setPwmOutput(ExtensionIndex::Local, PWM::O1, pressed ? kOn : std::int16_t{0});
                    wasPressed = pressed;
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
