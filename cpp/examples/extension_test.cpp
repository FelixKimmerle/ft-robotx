// Master and one extension, each with a button on I1 and a lamp on O1, wired crosswise:
// the master's button switches the extension's lamp and the other way round.
// Usage: ftrobotx_extension_test [port], default /dev/ttyACM0 (USB)
#include <array>
#include <csignal>
#include <exception>
#include <iostream>
#include <optional>

#include "ftrobotx/FtConnection.hpp"

namespace
{

constexpr std::int16_t kOn = 512;

volatile std::sig_atomic_t stopRequested = 0;

void onSignal(int /*signal*/)
{
    stopRequested = 1;
}

const char* toString(ftrobotx::ExtensionIndex index)
{
    return index == ftrobotx::ExtensionIndex::Local ? "Local" : "Slave1";
}

} // namespace

int main(int argc, char* argv[])
{
    using namespace ftrobotx;

    constexpr std::array controllers{ExtensionIndex::Local, ExtensionIndex::Slave1};

    try
    {
        std::string portName = argc > 1 ? argv[1] : "/dev/ttyACM0";
        std::signal(SIGINT, onSignal);

        FtConnection ft(portName, controllers.size());
        for (ExtensionIndex index : controllers)
        {
            const Extension& extension = ft.extension(index);
            std::cout << toString(index) << ": " << extension.name << ' '
                      << (extension.version ? extension.version->toString() : "") << ' ' << extension.macAddress
                      << '\n';
        }
        std::cout << "Master button -> extension lamp, extension button -> master lamp. Press Ctrl+C to quit.\n";

        std::array<std::optional<bool>, controllers.size()> wasPressed;
        try
        {
            while (stopRequested == 0)
            {
                ft.update();

                for (std::size_t i = 0; i < controllers.size(); ++i)
                {
                    ExtensionIndex button = controllers[i];
                    ExtensionIndex lamp = controllers[(i + 1) % controllers.size()];

                    bool pressed = ft.digitalValue(button, UniversalInput::I1);
                    if (pressed != wasPressed[i])
                    {
                        std::cout << toString(button) << " button " << (pressed ? "pressed" : "released") << " -> "
                                  << toString(lamp) << " lamp " << (pressed ? "on" : "off") << '\n';
                        ft.setPwmOutput(lamp, PWM::O1, pressed ? kOn : std::int16_t{0});
                        wasPressed[i] = pressed;
                    }
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
