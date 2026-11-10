// Hybrid mode: the button on I1 of the relay TX switches the lamp on O1 of the
// model TX (which runs tx/relay_model.c). Build: hybrid/relay (see README.md).
// Usage: ftrobotx_relay_button_lamp [-p PORT] -a ADDRESS; --help explains it.
#include <csignal>
#include <cstdio>
#include <exception>
#include <iostream>
#include <optional>
#include <string>

#include "Cli.hpp"
#include "ftrobotx/relay/RelayConnection.hpp"

#define DESCRIPTION                                                                                                    \
    "The button on I1 of the relay TX (connected to this PC) switches the lamp "                                       \
    "on O1 of the model TX, which runs\n"                                                                              \
    "RelayModel (hybrid/relay/tx) and gets the command over Bluetooth through "                                        \
    "the relay. Ctrl+C quits and switches\n"                                                                           \
    "the lamp off."

namespace
{

constexpr std::int16_t kOn = 512;

const std::string kRelayPortHelp =
    "Port of the relay TX (the one connected to this PC): /dev/ttyACM0 (USB),\n"
    "                        bt:00:13:7B:53:DB:5E (Bluetooth, Linux) or COM3 (Windows).\n"
    "                        Default: the first ROBO TX Controller found on USB.";

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
    std::string addressOption;
    int exitCode = 0;
    if (!cli::parse(argc, argv, "[-p PORT] -a ADDRESS", DESCRIPTION,
                    {{"-p", "--port", "PORT", kRelayPortHelp.c_str(), &portOption},
                     {"-a", "--address", "ADDRESS",
                      "Bluetooth address of the model TX (running RelayModel), not the relay's,\n"
                      "                        e.g. 00:13:7B:48:B2:E2.",
                      &addressOption}},
                    nullptr, exitCode))
    {
        return exitCode;
    }
    std::optional<BtAddress> address = parseBtAddress(addressOption);
    if (!address)
    {
        std::cerr << "The model's Bluetooth address is needed: -a 00:13:7B:...; "
                     "see --help\n";
        return 2;
    }

    try
    {
        std::string portName = cli::choosePort(portOption);
        if (portName.empty())
        {
            return 1;
        }
        std::signal(SIGINT, onSignal);

        FtConnection relay(portName);
        RelayConnection model(relay, *address);
        model.setErrorCallback([](const std::string& message) { std::cerr << "Relay: " << message << '\n'; });
        std::cout << "Relay " << relay.extensions().front().name << ", connecting to the model TX "
                  << toString(*address) << "...\n";
        model.connect();
        std::cout << "Connected. Press the button on I1 of the relay; Ctrl+C quits.\n";

        // O1 and O2 of the model as single outputs, not motor M1.
        model.setMotorMode(ExtensionIndex::Local, Motor::M1, false);
        model.showMessage("Lamp on O1 follows\nthe relay's button");

        std::optional<bool> wasPressed;
        while (stopRequested == 0 && model.connected())
        {
            model.update(); // also the relay's online cycle, so its inputs are fresh

            bool pressed = relay.digitalValue(ExtensionIndex::Local, UniversalInput::I1);
            if (pressed != wasPressed)
            {
                model.setPwmOutput(ExtensionIndex::Local, PWM::O1, pressed ? kOn : std::int16_t{0});
                wasPressed = pressed;
                std::printf("%s (last command round trip %.0f ms)\n",
                            pressed ? "Pressed, lamp on" : "Released, lamp off", model.statistics().lastRoundTripMs);
                std::fflush(stdout);
            }
        }
        if (!model.connected())
        {
            std::cerr << "Connection to the model TX lost\n";
        }

        model.stopAllOutputs();
        model.flush();
        const RelayConnection::Statistics& stats = model.statistics();
        std::printf("%u commands, %u resent, %u failed\n", stats.commands, stats.resends, stats.failures);
        model.disconnect();
        relay.update();
    }
    catch (const std::exception& e)
    {
        std::cerr << "Error: " << e.what() << '\n';
        return 1;
    }
    return 0;
}
