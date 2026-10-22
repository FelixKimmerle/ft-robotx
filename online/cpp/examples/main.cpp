// Same demo as online/cs/src/Program.cs. Usage: ftrobotx_demo [-p PORT]; --help explains the options.
#include <csignal>
#include <exception>
#include <iostream>

#include "Cli.hpp"
#include "ftrobotx/FtConnection.hpp"

#define DESCRIPTION                                                                                                    \
    "The original demo: master and extension 1; a motor on M3 of the extension (M1 synchronized to it), and the\n"     \
    "button on I8 of the master starts a distance run of M1. Lists the ports found. Ctrl+C quits."

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
        for (const SerialPort::PortInfo& port : SerialPort::listPorts())
        {
            std::cout << port.name << "  " << port.description << '\n';
        }

        std::string portName = cli::choosePort(portOption);
        if (portName.empty())
        {
            return 1;
        }

        std::signal(SIGINT, onSignal);

        FtConnection ft(portName, 2);
        std::cout << "Config success\n";

        for (const Extension& extension : ft.extensions())
        {
            std::cout << extension.name << '\n'
                      << (extension.version ? extension.version->toString() : "") << '\n'
                      << extension.macAddress << '\n';
        }

        // ft.setMotorSync(ExtensionIndex::Slave1, Motor::M1, Motor::M3, 512);
        // ft.setMotorDistance(ExtensionIndex::Slave1, Motor::M1, -512, 200);

        ft.extension(ExtensionIndex::Slave1).output.motorMasterValues[static_cast<std::size_t>(Motor::M1)] =
            static_cast<std::uint8_t>(static_cast<int>(Motor::M3) + 1);
        ft.setMotorOutput(ExtensionIndex::Slave1, Motor::M3, 512);

        try
        {
            while (stopRequested == 0)
            {
                ft.update();

                for (Extension& extension : ft.extensions())
                {
                    if (extension.input.changed)
                    {
                        std::cout << extension.input.motorReachedDestination(static_cast<std::size_t>(Motor::M1))
                                  << '\n';
                        extension.input.changed = false;
                    }
                }

                if (ft.digitalValue(ExtensionIndex::Local, UniversalInput::I8))
                {
                    ft.setMotorDistance(ExtensionIndex::Slave1, Motor::M1, -512, 200);
                }
            }
        }
        catch (...)
        {
            // Don't leave the motors running.
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
