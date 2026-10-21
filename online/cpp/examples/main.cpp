// Same demo as cs/src/Program.cs. Usage: ftrobotx_demo [port]
#include <csignal>
#include <exception>
#include <iostream>

#include "ftrobotx/FtConnection.hpp"

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

    try
    {
        std::vector<SerialPort::PortInfo> ports = SerialPort::listPorts();
        for (const SerialPort::PortInfo& port : ports)
        {
            std::cout << port.name << "  " << port.description << '\n';
        }

        std::string portName;
        if (argc > 1)
        {
            portName = argv[1];
        }
        else if (!ports.empty())
        {
            portName = ports.front().name;
        }
        else
        {
            std::cerr << "No serial port found.\n";
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
