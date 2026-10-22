// Command-line handling shared by the example programs and ftrobotx_gui: -h/--help, -p/--port and a few more
// options, all in the usual "--help" style.
#pragma once

#include <cstdlib>
#include <iostream>
#include <string>
#include <string_view>
#include <vector>

#include "ftrobotx/SerialPort.hpp"

namespace cli
{

struct Option
{
    const char* shortName; // e.g. "-p"
    const char* longName;  // e.g. "--port"
    const char* valueName; // e.g. "PORT", or nullptr for a flag without value
    const char* help;
    std::string* value; // receives the value, or "1" for a flag
};

inline constexpr const char* kPortHelp =
    "Port of the ROBO TX Controller: /dev/ttyACM0 (USB), /dev/rfcomm0 (Bluetooth via rfcomm),\n"
    "                        bt:00:13:7B:53:DB:5E (Bluetooth directly, Linux) or COM3 (Windows).\n"
    "                        Default: the first ROBO TX Controller found on USB.";

/// Parses argv. Prints the help for -h/--help (exit code 0) or an error (exit code 2) and returns false then;
/// `exitCode` says which. Arguments that are no option end up in `positional`.
inline bool parse(int argc, char* argv[], std::string_view usage, std::string_view description,
                  const std::vector<Option>& options, std::vector<std::string>* positional, int& exitCode)
{
    auto printHelp = [&] {
        std::cout << "Usage: " << argv[0] << ' ' << usage << "\n\n" << description << "\n\nOptions:\n";
        std::cout << "  -h, --help            Show this help and exit.\n";
        for (const Option& option : options)
        {
            std::string flags = std::string(option.shortName) + ", " + option.longName;
            if (option.valueName != nullptr)
            {
                flags += std::string(" ") + option.valueName;
            }
            // help text in a column of its own; on the next line if the option is too long
            std::cout << "  " << flags
                      << (flags.size() < 22 ? std::string(22 - flags.size(), ' ') : "\n" + std::string(24, ' '))
                      << option.help << '\n';
        }
    };

    for (int i = 1; i < argc; ++i)
    {
        std::string_view arg = argv[i];
        if (arg == "-h" || arg == "--help")
        {
            printHelp();
            exitCode = 0;
            return false;
        }
        bool matched = false;
        for (const Option& option : options)
        {
            if (arg != option.shortName && arg != option.longName)
            {
                continue;
            }
            matched = true;
            if (option.valueName == nullptr)
            {
                *option.value = "1";
            }
            else if (i + 1 < argc)
            {
                *option.value = argv[++i];
            }
            else
            {
                std::cerr << argv[0] << ": " << arg << " needs a value (" << option.valueName << "); see --help\n";
                exitCode = 2;
                return false;
            }
        }
        if (!matched)
        {
            if (arg.starts_with("-") && arg.size() > 1)
            {
                std::cerr << argv[0] << ": unknown option " << arg << "; see --help\n";
                exitCode = 2;
                return false;
            }
            if (positional == nullptr)
            {
                std::cerr << argv[0] << ": unexpected argument " << arg << "; see --help\n";
                exitCode = 2;
                return false;
            }
            positional->emplace_back(arg);
        }
    }
    return true;
}

/// The port given with -p, or else the first ROBO TX Controller on USB. Empty if there is none.
inline std::string choosePort(const std::string& port)
{
    if (!port.empty())
    {
        return port;
    }
    for (const ftrobotx::SerialPort::PortInfo& info : ftrobotx::SerialPort::listPorts())
    {
        if (info.description == "ROBO TX Controller (USB)")
        {
            std::cout << "Using " << info.name << " (" << info.description << ")\n";
            return info.name;
        }
    }
    std::cerr << "No ROBO TX Controller found on USB; choose the port with -p (see --help)\n";
    return {};
}

} // namespace cli
