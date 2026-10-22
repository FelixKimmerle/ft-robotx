// Shows a text on a controller's display; without text, clears it.
// Usage: ftrobotx_message [-p PORT] [-c N] [TEXT...]; --help explains the options.
#include <exception>
#include <iostream>
#include <string>
#include <vector>

#include "Cli.hpp"
#include "ftrobotx/FtConnection.hpp"

int main(int argc, char* argv[])
{
    using namespace ftrobotx;

    std::string portOption;
    std::string controllerOption = "0";
    std::vector<std::string> words;
    int exitCode = 0;
    if (!cli::parse(argc, argv, "[-p PORT] [-c N] [TEXT...]",
                    "Shows TEXT (up to 98 characters) on a controller's display. Without TEXT, the message on the\n"
                    "display is cleared.",
                    {{"-p", "--port", "PORT", cli::kPortHelp, &portOption},
                     {"-c", "--controller", "N", "Display to use: 0 = master (default), 1 ... 8 = extension 1 ... 8.",
                      &controllerOption}},
                    &words, exitCode))
    {
        return exitCode;
    }

    int controller = -1;
    try
    {
        controller = std::stoi(controllerOption);
    }
    catch (const std::exception&)
    {
    }
    if (controller < 0 || controller > 8)
    {
        std::cerr << argv[0] << ": N must be 0 ... 8; see --help\n";
        return 2;
    }

    std::string text;
    for (const std::string& word : words)
    {
        text += (text.empty() ? "" : " ") + word;
    }

    try
    {
        std::string portName = cli::choosePort(portOption);
        if (portName.empty())
        {
            return 1;
        }
        FtConnection ft(portName, static_cast<std::size_t>(controller) + 1);
        ft.showMessage(static_cast<ExtensionIndex>(controller), text);
        std::cout << (text.empty() ? "Message cleared" : "Message shown") << '\n';
    }
    catch (const std::exception& e)
    {
        std::cerr << "Error: " << e.what() << '\n';
        return 1;
    }
    return 0;
}
