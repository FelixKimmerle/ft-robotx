// Shows a text on a controller's display; without text, clears it.
// Usage: ftrobotx_message <port> <controller> [text]
//   controller: 0 = master, 1 ... 8 = extension 1 ... 8
#include <exception>
#include <iostream>
#include <string>

#include "ftrobotx/FtConnection.hpp"

int main(int argc, char* argv[])
{
    using namespace ftrobotx;

    if (argc < 3)
    {
        std::cerr << "Usage: " << argv[0] << " <port> <controller> [text]\n"
                  << "  controller: 0 = master, 1 ... 8 = extension 1 ... 8\n"
                  << "  without text, the message on the display is cleared\n";
        return 2;
    }

    try
    {
        int controller = std::stoi(argv[2]);
        if (controller < 0 || controller > 8)
        {
            std::cerr << "controller must be 0 ... 8\n";
            return 2;
        }

        std::string text;
        for (int i = 3; i < argc; ++i)
        {
            text += (i > 3 ? " " : "") + std::string(argv[i]);
        }

        FtConnection ft(argv[1], static_cast<std::size_t>(controller) + 1);
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
