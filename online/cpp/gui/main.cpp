// ft-robotx test tool: shows and controls all inputs, outputs and counters of a master and its extensions.
// Usage: ftrobotx_gui [port [extensions]]; with a port given, it connects right away.
#include <cstdio>
#include <string>

#include <SDL3/SDL.h>
#include <SDL3/SDL_main.h>
#include <imgui.h>
#include <imgui_impl_sdl3.h>
#include <imgui_impl_sdlrenderer3.h>

#include "App.hpp"

int main(int argc, char* argv[])
{
    if (!SDL_Init(SDL_INIT_VIDEO))
    {
        std::fprintf(stderr, "SDL_Init failed: %s\n", SDL_GetError());
        return 1;
    }

    float scale = SDL_GetDisplayContentScale(SDL_GetPrimaryDisplay());
    if (scale <= 0)
    {
        scale = 1;
    }
    SDL_Window* window =
        SDL_CreateWindow("ft-robotx", static_cast<int>(1100 * scale), static_cast<int>(640 * scale),
                         SDL_WINDOW_RESIZABLE | SDL_WINDOW_HIDDEN | SDL_WINDOW_HIGH_PIXEL_DENSITY);
    if (window == nullptr)
    {
        std::fprintf(stderr, "SDL_CreateWindow failed: %s\n", SDL_GetError());
        return 1;
    }
    SDL_Renderer* renderer = SDL_CreateRenderer(window, nullptr);
    if (renderer == nullptr)
    {
        std::fprintf(stderr, "SDL_CreateRenderer failed: %s\n", SDL_GetError());
        return 1;
    }
    SDL_SetRenderVSync(renderer, 1);
    SDL_SetWindowPosition(window, SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED);
    SDL_ShowWindow(window);

    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGuiIO& io = ImGui::GetIO();
    io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard | ImGuiConfigFlags_DockingEnable;

    // Remember the window layout per user, e.g. ~/.local/share/ft-robotx/gui/imgui.ini
    std::string iniPath;
    if (char* prefPath = SDL_GetPrefPath("ft-robotx", "gui"))
    {
        iniPath = std::string(prefPath) + "imgui.ini";
        SDL_free(prefPath);
    }
    bool testRun = SDL_getenv("FT_GUI_SCREENSHOT") != nullptr; // never touch the user's layout from tests
    io.IniFilename = iniPath.empty() || testRun ? nullptr : iniPath.c_str();

    ImGui::StyleColorsDark();
    ImGuiStyle& style = ImGui::GetStyle();
    style.FrameRounding = 3;
    style.GrabRounding = 3;
    style.ScaleAllSizes(scale);
    style.FontScaleDpi = scale;

    ImGui_ImplSDL3_InitForSDLRenderer(window, renderer);
    ImGui_ImplSDLRenderer3_Init(renderer);

    // For testing: FT_GUI_SCREENSHOT=file.bmp saves the window after a few seconds and quits.
    const char* screenshotPath = SDL_getenv("FT_GUI_SCREENSHOT");
    const Uint64 screenshotAt = SDL_GetTicks() + 3000;

    {
        App app(argc > 1 ? argv[1] : "", argc > 2 ? SDL_atoi(argv[2]) : 1);

        bool done = false;
        while (!done)
        {
            SDL_Event event;
            while (SDL_PollEvent(&event))
            {
                ImGui_ImplSDL3_ProcessEvent(&event);
                if (event.type == SDL_EVENT_QUIT || (event.type == SDL_EVENT_WINDOW_CLOSE_REQUESTED &&
                                                     event.window.windowID == SDL_GetWindowID(window)))
                {
                    done = true;
                }
            }
            if ((SDL_GetWindowFlags(window) & SDL_WINDOW_MINIMIZED) != 0)
            {
                SDL_Delay(10);
                continue;
            }

            ImGui_ImplSDLRenderer3_NewFrame();
            ImGui_ImplSDL3_NewFrame();
            ImGui::NewFrame();
            app.draw();
            ImGui::Render();

            SDL_SetRenderScale(renderer, io.DisplayFramebufferScale.x, io.DisplayFramebufferScale.y);
            SDL_SetRenderDrawColorFloat(renderer, 0.1f, 0.1f, 0.12f, 1.0f);
            SDL_RenderClear(renderer);
            ImGui_ImplSDLRenderer3_RenderDrawData(ImGui::GetDrawData(), renderer);

            if (screenshotPath != nullptr && SDL_GetTicks() >= screenshotAt)
            {
                if (SDL_Surface* surface = SDL_RenderReadPixels(renderer, nullptr))
                {
                    SDL_SaveBMP(surface, screenshotPath);
                    SDL_DestroySurface(surface);
                }
                done = true;
            }

            SDL_RenderPresent(renderer);
        }
    } // App closes the connection and stops all outputs here.

    ImGui_ImplSDLRenderer3_Shutdown();
    ImGui_ImplSDL3_Shutdown();
    ImGui::DestroyContext();
    SDL_DestroyRenderer(renderer);
    SDL_DestroyWindow(window);
    SDL_Quit();
    return 0;
}
