#include <memory>
#include <sstream>
#include <SDL2/SDL.h>

#include "interface.h"
#include "common.h"

void setup_cpu_interface(size_t, size_t);
Color* get_cpu_pixels();
void cleanup_cpu_interface();

static bool handle_events() {
    SDL_Event event;
    while (SDL_PollEvent(&event)) {
        switch (event.type) {
            case SDL_KEYDOWN:
                if (event.key.keysym.sym == SDLK_ESCAPE)
                    return true;
                break;
            case SDL_QUIT:
                return true;
            default:
                break;
        }
    }
    return false;
}

int main(int argc, char** argv) {
    size_t width  = 1024;
    size_t height = 1024;

    if (SDL_Init(SDL_INIT_VIDEO) != 0)
        error("Cannot initialize SDL.");

    auto window = SDL_CreateWindow(
        "Rodent",
        SDL_WINDOWPOS_UNDEFINED,
        SDL_WINDOWPOS_UNDEFINED,
        width,
        height,
        0);
    if (!window)
        error("Cannot create window.");

    auto renderer = SDL_CreateRenderer(window, -1, 0);
    if (!renderer)
        error("Cannot create renderer.");

    auto texture = SDL_CreateTexture(renderer, SDL_PIXELFORMAT_ARGB8888, SDL_TEXTUREACCESS_STATIC, width, height);
    if (!texture)
        error("Cannot create texture");

    std::unique_ptr<uint32_t> buf(new uint32_t[width * height]);

    setup_cpu_interface(width, height);

    bool done = false;
    uint64_t tick_counter = 0;
    uint32_t frames = 0;
    uint32_t iter = 0;
    while (!done) {
        done = handle_events();

        auto ticks = SDL_GetTicks();
        render(iter++);
        tick_counter += SDL_GetTicks() - ticks;
        frames++;
        if (frames > 10 || tick_counter >= 5000) {
            std::ostringstream os;
            os << "Rodent [" << double(frames) * 1000.0 / double(tick_counter) << " FPS, " << iter << " samples]";
            SDL_SetWindowTitle(window, os.str().c_str());
            frames = 0;
            tick_counter = 0;
        }

        auto film = get_cpu_pixels();
        auto inv_iter = 1.0f / iter;
        for (size_t y = 0; y < height; ++y) {
            for (size_t x = 0; x < width; ++x) {
                auto pixel = film[y * width + x];
                buf.get()[y * width + x] =
                    (uint32_t(clamp(pixel.r * inv_iter, 0.0f, 1.0f) * 255.0f) << 16) |
                    (uint32_t(clamp(pixel.g * inv_iter, 0.0f, 1.0f) * 255.0f) << 8)  |
                     uint32_t(clamp(pixel.b * inv_iter, 0.0f, 1.0f) * 255.0f);
            }
        }
        SDL_UpdateTexture(texture, nullptr, buf.get(), width * sizeof(uint32_t));
        SDL_RenderClear(renderer);
        SDL_RenderCopy(renderer, texture, nullptr, nullptr);
        SDL_RenderPresent(renderer);
    }

    cleanup_cpu_interface();

    SDL_DestroyTexture(texture);
    SDL_DestroyRenderer(renderer);
    SDL_DestroyWindow(window);
    SDL_Quit();
    return 0;
}
