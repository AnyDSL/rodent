#include <memory>
#include <sstream>
#include <algorithm>

#include <SDL2/SDL.h>

#include "interface.h"
#include "float3.h"
#include "common.h"

// On x86, set flush-to-zero mode
#include <x86intrin.h>

static constexpr float pi = 3.14159265359f;

struct Camera {
    float3 eye;
    float3 dir;
    float3 right;
    float3 up;
    float w, h;

    Camera(const float3& e, const float3& d, const float3& u, float fov, float ratio) {
        eye = e;
        dir = normalize(d);
        right = normalize(cross(dir, u));
        up = normalize(cross(right, dir));

        w = std::tan(fov * pi / 360.0f);
        h = w / ratio;
    }

    void rotate(float yaw, float pitch) {
        dir = ::rotate(dir, right,  -pitch);
        dir = ::rotate(dir, up,     -yaw);
        dir = normalize(dir);
        right = normalize(cross(dir, up));
        up = normalize(cross(right, dir));
    }

    void move(float x, float y, float z) {
        eye += right * x + up * y + dir * z;
    }
};

void setup_interface(size_t, size_t);
float* get_pixels();
void clear_pixels();
void cleanup_interface();

static bool handle_events(uint32_t& iter, Camera& cam) {
    static bool camera_on = false;
    static bool arrows[4] = { false, false, false, false };
    static bool speed[2] = { false, false };
    const float rspeed = 0.005f;
    static float tspeed = 0.1f;

    SDL_Event event;
    while (SDL_PollEvent(&event)) {
        bool key_down = event.type == SDL_KEYDOWN;
        switch (event.type) {
            case SDL_KEYUP:
            case SDL_KEYDOWN:
                switch (event.key.keysym.sym) {
                    case SDLK_ESCAPE:   return true;
                    case SDLK_KP_PLUS:  speed[0] = key_down; break;
                    case SDLK_KP_MINUS: speed[1] = key_down; break;
                    case SDLK_UP:       arrows[0] = key_down; break;
                    case SDLK_DOWN:     arrows[1] = key_down; break;
                    case SDLK_LEFT:     arrows[2] = key_down; break;
                    case SDLK_RIGHT:    arrows[3] = key_down; break;
                }
                break;
            case SDL_MOUSEBUTTONDOWN:
                if (event.button.button == SDL_BUTTON_LEFT) {
                    SDL_SetRelativeMouseMode(SDL_TRUE);
                    camera_on = true;
                }
                break;
            case SDL_MOUSEBUTTONUP:
                if (event.button.button == SDL_BUTTON_LEFT) {
                    SDL_SetRelativeMouseMode(SDL_FALSE);
                    camera_on = false;
                }
                break;
            case SDL_MOUSEMOTION:
                if (camera_on) {
                    cam.rotate(event.motion.xrel * rspeed, event.motion.yrel * rspeed);
                    iter = 0;
                }
                break;
            case SDL_QUIT:
                return true;
            default:
                break;
        }
    }

    if (arrows[0]) cam.move(0, 0,  tspeed);
    if (arrows[1]) cam.move(0, 0, -tspeed);
    if (arrows[2]) cam.move(-tspeed, 0, 0);
    if (arrows[3]) cam.move( tspeed, 0, 0);
    if (arrows[0] | arrows[1] | arrows[2] | arrows[3]) iter = 0;
    if (speed[0]) tspeed *= 1.1f;
    if (speed[1]) tspeed *= 0.9f;
    return false;
}

static void update_texture(uint32_t* buf, SDL_Texture* texture, size_t width, size_t height, uint32_t iter) {
    auto film = get_pixels();
    auto inv_iter = 1.0f / iter;
    auto inv_gamma = 1.0f / 2.2f;
    for (size_t y = 0; y < height; ++y) {
        for (size_t x = 0; x < width; ++x) {
            auto r = film[(y * width + x) * 3 + 0];
            auto g = film[(y * width + x) * 3 + 1];
            auto b = film[(y * width + x) * 3 + 2];

            buf[y * width + x] =
                (uint32_t(clamp(pow(r * inv_iter, inv_gamma), 0.0f, 1.0f) * 255.0f) << 16) |
                (uint32_t(clamp(pow(g * inv_iter, inv_gamma), 0.0f, 1.0f) * 255.0f) << 8)  |
                 uint32_t(clamp(pow(b * inv_iter, inv_gamma), 0.0f, 1.0f) * 255.0f);
        }
    }
    SDL_UpdateTexture(texture, nullptr, buf, width * sizeof(uint32_t));
}

static inline void check_arg(int argc, char** argv, int arg, int n) {
    if (arg + n >= argc)
        error("Option '", argv[arg], "' expects ", n, " arguments, got ", argc - arg);
}

static inline void usage() {
    std::cout << "Usage: rodent [options]\n"
              << "Available options:\n"
              << "   --help            Shows this message\n"
              << "   --width  pixels   Sets the viewport horizontal dimension (in pixels)\n"
              << "   --height pixels   Sets the viewport vertical dimension (in pixels)\n"
              << "   --eye    x y z    Sets the position of the camera\n"
              << "   --dir    x y z    Sets the direction vector of the camera\n"
              << "   --up     x y z    Sets the up vector of the camera\n"
              << "   --fov    degrees  Sets the horizontal field of view (in degrees)\n"
              << "   --bench           Enables benchmarking mode" << std::endl;
}

int main(int argc, char** argv) {
    size_t bench_iter = 0;
    size_t width  = 1080;
    size_t height = 720;
    float fov = 60.0f;
    float3 eye(0.0f), dir(0.0f, 0.0f, 1.0f), up(0.0f, 1.0f, 0.0f);

    for (int i = 1; i < argc; ++i) {
        if (argv[i][0] == '-') {
            if (!strcmp(argv[i], "--width")) {
                check_arg(argc, argv, i, 1);
                width = strtoul(argv[++i], nullptr, 10);
            } else if (!strcmp(argv[i], "--height")) {
                check_arg(argc, argv, i, 1);
                height = strtoul(argv[++i], nullptr, 10);
            } else if (!strcmp(argv[i], "--eye")) {
                check_arg(argc, argv, i, 3);
                eye.x = strtof(argv[++i], nullptr);
                eye.y = strtof(argv[++i], nullptr);
                eye.z = strtof(argv[++i], nullptr);
            } else if (!strcmp(argv[i], "--dir")) {
                check_arg(argc, argv, i, 3);
                dir.x = strtof(argv[++i], nullptr);
                dir.y = strtof(argv[++i], nullptr);
                dir.z = strtof(argv[++i], nullptr);
            } else if (!strcmp(argv[i], "--up")) {
                check_arg(argc, argv, i, 3);
                up.x = strtof(argv[++i], nullptr);
                up.y = strtof(argv[++i], nullptr);
                up.z = strtof(argv[++i], nullptr);
            } else if (!strcmp(argv[i], "--fov")) {
                check_arg(argc, argv, i, 1);
                fov = strtof(argv[++i], nullptr);
            } else if (!strcmp(argv[i], "--bench")) {
                check_arg(argc, argv, i, 1);
                bench_iter = strtoul(argv[++i], nullptr, 10);
            } else if (!strcmp(argv[i], "--help")) {
                usage();
                return 0;
            } else {
                error("Unknown option '", argv[i], "'");
            }
            continue;
        }
        error("Unexpected argument '", argv[i], "'");
    }
    Camera cam(eye, dir, up, fov, (float)width / (float)height);

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

    setup_interface(width, height);

    // Force flush to zero mode for denormals
#if defined(__x86_64__) || defined(__amd64__) || defined(_M_X64)
    _mm_setcsr(_mm_getcsr() | (_MM_FLUSH_ZERO_ON | _MM_DENORMALS_ZERO_ON));
#endif

    auto spp = get_spp();
    bool done = false;
    uint64_t tick_counter = 0;
    uint32_t frames = 0;
    uint32_t iter = 0;
    std::vector<double> samples_sec;
    while (!done) {
        done = handle_events(iter, cam);

        if (iter == 0)
            clear_pixels();

        Settings settings {
            Vec3 { cam.eye.x, cam.eye.y, cam.eye.z },
            Vec3 { cam.dir.x, cam.dir.y, cam.dir.z },
            Vec3 { cam.up.x, cam.up.y, cam.up.z },
            Vec3 { cam.right.x, cam.right.y, cam.right.z },
            cam.w,
            cam.h
        };

        auto ticks = SDL_GetTicks();
        render(&settings, iter++);
        tick_counter += SDL_GetTicks() - ticks;

        frames++;
        if (frames > 10 || tick_counter >= 2500) {
            std::ostringstream os;
            auto frames_sec = double(frames) * 1000.0 / double(tick_counter);
            if (bench_iter != 0) {
                samples_sec.emplace_back(frames_sec * spp * width * height);
                if (samples_sec.size() == bench_iter)
                    break;
            }
            os << "Rodent [" << frames_sec << " FPS, "
               << iter * spp << " " << "sample" << (iter * spp > 1 ? "s" : "") << "]";
            SDL_SetWindowTitle(window, os.str().c_str());
            frames = 0;
            tick_counter = 0;
        }

        update_texture(buf.get(), texture, width, height, iter);
        SDL_RenderClear(renderer);
        SDL_RenderCopy(renderer, texture, nullptr, nullptr);
        SDL_RenderPresent(renderer);
    }

    cleanup_interface();

    SDL_DestroyTexture(texture);
    SDL_DestroyRenderer(renderer);
    SDL_DestroyWindow(window);
    SDL_Quit();

    if (bench_iter != 0) {
        auto inv = 1.0e-6;
        std::sort(samples_sec.begin(), samples_sec.end());
        std::cout << "# "
                  << samples_sec.front() * inv << "/"
                  << samples_sec[samples_sec.size() / 2] * inv << "/"
                  << samples_sec.back() * inv << " (min/med/max Msamples/s)" << std::endl;
    }
    return 0;
}
