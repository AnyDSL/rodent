#include <iostream>
#include <sstream>
#include <fstream>
#include <memory>
#include <string>
#include <unordered_map>
#include <atomic>
#include <chrono>

#include <embree3/rtcore.h>
#include <SDL2/SDL.h>
#include <tbb/tbb.h>

#include "float3.h"
#include "obj.h"
#include "image.h"
#include "common.h"

#include "embree_path_tracer.h"

extern "C" void render_tile(const struct scene_s* scene, counters_s* counters, uint32_t xmin, uint32_t ymin, uint32_t xmax, uint32_t ymax, uint32_t iter, uint32_t spp, uint32_t max_path_len);

static constexpr float pi = 3.14159265359f;

struct Camera {
    float3 eye;
    float3 dir;
    float3 right;
    float3 up;
    float w, h;

    Camera() {}

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

static void error_handler(void*, const RTCError code, const char* str) {
    if (code == RTC_ERROR_NONE)
        return;
    std::cerr << "Embree error ";
    switch (code) {
        case RTC_ERROR_UNKNOWN:             std::cerr << "RTC_ERROR_UNKNOWN"; break;
        case RTC_ERROR_INVALID_ARGUMENT:    std::cerr << "RTC_ERROR_INVALID_ARGUMENT"; break;
        case RTC_ERROR_INVALID_OPERATION:   std::cerr << "RTC_ERROR_INVALID_OPERATION"; break;
        case RTC_ERROR_OUT_OF_MEMORY:       std::cerr << "RTC_ERROR_OUT_OF_MEMORY"; break;
        case RTC_ERROR_UNSUPPORTED_CPU:     std::cerr << "RTC_ERROR_UNSUPPORTED_CPU"; break;
        case RTC_ERROR_CANCELLED:           std::cerr << "RTC_ERROR_CANCELLED"; break;
        default: break;
    }
    if (str) std::cerr << ": " << str;
    std::cerr << std::endl;
    abort();
}

static int32_t load_texture(std::vector<ImageRgba32>& rgba_images, std::vector<image_s>& images, std::unordered_map<std::string, int32_t>& tex_map, const std::string& file) {
    auto it = tex_map.find(file);
    if (it != tex_map.end())
        return it->second;
    ImageRgba32 img;
    if (!load_png(file, img) && !load_jpg(file, img)) {
        std::cerr << "Cannot load texture '" << file << "'" << std::endl;
        return -1;
    }
    images.emplace_back(image_s {
        (uint32_t*)img.pixels.get(),
        (uint32_t)img.width,
        (uint32_t)img.height
    });
    rgba_images.emplace_back(std::move(img));
    return tex_map[file] = images.size() - 1;
}

static bool load_scene(scene_s& ispc_scene,
                       Camera& cam,
                       std::vector<ImageRgba32>& rgba_images,
                       std::vector<image_s>& images,
                       std::vector<material_s>& materials,
                       std::vector<light_s>& lights,
                       const FilePath& path,
                       obj::TriMesh& tri_mesh,
                       obj::File& obj_file,
                       obj::MaterialLib& mtl_lib) {
    auto device = rtcNewDevice(nullptr);
    if (!device) {
        std::cerr << "Cannot initialize Embree" << std::endl;
        return false;
    }
    rtcSetDeviceErrorFunction(device, error_handler, nullptr);
    auto scene = rtcNewScene(device);
    auto geometry = rtcNewGeometry(device, RTC_GEOMETRY_TYPE_TRIANGLE);
    auto vertex_ptr = reinterpret_cast<float4*>(rtcSetNewGeometryBuffer(geometry, RTC_BUFFER_TYPE_VERTEX, 0, RTC_FORMAT_FLOAT3, sizeof(float4), tri_mesh.vertices.size()));
    for (auto& v : tri_mesh.vertices)
        *(vertex_ptr++) = float4(v.x, v.y, v.z, 1.0f);
    auto index_ptr = reinterpret_cast<uint32_t*>(rtcSetNewGeometryBuffer(geometry, RTC_BUFFER_TYPE_INDEX, 0,RTC_FORMAT_UINT3, sizeof(uint32_t) * 4, tri_mesh.indices.size() / 4));
    for (auto& i : tri_mesh.indices)
        *(index_ptr++) = i;
    rtcCommitGeometry(geometry);
    rtcAttachGeometry(scene, geometry);
    rtcCommitScene(scene);

    std::unordered_map<std::string, int32_t> tex_map;

    // Create materials
    for (size_t i = 0; i < obj_file.materials.size(); ++i) {
        auto& mtl_name = obj_file.materials[i];
        if (mtl_name == "") {
            materials.emplace_back(material_s {
                rgb_s { 0, 0, 0 },
                rgb_s { 0, 0, 0 },
                rgb_s { 0, 0, 0 },
                rgb_s { 0, 0, 0 },
                1.0f,
                1.0f,
                -1,
                -1,
                -1,
                2
            });
            continue;
        }
        auto it = mtl_lib.find(mtl_name);
        if (it == mtl_lib.end()) {
            std::cerr << "Cannot find material '" << mtl_name << "'" << std::endl;
            return false;
        }
        auto& mat = it->second;

        int32_t map_kd = -1, map_ks = -1;
        if (mat.map_kd != "")
            map_kd = load_texture(rgba_images, images, tex_map, path.base_name() + "/" + mat.map_kd);
        if (mat.map_ks != "")
            map_ks = load_texture(rgba_images, images, tex_map, path.base_name() + "/" + mat.map_ks);
        materials.emplace_back(material_s {
            rgb_s { mat.ke.x, mat.ke.y, mat.ke.z },
            rgb_s { mat.kd.x, mat.kd.y, mat.kd.z },
            rgb_s { mat.ks.x, mat.ks.y, mat.ks.z },
            rgb_s { mat.tf.x, mat.tf.y, mat.tf.z },
            mat.ns,
            mat.ni,
            map_kd,
            map_ks,
            mat.ke != float3(0.0f) ? 1 : 0,
            uint32_t(mat.illum)
        });
    }

    // Create lights
    for (size_t i = 0; i < tri_mesh.indices.size(); i += 4) {
        auto& mtl_name = obj_file.materials[tri_mesh.indices[i + 3]];
        if (mtl_name == "")
            continue;
        auto& mat = mtl_lib.find(mtl_name)->second;
        if (mat.ke == rgb(0.0f))
            continue;

        auto& v0 = tri_mesh.vertices[tri_mesh.indices[i + 0]];
        auto& v1 = tri_mesh.vertices[tri_mesh.indices[i + 1]];
        auto& v2 = tri_mesh.vertices[tri_mesh.indices[i + 2]];
        auto n = cross(v1 - v0, v2 - v0);
        auto inv_area = 1.0f / (0.5f * length(n));
        n *= 0.5f * inv_area;
        auto u = v1 - v0;
        auto v = v2 - v0;
        lights.emplace_back(light_s {
            float3_s { v0.x, v0.y, v0.z },
            float3_s { v1.x, v1.y, v1.z },
            float3_s { v2.x, v2.y, v2.z },
            float3_s { n.x, n.y, n.z },
            inv_area,
            rgb_s { mat.ke.x, mat.ke.y, mat.ke.z }
        });
    }
    if (lights.empty()) {
        std::cerr << "No lights in the scene!" << std::endl;
        return false;
    }

    ispc_scene.scene     = scene;
    ispc_scene.indices   = tri_mesh.indices.data();
    ispc_scene.vertices  = (float3_s*)tri_mesh.vertices.data();
    ispc_scene.normals   = (float3_s*)tri_mesh.normals.data();
    ispc_scene.texcoords = (float2_s*)tri_mesh.texcoords.data();
    ispc_scene.camera    = (camera_s*)&cam;
    ispc_scene.images    = images.data();
    ispc_scene.materials = materials.data();
    ispc_scene.lights    = lights.data();

    ispc_scene.num_lights    = lights.size();
    ispc_scene.pdf_lightpick = 1.0f / lights.size();
    return true;
}

static void update_texture(uint32_t* buf, float* film, SDL_Texture* texture, size_t width, size_t height, uint32_t iter) {
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
    std::cout << "Usage: embree_path_tracer [options] file.obj\n"
              << "Available options:\n"
              << "   --help                 Shows this message\n"
              << "   --width     pixels     Sets the viewport horizontal dimension (in pixels)\n"
              << "   --height    pixels     Sets the viewport vertical dimension (in pixels)\n"
              << "   --eye       x y z      Sets the position of the camera\n"
              << "   --dir       x y z      Sets the direction vector of the camera\n"
              << "   --up        x y z      Sets the up vector of the camera\n"
              << "   --fov       degrees    Sets the horizontal field of view (in degrees)\n"
              << "   --spp       spp        Sets the number of samples per pixel\n"
              << "   --max-depth depth      Sets the maximum path depth\n"
              << "   --bench     iterations Enables benchmarking mode and sets the number of iterations" << std::endl;
}

int main(int argc, char** argv) {
    if (argc < 2) {
        usage();
        return 1;
    }

    size_t bench_iter = 0;
    size_t width  = 1080;
    size_t height = 720;
    size_t max_path_depth = 64;
    size_t samples_per_pixel = 4;
    float fov = 60.0f;
    float3 eye(0.0f), dir(0.0f, 0.0f, 1.0f), up(0.0f, 1.0f, 0.0f);
    std::string file_name;

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
            } else if (!strcmp(argv[i], "--spp")) {
                check_arg(argc, argv, i, 1);
                samples_per_pixel = strtoul(argv[++i], nullptr, 10);
            } else if (!strcmp(argv[i], "--max-depth")) {
                check_arg(argc, argv, i, 1);
                max_path_depth = strtoul(argv[++i], nullptr, 10);
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
        if (file_name != "")
            error("Unexpected argument '", argv[i], "'");
        file_name = argv[i];
    }
    Camera cam(eye, dir, up, fov, (float)width / (float)height);

    obj::File obj_file;
    obj::MaterialLib mtl_lib;
    FilePath path(file_name);
    if (!obj::load_obj(path, obj_file)) {
        std::cerr << "Invalid OBJ file '" << path.file_name() << "'" << std::endl;
        return 1;
    }
    for (auto lib_name : obj_file.mtl_libs) {
        auto mtl_name = path.base_name() + "/" + lib_name;
        if (!obj::load_mtl(mtl_name, mtl_lib)) {
            std::cerr << "Invalid MTL file '" << mtl_name << "'" << std::endl;
            return 1;
        }
    }

    std::vector<ImageRgba32> rgba_images;
    scene_s scene;
    std::vector<image_s> images;
    std::vector<material_s> materials;
    std::vector<light_s> lights;

    auto tri_mesh = compute_tri_mesh(obj_file, mtl_lib, 0);
    if (!load_scene(scene, cam, rgba_images, images, materials, lights, path, tri_mesh, obj_file, mtl_lib)) {
        std::cerr << "Cannot load scene" << std::endl;
        return 1;
    }

    if (SDL_Init(SDL_INIT_VIDEO) < 0) {
        std::cerr << "Cannot initialize SDL" << std::endl;
        return 1;
    }

    auto window = SDL_CreateWindow(
        "cats [Embree]",
        SDL_WINDOWPOS_UNDEFINED,
        SDL_WINDOWPOS_UNDEFINED,
        width,
        height,
        0);
    if (!window) {
        std::cerr << "Cannot create window." << std::endl;
        return 1;
    }

    auto renderer = SDL_CreateRenderer(window, -1, 0);
    if (!renderer) {
        std::cerr << "Cannot create renderer." << std::endl;
        return 1;
    }

    auto texture = SDL_CreateTexture(renderer, SDL_PIXELFORMAT_ARGB8888, SDL_TEXTUREACCESS_STATIC, width, height);
    if (!texture) {
        std::cerr << "Cannot create texture" << std::endl;
        return 1;
    }
    std::unique_ptr<uint32_t> buf(new uint32_t[width * height]);

    bool done = false;
    uint64_t timing = 0;
    uint32_t frames = 0;
    uint32_t iter = 0;
    std::vector<double> samples_sec;

    std::unique_ptr<float3[]> film(new float3[width * height]);
    scene.film = (float*)film.get();
    scene.film_width  = width;
    scene.film_height = height;

    while (!done) {
        done = handle_events(iter, cam);

        if (iter == 0)
            memset(film.get(), 0, sizeof(float3) * width * height);

        auto ticks = std::chrono::high_resolution_clock::now();
#if defined(ENABLE_TIMING) && defined(FORCE_SERIAL_TIMING)
        counters_s counters = { 0, 0, 0, 0, 0 };
        for (size_t y = 0, tile_height = 4; y < height; y += tile_height) {
            auto ymax = y + tile_height < height ? y + tile_height : height;
            for (size_t x = 0, tile_width = 4; x < width; x += tile_width) {
                auto xmax = x + tile_width < width ? x + tile_width : width;
                render_tile(
                    &scene,
                    &counters,
                    x,
                    y,
                    xmax,
                    ymax,
                    iter,
                    samples_per_pixel,
                    max_path_depth
                );
            }
        }
        int64_t primary_time = counters.primary;
        int64_t bounces_time = counters.bounces;
        int64_t shadow_time  = counters.shadow;
        int64_t shade_time   = counters.shade;
        int64_t total_time   = counters.total;
        int64_t total_rays   = counters.total_rays;
#else
#ifdef ENABLE_TIMING
        std::atomic<int64_t> primary_time(0);
        std::atomic<int64_t> bounces_time(0);
        std::atomic<int64_t> shadow_time (0);
        std::atomic<int64_t> shade_time  (0);
        std::atomic<int64_t> total_time  (0);
        std::atomic<int64_t> total_rays  (0);
#endif
        tbb::parallel_for(tbb::blocked_range2d<uint32_t>(0, height, 0, width),
            [&] (const tbb::blocked_range2d<uint32_t>& range) {
                counters_s counters = { 0, 0, 0, 0, 0 };
                render_tile(
                    &scene,
                    &counters,
                    range.cols().begin(),
                    range.rows().begin(),
                    range.cols().end(),
                    range.rows().end(),
                    iter,
                    samples_per_pixel,
                    max_path_depth
                );
#ifdef ENABLE_TIMING
                primary_time += counters.primary;
                bounces_time += counters.bounces;
                shadow_time  += counters.shadow;
                shade_time   += counters.shade;
                total_time   += counters.total;
                total_rays   += counters.total_rays;
#endif
            });
#endif
#ifdef ENABLE_TIMING
        if (total_time > 0) {
            auto others_time = total_time - primary_time - bounces_time - shadow_time - shade_time;
            std::cout << "primary: " << primary_time << " (" << primary_time * 100 / total_time << "%)\n"
                      << "bounces: " << bounces_time << " (" << bounces_time * 100 / total_time << "%)\n"
                      << "shadow: "  << shadow_time  << " (" << shadow_time  * 100 / total_time << "%)\n"
                      << "shade: "   << shade_time   << " (" << shade_time   * 100 / total_time << "%)\n"
                      << "others: "  << others_time  << " (" << others_time  * 100 / total_time << "%)\n"
                      << "total: "   << total_time   << " (100%)\n"
                      << "total rays: " << total_rays << std::endl;
        }
#endif
        auto elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now() - ticks).count();
        iter++;

        if (bench_iter != 0) {
            samples_sec.emplace_back(1000.0 * double(samples_per_pixel * width * height) / double(elapsed_ms));
            if (samples_sec.size() == bench_iter)
                break;
        }

        frames++;
        timing += elapsed_ms;
        if (frames > 10 || timing >= 2500) {
            auto frames_sec = double(frames) * 1000.0 / double(timing);
            std::ostringstream os;
            os << "cats [Embree: " << frames_sec << " FPS, "
               << iter * samples_per_pixel << " " << "sample" << (iter * samples_per_pixel > 1 ? "s" : "") << "]";
            SDL_SetWindowTitle(window, os.str().c_str());
            frames = 0;
            timing = 0;
        }

        update_texture(buf.get(), (float*)film.get(), texture, width, height, iter);

        SDL_RenderClear(renderer);
        SDL_RenderCopy(renderer, texture, nullptr, nullptr);
        SDL_RenderPresent(renderer);
    }

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
