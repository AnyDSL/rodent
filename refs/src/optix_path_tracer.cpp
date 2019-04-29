#include <iostream>
#include <sstream>
#include <fstream>
#include <memory>
#include <string>
#include <unordered_map>
#include <chrono>

#include <optixu/optixpp_namespace.h>
#include <optixu/optixu_math_stream_namespace.h>

#include <SDL2/SDL.h>

#include "obj.h"
#include "image.h"
#include "file_path.h"
#include "optix_path_tracer.h"

#ifndef GENERATED_PTX_FILE
#error "Missing definition for GENERATED_PTX_FILE macro"
#endif

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

void setup_renderer(optix::Context& context, optix::Buffer& frame_buffer, size_t max_path_depth, size_t samples_per_pixel, size_t width, size_t height) {
    context["film_width"]       ->setUint(width);
    context["film_height"]      ->setUint(height);
    context["max_path_depth"]   ->setUint(max_path_depth);
    context["samples_per_pixel"]->setUint(samples_per_pixel);
    context["frame_buffer"]     ->setBuffer(frame_buffer);
}

void setup_camera(optix::Context& context, const Camera& cam) {
    context["cam_eye"]  ->setFloat(optix::make_float3(cam.eye.x, cam.eye.y, cam.eye.z));
    context["cam_dir"]  ->setFloat(optix::make_float3(cam.dir.x, cam.dir.y, cam.dir.z));
    context["cam_right"]->setFloat(optix::make_float3(cam.right.x, cam.right.y, cam.right.z));
    context["cam_up"]   ->setFloat(optix::make_float3(cam.up.x, cam.up.y, cam.up.z));
    context["cam_dim"]  ->setFloat(optix::make_float2(cam.w, cam.h));
}

void set_frame_number(optix::Context& context, size_t frame) {
    context["frame_number"]->setUint(frame);
}

int load_texture(optix::Context& context, std::unordered_map<std::string, int>& tex_map, const std::string& name) {
    auto it = tex_map.find(name);
    if (it != tex_map.end())
        return it->second;
    ImageRgba32 img;
    if (!load_png(name, img) && !load_jpg(name, img)) {
        std::cerr << "Cannot load texture '" << name << "'" << std::endl;
        return -1;
    }
    auto tex_buffer = context->createBuffer(RT_BUFFER_INPUT);
    tex_buffer->setFormat(RT_FORMAT_UNSIGNED_BYTE4);
    tex_buffer->setSize(img.width, img.height);
    memcpy(tex_buffer->map(), img.pixels.get(), sizeof(uint32_t) * img.width * img.height);
    tex_buffer->unmap();
    auto sampler = context->createTextureSampler();
    sampler->setFilteringModes(RT_FILTER_LINEAR, RT_FILTER_LINEAR, RT_FILTER_NONE);
    sampler->setWrapMode(0, RT_WRAP_REPEAT);
    sampler->setWrapMode(1, RT_WRAP_REPEAT);
    sampler->setIndexingMode(RT_TEXTURE_INDEX_NORMALIZED_COORDINATES);
    sampler->setReadMode(RT_TEXTURE_READ_NORMALIZED_FLOAT);
    sampler->setMaxAnisotropy(1.0f);
    sampler->setBuffer(0, 0, tex_buffer);
    return tex_map[name] = sampler->getId();
}

optix::Context load_scene(const FilePath& path, obj::TriMesh& tri_mesh, obj::File& obj_file, obj::MaterialLib& mtl_lib) {
    auto context = optix::Context::create();
    std::ifstream ptx_stream(GENERATED_PTX_FILE);
    if (!ptx_stream) {
        std::cerr << "Missing PTX file \'" << GENERATED_PTX_FILE << "\'" << std::endl;
        return nullptr;
    }

    auto ptx = std::string(std::istreambuf_iterator<char>(ptx_stream), std::istreambuf_iterator<char>());

    context->setRayTypeCount(2);
    context->setEntryPointCount(1);
    context->setStackSize(1800);

    // Load OptiX programs
    auto obj_material   = context->createMaterial();
    auto exception      = context->createProgramFromPTXString(ptx.c_str(), "exception");
    auto miss           = context->createProgramFromPTXString(ptx.c_str(), "miss");
    auto closest_hit    = context->createProgramFromPTXString(ptx.c_str(), "closest_hit");
    auto shadow         = context->createProgramFromPTXString(ptx.c_str(), "shadow");
    auto path_trace     = context->createProgramFromPTXString(ptx.c_str(), "path_trace");
    auto mesh_intersect = context->createProgramFromPTXString(ptx.c_str(), "mesh_intersect");
    auto mesh_bounds    = context->createProgramFromPTXString(ptx.c_str(), "mesh_bounds");
    obj_material->setClosestHitProgram(0, closest_hit);
    obj_material->setAnyHitProgram(1, shadow);

    context->setRayGenerationProgram(0, path_trace);
    context->setExceptionProgram(0, exception);
    context->setMissProgram(0, miss);

    // Create geometry
    auto geometry = context->createGeometry();

    auto indices = context->createBuffer(RT_BUFFER_INPUT);
    indices->setFormat(RT_FORMAT_UNSIGNED_INT4);
    indices->setSize(tri_mesh.indices.size() / 4);
    memcpy(indices->map(), tri_mesh.indices.data(), sizeof(uint32_t) * tri_mesh.indices.size());
    indices->unmap();
    context["index_buffer"]->setBuffer(indices);

    auto vertices = context->createBuffer(RT_BUFFER_INPUT);
    vertices->setFormat(RT_FORMAT_FLOAT3);
    vertices->setSize(tri_mesh.vertices.size());
    memcpy(vertices->map(), tri_mesh.vertices.data(), sizeof(float3) * tri_mesh.vertices.size());
    vertices->unmap();
    context["vertex_buffer"]->setBuffer(vertices);

    auto normals = context->createBuffer(RT_BUFFER_INPUT);
    normals->setFormat(RT_FORMAT_FLOAT3);
    normals->setSize(tri_mesh.normals.size());
    memcpy(normals->map(), tri_mesh.normals.data(), sizeof(float3) * tri_mesh.normals.size());
    normals->unmap();
    context["normal_buffer"]->setBuffer(normals);

    auto texcoords = context->createBuffer(RT_BUFFER_INPUT);
    texcoords->setFormat(RT_FORMAT_FLOAT2);
    texcoords->setSize(tri_mesh.texcoords.size());
    memcpy(texcoords->map(), tri_mesh.texcoords.data(), sizeof(float2) * tri_mesh.texcoords.size());
    texcoords->unmap();
    context["texcoord_buffer"]->setBuffer(texcoords);

    geometry->setPrimitiveCount(tri_mesh.indices.size() / 4);
    geometry->setIntersectionProgram(mesh_intersect);
    geometry->setBoundingBoxProgram(mesh_bounds);

    auto accel = context->createAcceleration("Sbvh");
    accel->setProperty("vertex_buffer_name", "vertex_buffer");
    accel->setProperty("index_buffer_name",  "index_buffer");
    accel->setProperty("index_buffer_stride",  std::to_string(sizeof(uint32_t) * 4));
    accel->setProperty("vertex_buffer_Stride", std::to_string(sizeof(float) * 3));

    auto instance = context->createGeometryInstance();
    instance->setGeometry(geometry);
    instance->setMaterialCount(1);
    instance->setMaterial(0, obj_material);

    auto geometry_group = context->createGeometryGroup();
    geometry_group->addChild(instance);
    geometry_group->setAcceleration(accel);
    context["top_object"]->set(geometry_group);

    // Create lights
    std::vector<Light> lights;
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
        lights.emplace_back(Light {
            v0, v1, v2, n, inv_area, mat.ke
        });
    }
    if (lights.empty()) {
        std::cerr << "No lights in the scene!" << std::endl;
        return nullptr;
    }

    context["pdf_lightpick"]->setFloat(1.0f / lights.size());
    context["num_lights"]   ->setUint(lights.size());

    auto light_buffer = context->createBuffer(RT_BUFFER_INPUT);
    light_buffer->setFormat(RT_FORMAT_USER);
    light_buffer->setElementSize(sizeof(Light));
    light_buffer->setSize(lights.size());
    memcpy(light_buffer->map(), lights.data(), sizeof(Light) * lights.size());
    light_buffer->unmap();

    context["lights"]->setBuffer(light_buffer);

    // Create materials
    std::vector<Material> materials;
    std::unordered_map<std::string, int> tex_map;
    for (size_t i = 0; i < obj_file.materials.size(); ++i) {
        auto& mtl_name = obj_file.materials[i];
        if (mtl_name == "") {
            materials.emplace_back(Material {
                float3(0.0f),
                float3(0.0f),
                float3(0.0f),
                float3(0.0f),
                -1,
                -1,
                1.0f,
                1.0f,
                2
            });
            continue;
        }
        auto it = mtl_lib.find(mtl_name);
        if (it == mtl_lib.end()) {
            std::cerr << "Cannot find material '" << mtl_name << "'" << std::endl;
            return nullptr;
        }
        auto& mat = it->second;

        int32_t map_kd = -1, map_ks = -1;
        if (mat.map_kd != "")
            map_kd = load_texture(context, tex_map, path.base_name() + "/" + mat.map_kd);
        if (mat.map_ks != "")
            map_ks = load_texture(context, tex_map, path.base_name() + "/" + mat.map_ks);
        materials.emplace_back(Material {
            mat.kd,
            mat.ks,
            mat.ke,
            mat.tf,
            map_kd,
            map_ks,
            mat.ns,
            mat.ni,
            uint32_t(mat.illum)
        });
    }

    auto mat_buffer = context->createBuffer(RT_BUFFER_INPUT);
    mat_buffer->setFormat(RT_FORMAT_USER);
    mat_buffer->setElementSize(sizeof(Material));
    mat_buffer->setSize(materials.size());
    memcpy(mat_buffer->map(), materials.data(), sizeof(Material) * materials.size());
    mat_buffer->unmap();

    context["materials"]->setBuffer(mat_buffer);

    return context;
}

optix::Buffer create_frame_buffer(optix::Context& context, size_t width, size_t height) {
    auto frame_buffer = context->createBuffer(RT_BUFFER_INPUT_OUTPUT);
    frame_buffer->setFormat(RT_FORMAT_FLOAT3);
    frame_buffer->setSize(width * height);
    return frame_buffer;
}

static inline void check_arg(int argc, char** argv, int arg, int n) {
    if (arg + n >= argc)
        error("Option '", argv[arg], "' expects ", n, " arguments, got ", argc - arg);
}

static inline void usage() {
    std::cout << "Usage: optix_path_tracer [options] file.obj\n"
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

    auto tri_mesh = compute_tri_mesh(obj_file, mtl_lib, 0);
    auto context = load_scene(path, tri_mesh, obj_file, mtl_lib);
    if (!context) {
        std::cerr << "Cannot create OptiX context" << std::endl;
        return 1;
    }

    auto frame_buffer = create_frame_buffer(context, width, height);
    setup_renderer(context, frame_buffer, max_path_depth, samples_per_pixel, width, height);

    if (SDL_Init(SDL_INIT_VIDEO) < 0) {
        std::cerr << "Cannot initialize SDL" << std::endl;
        return 1;
    }

    auto window = SDL_CreateWindow(
        "cats [OptiX]",
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
    while (!done) {
        done = handle_events(iter, cam);

        if (iter == 0) {
            memset(frame_buffer->map(), 0, sizeof(float3) * width * height);
            frame_buffer->unmap();
            setup_camera(context, cam);
        }
        set_frame_number(context, iter);

        auto ticks = std::chrono::high_resolution_clock::now();
        context->launch(0, width, height);
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
            os << "cats [OptiX: " << frames_sec << " FPS, "
               << iter * samples_per_pixel << " " << "sample" << (iter * samples_per_pixel > 1 ? "s" : "") << "]";
            SDL_SetWindowTitle(window, os.str().c_str());
            frames = 0;
            timing = 0;
        }

        update_texture(buf.get(), (float*)frame_buffer->map(), texture, width, height, iter);
        frame_buffer->unmap();

        SDL_RenderClear(renderer);
        SDL_RenderCopy(renderer, texture, nullptr, nullptr);
        SDL_RenderPresent(renderer);
    }

    context->destroy();

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
