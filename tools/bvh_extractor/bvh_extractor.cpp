#include <iostream>
#include <fstream>
#include <vector>
#include <cstring>

#include "load_obj.h"
#include "file_path.h"
#include "tri.h"

bool build_bvh4(std::ofstream&, const std::vector<Tri>&);
bool build_bvh2(std::ofstream&, const std::vector<Tri>&);

inline void check_argument(int i, int argc, char** argv) {
    if (i + 1 >= argc) {
        std::cerr << "Missing argument for " << argv[i] << std::endl;
        exit(1);
    }
}

inline void usage() {
    std::cout << "Usage: bvh_extractor [options]\n"
                 "Available options:\n"
                 "  -obj     --obj-file        Sets the OBJ file to use\n"
                 "  -o       --output          Sets the output file name\n";
}

static void create_triangles(const obj::File& obj_file, std::vector<Tri>& tris) {
    for (auto& object : obj_file.objects) {
        for (auto& group : object.groups) {
            for (auto& face : group.faces) {
                auto v0 = obj_file.vertices[face.indices[0].v];
                for (int i = 0; i < face.index_count - 2; i++) {
                    auto v1 = obj_file.vertices[face.indices[i + 1].v];
                    auto v2 = obj_file.vertices[face.indices[i + 2].v];
                    tris.emplace_back(float3(v0.x, v0.y, v0.z),
                                      float3(v1.x, v1.y, v1.z),
                                      float3(v2.x, v2.y, v2.z));
                }
            }
        }
    }
}

int main(int argc, char** argv) {
    std::string obj_file, out_file;
    for (int i = 1; i < argc; i++) {
        auto arg = argv[i];
        if (arg[0] == '-') {
            if (!strcmp(arg, "-h") || !strcmp(arg, "--help")) {
                usage();
                return 0;
            } else if (!strcmp(arg, "-obj") || !strcmp(arg, "--obj-file")) {
                check_argument(i, argc, argv);
                obj_file = argv[++i];
            } else if (!strcmp(arg, "-o") || !strcmp(arg, "--output")) {
                check_argument(i, argc, argv);
                out_file = argv[++i];
            } else {
                std::cerr << "Unknown option '" << arg << "'" << std::endl;
                return 1;
            }
        } else {
            std::cerr << "Invalid argument '" << arg << "'" << std::endl;
            return 1;
        }
    }

    if (obj_file == "") {
        std::cerr << "No OBJ file specified" << std::endl;
        return 1;
    }
    if (out_file == "") {
        std::cerr << "No output file specified" << std::endl;
        return 1;
    }

    obj::File obj;
    if (!load_obj(obj_file, obj)) {
        std::cerr << "Cannot load OBJ file" << std::endl;
        return 1;
    }

    std::vector<Tri> tris;
    create_triangles(obj, tris);

    std::cout << "Loaded OBJ file with " << tris.size() << " triangle(s)" << std::endl;

    std::ofstream out(out_file, std::ofstream::binary);
    if (!out) {
        std::cerr << "Cannot create output file" << std::endl;
        return 1;
    }

    uint32_t magic = 0x95CBED1F;
    out.write((char*)&magic, sizeof(uint32_t));

    if (!build_bvh4(out, tris)) {
        std::cerr << "Cannot build a BVH4 using Embree" << std::endl;
        return 1;
    }

    std::cout << "BVH4 successfully built" << std::endl;

    if (!build_bvh2(out, tris)) {
        std::cerr << "Cannot build a BVH2" << std::endl;
        return 1;
    }

    std::cout << "BVH2 successfully built" << std::endl;

    return 0;
}
