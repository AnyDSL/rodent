#include <iostream>
#include <fstream>
#include <vector>
#include <cstring>

#include "driver/obj.h"
#include "driver/file_path.h"
#include "driver/bvh.h"

#ifdef ENABLE_EMBREE_BVH
size_t build_bvh8(std::ofstream&, const obj::TriMesh&);
size_t build_bvh4(std::ofstream&, const obj::TriMesh&);
#endif
size_t build_bvh2(std::ofstream&, const obj::TriMesh&);

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

    FilePath path(obj_file);
    obj::File obj;
    if (!load_obj(obj_file, obj)) {
        std::cerr << "Cannot load OBJ file" << std::endl;
        return 1;
    }
    obj::TriMesh tri_mesh = compute_tri_mesh(obj, 0);

    std::cout << "Loaded OBJ file with " << tri_mesh.indices.size() / 4 << " triangle(s)" << std::endl;

    std::ofstream out(out_file, std::ofstream::binary);
    if (!out) {
        std::cerr << "Cannot create output file" << std::endl;
        return 1;
    }

    uint32_t magic = 0x95CBED1F;
    out.write((char*)&magic, sizeof(uint32_t));

#ifdef ENABLE_EMBREE_BVH
    auto bvh8_nodes = build_bvh8(out, tri_mesh);
    if (!bvh8_nodes) {
        std::cerr << "Cannot build a BVH8 using Embree" << std::endl;
        return 1;
    }

    std::cout << "BVH8 successfully built (" << bvh8_nodes << " nodes)" << std::endl;

    auto bvh4_nodes = build_bvh4(out, tri_mesh);
    if (!bvh4_nodes) {
        std::cerr << "Cannot build a BVH4 using Embree" << std::endl;
        return 1;
    }

    std::cout << "BVH4 successfully built (" << bvh4_nodes << " nodes)" << std::endl;
#else
    std::cout << "Compiled without Embree. Will only build a GPU BVH." << std::endl;
#endif

    auto bvh2_nodes = build_bvh2(out, tri_mesh);
    if (!bvh2_nodes) {
        std::cerr << "Cannot build a BVH2" << std::endl;
        return 1;
    }

    std::cout << "BVH2 successfully built (" << bvh2_nodes << " nodes)" << std::endl;

    return 0;
}
