#include <embree2/rtcore.h>

#include <config.h>
#include <kernels/common/alloc.h>
#include <kernels/builders/bvh_builder_sah.h>

#include <iostream>
#include <vector>
#include <cstring>

#include "load_obj.h"
#include "file_path.h"

using namespace embree;

struct Node {
    virtual float sah() = 0;
};

template <int N>
struct InnerNode : public Node {
    BBox3fa bounds[N];
    Node* children[N];

    InnerNode() {
        for (int i = 0; i < N; i++) {
            bounds[i]   = empty;
            children[i] = nullptr;
        }
    }

    float sah() {
        BBox3fa m = empty;
        float c = 1.0f;
        for (int i = 0; i < N; i++) {
            c += area(bounds[i]) * children[i]->sah();
            m = merged(m, bounds[i]);
        }
        return c / area(m);
    }
};

struct LeafNode : public Node {
    std::vector<size_t> ids;
    BBox3fa bounds;

    LeafNode (const std::vector<size_t>& ids, const BBox3fa& bounds)
        : ids(ids), bounds(bounds) {}

    float sah() { return ids.size(); }
};

struct Tri {
    Vec3fa v0, v1, v2;

    Tri() {}
    Tri(const Vec3fa& v0, const Vec3fa& v1, const Vec3fa& v2)
        : v0(v0), v1(v1), v2(v2)
    {}
};

void error_handler(const RTCError code, const char* str) {
    if (code == RTC_NO_ERROR)
        return;

    std::cerr << "Embree error: ";
    switch (code) {
        case RTC_UNKNOWN_ERROR:     std::cerr << "RTC_UNKNOWN_ERROR";       break;
        case RTC_INVALID_ARGUMENT:  std::cerr << "RTC_INVALID_ARGUMENT";    break;
        case RTC_INVALID_OPERATION: std::cerr << "RTC_INVALID_OPERATION";   break;
        case RTC_OUT_OF_MEMORY:     std::cerr << "RTC_OUT_OF_MEMORY";       break;
        case RTC_UNSUPPORTED_CPU:   std::cerr << "RTC_UNSUPPORTED_CPU";     break;
        case RTC_CANCELLED:         std::cerr << "RTC_CANCELLED";           break;
        default:                    std::cerr << "invalid error code";      break;
    }

    if (str) std::cerr << " (" << str << ")";
    std::cerr << std::endl;

    exit(1);
}

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
                    tris.emplace_back(Vec3fa(v0.x, v0.y, v0.z),
                                      Vec3fa(v1.x, v1.y, v1.z),
                                      Vec3fa(v2.x, v2.y, v2.z));
                }
            }
        }
    }
}

template <int Arity>
void build_bvh(const std::vector<Tri>& tris) {
    FastAllocator allocator(nullptr, false);

    size_t max_depth = 64;

    Node* root;
    /*isa::BVHBuilderBinnedFastSpatialSAH::build<Node*>(
        root,
        [&] () -> FastAllocator::ThreadLocal* {
          return allocator.threadLocal();
        },
        [&] (const isa::BVHBuilderBinnedFastSpatialSAH::BuildRecord& current,
            isa::BVHBuilderBinnedSAH::BuildRecord* children,
            const size_t N, FastAllocator::ThreadLocal* alloc) -> int {
          assert(N <= Arity);
          InnerNode* node = new (alloc->malloc(sizeof(InnerNode))) InnerNode;
          for (size_t i = 0; i < N; i++) {
            node->bounds[i] = children[i].pinfo.geomBounds;
            children[i].parent = (size_t*)&node->children[i];
          }
          *current.parent = (size_t)node;
          return 0;
        },
        [&] (const isa::BVHBuilderBinnedSAH::BuildRecord& current,
             FastAllocator::ThreadLocal* alloc) -> int {
          assert(current.prims.size() >= 1);
          Node* node = new (alloc->malloc(sizeof(LeafNode))) LeafNode(prims[current.prims.begin()].ID(),prims[current.prims.begin()].bounds());
          *current.parent = (size_t) node;
          return 0;
        },
        [&] () {},
        [&] (size_t dn) {},
        prims.data(),
        pinfo,
        Arity,
        max_depth,
        1,1,1,
        1.0f,1.0f,
        Builder::DEFAULT_SINGLE_THREAD_THRESHOLD);*/
}

int main(int argc, char** argv) {
    auto device = rtcNewDevice(nullptr);
    error_handler(rtcDeviceGetError(device), "");

    rtcDeviceSetErrorFunction(device, error_handler);

    std::string obj_file, output;
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
                output = argv[++i];
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
    if (output == "") {
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
    build_bvh<2>(tris);

    rtcDeleteDevice(device);
    return 0;
}
