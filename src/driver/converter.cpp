#include <iostream>
#include <fstream>
#include <unordered_map>
#include <unordered_set>
#include <cstring>
#include <limits>

#include <lz4.h>

#include "bvh.h"
#include "interface.h"
#include "obj.h"
#include "buffer.h"

#ifdef WIN32
#include <direct.h>
#define create_directory(d) _mkdir(d)
#else
#include <sys/stat.h>
#define create_directory(d) { umask(0); mkdir(d, 0777); }
#endif

enum class Target : uint32_t {
    AVX2 = 1,
    AVX = 2,
    SSE42 = 3,
    ASIMD = 4,
    NVVM = 5
};

inline Target cpuid() {
    std::ifstream info(CPUINFO_PATH);
    if (!info) return Target(0);

    std::string line;
    std::vector<std::string> isa_list{
        "asimd", "sse4_2", "avx", "avx2"
    };
    std::unordered_set<std::string> detected;
    while (std::getline(info, line)) {
        for (auto isa : isa_list) {
            if (line.find(isa) != std::string::npos)
                detected.insert(isa);
        }
    }
    if (detected.count("avx2"))   return Target::AVX2;
    if (detected.count("avx"))    return Target::AVX;
    if (detected.count("sse4_2")) return Target::SSE42;
    if (detected.count("asimd"))  return Target::ASIMD;
    return Target(0);
}

void copy_file(const std::string& src, const std::string& dst) {
    for (size_t i = 0; i < dst.size();) {
        auto j = dst.find_first_of("/\\", i);
        if (j == std::string::npos)
            break;
        create_directory(dst.substr(0, j).c_str());
        i = j + 1;
    }
    info("Copying '", src, "' to '", dst, "'");
    std::ifstream is(src, std::ios::binary);
    std::ofstream os(dst, std::ios::binary);
    assert(is && os);
    os << is.rdbuf();
}

inline std::string make_id(const std::string& str) {
    auto id = str;
    std::transform(id.begin(), id.end(), id.begin(), [] (char c) {
        if (std::isspace(c) || !std::isalnum(c)) return '_';
        return char(std::tolower(c));
    });
    return id;
}

inline bool ends_with(const std::string& str, const std::string& ext) {
    return str.rfind(ext) == str.length() - ext.length();
}

template <size_t N, size_t M>
struct BvhNTriM {};

template <>
struct BvhNTriM<8, 4> {
    using Node = Node8;
    using Tri  = Tri4;
};

template <>
struct BvhNTriM<4, 4> {
    using Node = Node4;
    using Tri  = Tri4;
};

template <>
struct BvhNTriM<2, 1> {
    using Node = Node2;
    using Tri  = Tri1;
};

template <size_t N, size_t M>
class BvhNTriMAdapter {
    struct CostFn {
        static float leaf_cost(int count, float area) {
            return ((count - 1) / M + 1) * area;
        }
        static float traversal_cost(float area) {
            return area * 0.5f;
        }
    };

    using BvhBuilder = SplitBvhBuilder<N, CostFn>;
    using Adapter    = BvhNTriMAdapter;
    using Node       = typename BvhNTriM<N, M>::Node;
    using Tri        = typename BvhNTriM<N, M>::Tri;

    std::vector<Node>& nodes_;
    std::vector<Tri>&  tris_;
    BvhBuilder         builder_;

public:
    BvhNTriMAdapter(std::vector<Node>& nodes, std::vector<Tri>& tris)
        : nodes_(nodes), tris_(tris)
    {}

    void build(const std::vector<::Tri>& tris) {
        builder_.build(tris, NodeWriter(*this), LeafWriter(*this, tris), M);
    }

#ifdef STATISTICS
    void print_stats() const override { builder_.print_stats(); }
#endif

private:
    struct NodeWriter {
        Adapter& adapter;

        NodeWriter(Adapter& adapter)
            : adapter(adapter)
        {}

        template <typename BBoxFn>
        int operator() (int parent, int child, const BBox& /*parent_bb*/, size_t count, BBoxFn bboxes) {
            auto& nodes = adapter.nodes_;

            size_t i = nodes.size();
            nodes.emplace_back();

            if (parent >= 0 && child >= 0) {
                assert(parent >= 0 && parent < nodes.size());
                assert(child >= 0 && child < N);
                nodes[parent].child[child] = i + 1;
            }

            assert(count >= 1 && count <= N);

            for (size_t j = 0; j < count; j++) {
                const BBox& bbox = bboxes(j);
                nodes[i].bounds[0][j] = bbox.min.x;
                nodes[i].bounds[2][j] = bbox.min.y;
                nodes[i].bounds[4][j] = bbox.min.z;

                nodes[i].bounds[1][j] = bbox.max.x;
                nodes[i].bounds[3][j] = bbox.max.y;
                nodes[i].bounds[5][j] = bbox.max.z;
            }

            for (size_t j = count; j < N; ++j) {
                nodes[i].bounds[0][j] = std::numeric_limits<float>::infinity();
                nodes[i].bounds[2][j] = std::numeric_limits<float>::infinity();
                nodes[i].bounds[4][j] = std::numeric_limits<float>::infinity();

                nodes[i].bounds[1][j] = -std::numeric_limits<float>::infinity();
                nodes[i].bounds[3][j] = -std::numeric_limits<float>::infinity();
                nodes[i].bounds[5][j] = -std::numeric_limits<float>::infinity();

                nodes[i].child[j] = 0;
            }

            return i;
        }
    };

    struct LeafWriter {
        Adapter& adapter;
        const std::vector<::Tri>& in_tris;

        LeafWriter(Adapter& adapter, const std::vector<::Tri>& in_tris)
            : adapter(adapter)
            , in_tris(in_tris)
        {}

        template <typename RefFn>
        void operator() (int parent, int child, const BBox& /*leaf_bb*/, size_t ref_count, RefFn refs) {
            auto& nodes   = adapter.nodes_;
            auto& tris    = adapter.tris_;

            nodes[parent].child[child] = ~tris.size();

            // Group triangles by packets of M
            for (size_t i = 0; i < ref_count; i += M) {
                const size_t c = i + M <= ref_count ? M : ref_count - i;

                Tri tri;
                std::memset(&tri, 0, sizeof(Tri));
                for (size_t j = 0; j < c; j++) {
                    const int id = refs(i + j);
                    auto& in_tri = in_tris[id];
                    const float3 e1 = in_tri.v0 - in_tri.v1;
                    const float3 e2 = in_tri.v2 - in_tri.v0;
                    const float3 n = cross(e1, e2);
                    tri.v0[0][j] = in_tri.v0.x;
                    tri.v0[1][j] = in_tri.v0.y;
                    tri.v0[2][j] = in_tri.v0.z;

                    tri.e1[0][j] = e1.x;
                    tri.e1[1][j] = e1.y;
                    tri.e1[2][j] = e1.z;

                    tri.e2[0][j] = e2.x;
                    tri.e2[1][j] = e2.y;
                    tri.e2[2][j] = e2.z;

                    tri.n[0][j] = n.x;
                    tri.n[1][j] = n.y;
                    tri.n[2][j] = n.z;

                    tri.id[j] = id;
                }

                for (size_t j = c; j < 4; j++)
                    tri.id[j] = 0xFFFFFFFF;

                tris.emplace_back(tri);
            }
            assert(ref_count > 0);
            tris.back().id[M - 1] |= 0x80000000;
        }
    };
};

template <>
class BvhNTriMAdapter<2, 1> {
    struct CostFn {
        static float leaf_cost(int count, float area) {
            return count * area;
        }
        static float traversal_cost(float area) {
            return area * 1.0f;
        }
    };

    using BvhBuilder = SplitBvhBuilder<2, CostFn>;
    using Adapter    = BvhNTriMAdapter;
    using Node       = Node2;
    using Tri        = Tri1;

    std::vector<Node>& nodes_;
    std::vector<Tri>&  tris_;
    BvhBuilder         builder_;

public:
    BvhNTriMAdapter(std::vector<Node>& nodes, std::vector<Tri>& tris)
        : nodes_(nodes), tris_(tris)
    {}

    void build(const std::vector<::Tri>& tris) {
        builder_.build(tris, NodeWriter(*this), LeafWriter(*this, tris), 4);
    }

#ifdef STATISTICS
    void print_stats() const override { builder_.print_stats(); }
#endif

private:
    struct NodeWriter {
        Adapter& adapter;

        NodeWriter(Adapter& adapter)
            : adapter(adapter)
        {}

        template <typename BBoxFn>
        int operator() (int parent, int child, const BBox& /*parent_bb*/, size_t count, BBoxFn bboxes) {
            auto& nodes = adapter.nodes_;

            size_t i = nodes.size();
            nodes.emplace_back();

            if (parent >= 0 && child >= 0) {
                assert(parent >= 0 && parent < nodes.size());
                assert(child >= 0 && child < 2);
                nodes[parent].child[child] = i + 1;
            }

            assert(count >= 1 && count <= 2);

            const BBox& bbox1 = bboxes(0);
            nodes[i].bounds[0] = bbox1.min.x;
            nodes[i].bounds[2] = bbox1.min.y;
            nodes[i].bounds[4] = bbox1.min.z;
            nodes[i].bounds[1] = bbox1.max.x;
            nodes[i].bounds[3] = bbox1.max.y;
            nodes[i].bounds[5] = bbox1.max.z;

            if (count == 2) {
                const BBox& bbox2 = bboxes(1);
                nodes[i].bounds[ 6] = bbox2.min.x;
                nodes[i].bounds[ 8] = bbox2.min.y;
                nodes[i].bounds[10] = bbox2.min.z;
                nodes[i].bounds[ 7] = bbox2.max.x;
                nodes[i].bounds[ 9] = bbox2.max.y;
                nodes[i].bounds[11] = bbox2.max.z;
            } else {
                nodes[i].bounds[ 6] =  std::numeric_limits<float>::infinity();
                nodes[i].bounds[ 8] =  std::numeric_limits<float>::infinity();
                nodes[i].bounds[10] =  std::numeric_limits<float>::infinity();
                nodes[i].bounds[ 7] = -std::numeric_limits<float>::infinity();
                nodes[i].bounds[ 9] = -std::numeric_limits<float>::infinity();
                nodes[i].bounds[11] = -std::numeric_limits<float>::infinity();
            }

            return i;
        }
    };

    struct LeafWriter {
        Adapter& adapter;
        const std::vector<::Tri>& in_tris;

        LeafWriter(Adapter& adapter, const std::vector<::Tri>& in_tris)
            : adapter(adapter)
            , in_tris(in_tris)
        {}

        template <typename RefFn>
        void operator() (int parent, int child, const BBox& /*leaf_bb*/, size_t ref_count, RefFn refs) {
            auto& nodes   = adapter.nodes_;
            auto& tris    = adapter.tris_;

            nodes[parent].child[child] = ~tris.size();

            for (int i = 0; i < ref_count; i++) {
                const int ref = refs(i);
                auto& tri = in_tris[ref];
                auto e1 = tri.v0 - tri.v1;
                auto e2 = tri.v2 - tri.v0;
                auto n = cross(e1, e2);
                Tri1 new_tri{
                    { tri.v0.x, tri.v0.y, tri.v0.z}, n.x,
                    { e1.x, e1.y, e1.z}, n.y,
                    { e2.x, e2.y, e2.z}, ref
                };
                tris.emplace_back(new_tri);
            }

            // Add sentinel
            tris.back().id |= 0x80000000;
        }
    };
};

static void write_tri_mesh(const obj::TriMesh& tri_mesh) {
    write_buffer("data/vertices.bin",     tri_mesh.vertices);
    write_buffer("data/normals.bin",      tri_mesh.normals);
    write_buffer("data/face_normals.bin", tri_mesh.face_normals);
    write_buffer("data/indices.bin",      tri_mesh.indices);
    write_buffer("data/texcoords.bin",    tri_mesh.texcoords);
}

template <size_t N, size_t M>
static void write_bvhn_trim(const obj::TriMesh& tri_mesh) {
    using Node = typename BvhNTriM<N, M>::Node;
    using Tri  = typename BvhNTriM<N, M>::Tri;

    std::vector<Node> nodes;
    std::vector<Tri>  tris;
    BvhNTriMAdapter<N, M> adapter(nodes, tris);
    auto num_tris = tri_mesh.indices.size() / 4;
    std::vector<::Tri> in_tris(num_tris);
    for (size_t i = 0; i < num_tris; i++) {
        auto& v0 = tri_mesh.vertices[tri_mesh.indices[i * 4 + 0]];
        auto& v1 = tri_mesh.vertices[tri_mesh.indices[i * 4 + 1]];
        auto& v2 = tri_mesh.vertices[tri_mesh.indices[i * 4 + 2]];
        in_tris[i] = ::Tri(v0, v1, v2);
    }
    adapter.build(in_tris);

    std::ofstream of("data/bvh.bin", std::ios::app | std::ios::binary);
    size_t node_size = sizeof(Node);
    size_t tri_size  = sizeof(Tri);
    of.write((char*)&node_size, sizeof(uint32_t));
    of.write((char*)&tri_size,  sizeof(uint32_t));
    write_buffer(of, nodes);
    write_buffer(of, tris );
}

static bool convert_obj(const std::string& file_name, Target target, std::ostream& os) {
    info("Converting OBJ file '", file_name, "'");
    obj::File obj_file;
    obj::MaterialLib mtl_lib;
    FilePath path(file_name);
    if (!obj::load_obj(path, obj_file)) {
        error("Invalid OBJ file '", file_name, "'");
        return false;
    }
    for (auto lib_name : obj_file.mtl_libs) {
        auto mtl_name = path.base_name() + "/" + lib_name;
        if (!obj::load_mtl(mtl_name, mtl_lib)) {
            error("Invalid MTL file '", mtl_name, "'");
            return false;
        }
    }

    // Check that all materials exist
    for (auto& mtl_name : obj_file.materials) {
        if (mtl_name != "" && !mtl_lib.count(mtl_name)) {
            warn("Missing material definition for '", mtl_name, "'. Replaced by dummy material.");
            mtl_name = "";
        }
    }

    std::unordered_map<std::string, size_t> images;
    for (auto& pair : mtl_lib) {
        auto & mat = pair.second;
        if (mat.map_kd != "")   images.emplace(mat.map_kd, images.size());
        if (mat.map_ks != "")   images.emplace(mat.map_kd, images.size());
        if (mat.map_ke != "")   images.emplace(mat.map_ke, images.size());
    }

    auto tri_mesh = compute_tri_mesh(obj_file, mtl_lib, 0);

    // Generate images
    std::vector<std::string> image_names(images.size());
    for (auto& pair : images)
        image_names[pair.second] = pair.first;

    create_directory("data/");

    os << "//------------------------------------------------------------------------------------\n"
       << "// Generated from '" << path.file_name() << "' with the scene conversion tool\n"
       << "//------------------------------------------------------------------------------------\n\n";

    os << "struct Settings {\n"
       << "    eye: Vec3,\n"
       << "    dir: Vec3,\n"
       << "    up: Vec3,\n"
       << "    right: Vec3,\n"
       << "    width: f32,\n"
       << "    height: f32\n"
       << "};\n";

    os << "\nextern fn render(settings: &Settings, iter: i32) -> () {\n";

    assert(target != Target(0));
    switch (target) {
        case Target::AVX2:  os << "    let device   = make_avx2_device();\n";  break;
        case Target::AVX:   os << "    let device   = make_avx_device();\n";   break;
        case Target::SSE42: os << "    let device   = make_sse42_device();\n"; break;
        case Target::ASIMD: os << "    let device   = make_asimd_device();\n"; break;
        case Target::NVVM:  os << "    let device   = make_nvvm_device(0);\n";  break;
        default:
            assert(false);
            break;
    }

    os << "    let renderer = make_path_tracing_renderer(32 /*max_path_len*/);\n"
       << "    let math     = device.intrinsics;\n";

    // Setup camera
    os << "\n    // Camera\n"
       << "    let camera = make_perspective_camera(\n"
       << "        settings.eye,\n"
       << "        make_mat3x3(settings.right, settings.up, settings.dir),\n"
       << "        settings.width,\n"
       << "        settings.height\n"
       << "    );\n";

    // Generate geometry
    os << "\n    // Geometry\n"
       << "    let vertices     = device.load_buffer(\"data/vertices.bin\")     as &[Vec3];\n"
       << "    let normals      = device.load_buffer(\"data/normals.bin\")      as &[Vec3];\n"
       << "    let face_normals = device.load_buffer(\"data/face_normals.bin\") as &[Vec3];\n"
       << "    let indices      = device.load_buffer(\"data/indices.bin\")      as &[i32];\n"
       << "    let texcoords    = device.load_buffer(\"data/texcoords.bin\")    as &[Vec2];\n"
       << "    let tri_mesh     = TriMesh {\n"
       << "        vertices:     @ |i| vertices(i),\n"
       << "        normals:      @ |i| normals(i),\n"
       << "        face_normals: @ |i| face_normals(i),\n"
       << "        triangles:    @ |i| (indices(i * 4 + 0), indices(i * 4 + 1), indices(i * 4 + 2)),\n"
       << "        materials:    @ |i| indices(i * 4 + 3),\n"
       << "        attrs:        @ |_| (false, @ |j| vec2_to_4(texcoords(j), 0.0f, 0.0f)),\n"
       << "        num_attrs:    1,\n"
       << "        num_tris:     " << tri_mesh.indices.size() / 4 << "\n"
       << "    };\n"
       << "    let bvh = device.load_bvh(\"data/bvh.bin\", 0);\n"
       << "    let geometries = @ |_| make_tri_mesh_geometry(math, tri_mesh, bvh);\n";

    info("Generating geometry for '", file_name, "'");
    write_tri_mesh(tri_mesh);

    // Generate BVHs
    info("Generating BVH for '", file_name, "'");
    std::remove("data/bvh.bin");
    if (target == Target::NVVM)
        write_bvhn_trim<2, 1>(tri_mesh);
    else if (target == Target::ASIMD || target == Target::SSE42)
        write_bvhn_trim<4, 4>(tri_mesh);
    else
        write_bvhn_trim<8, 4>(tri_mesh);

    // Generate images
    os << "\n    // Images\n"
       << "    let dummy_image = make_image(@ |x, y| make_color(0.0f, 0.0f, 0.0f), 1, 1);\n";
    for (size_t i = 0; i < images.size(); i++) {
        auto& name = image_names[i];
        copy_file(path.base_name() + "/" + name, "data/" + name);
        os << "    let image_" << make_id(name) << " = ";
        if (ends_with(name, ".png")) {
            os << "device.load_png(\"data/" << name << "\");\n";
        } else if (ends_with(name, ".tga")) {
            os << "device.load_tga(\"data/" << name << "\");\n";
        } else if (ends_with(name, ".tiff")) {
            os << "device.load_tga(\"data/" << name << "\");\n";
        } else if (ends_with(name, ".jpeg") || ends_with(name, ".jpg")) {
            os << "device.load_jpg(\"data/" << name << "\");\n";
        } else {
            os << "dummy_image; // Cannot determine image type for " << name << "\n";
        }
    }
    os << "    let images = @ |i| match i {\n";
    for (size_t i = 0; i < images.size(); i++) {
        os << "        " << i << " => image_" << make_id(image_names[i]) << ",\n";
    }
    os << "        _ => dummy_image\n"
       << "    };\n";

    // Lights
    std::vector<int> light_ids(tri_mesh.indices.size() / 4, 0);
    os << "\n    // Lights\n"
       << "    let dummy_light = make_point_light(make_vec3(0.0f, 0.0f, 0.0f), make_color(0.0f, 0.0f, 0.0f));\n";
    size_t num_lights = 0;
    for (size_t i = 0; i < tri_mesh.indices.size(); i += 4) {
        auto& mtl_name = obj_file.materials[tri_mesh.indices[i + 3]];
        if (mtl_name == "")
            continue;
        auto& mat = mtl_lib.find(mtl_name)->second;
        if (mat.ke == rgb(0.0f) && mat.map_ke == "")
            continue;

        auto& v0 = tri_mesh.vertices[tri_mesh.indices[i + 0]];
        auto& v1 = tri_mesh.vertices[tri_mesh.indices[i + 1]];
        auto& v2 = tri_mesh.vertices[tri_mesh.indices[i + 2]];
        light_ids[i / 4] = num_lights++;
        os << "    let light" << num_lights - 1 << " = make_triangle_light(\n"
           << "        math,\n"
           << "        make_vec3(" << v0.x << "f, " << v0.y << "f, " << v0.z << "f),\n"
           << "        make_vec3(" << v1.x << "f, " << v1.y << "f, " << v1.z << "f),\n"
           << "        make_vec3(" << v2.x << "f, " << v2.y << "f, " << v2.z << "f),\n";
        if (mat.map_ke != "") {
            os << "        make_texture(math, make_repeat_border(), make_bilinear_filter(), scene.images(" << images[mat.map_ke] << "))\n";
        } else {
            os << "        make_color(" << mat.ke.x << "f, " << mat.ke.y << "f, " << mat.ke.z << "f)\n";
        }
        os << "    );\n";
    }
    os << "    let lights = @ |i| match i {\n";
    for (size_t i = 0; i < num_lights; ++i)
        os << "        " << i << " => light" << i << ",\n";
    os << "        _ => dummy_light\n"
       << "    };\n";

    write_buffer("data/light_ids.bin", light_ids);

    os << "\n    // Mapping from primitive to light source\n"
       << "    let light_ids = device.load_buffer(\"data/light_ids.bin\") as &[i32];\n";

    // Generate shaders
    info("Generating materials for '", file_name, "'");
    os << "\n    // Shaders\n";
    os << "    let dummy_shader = @ |math, scene, ray, hit, surf| make_material(make_diffuse_bsdf(surf, make_color(0.0f, 1.0f, 1.0f)));\n";
    for (auto& mtl_name : obj_file.materials) {
        if (mtl_name == "")
            continue;
        auto it = mtl_lib.find(mtl_name);
        if (it == mtl_lib.end())
            continue;

        auto& mat = it->second;
        bool has_emission = mat.ke != rgb(0.0f) || mat.map_ke != "";
        os << "    let shader_" << make_id(mtl_name) << " = @ |math, scene, ray, hit, surf| {\n";
        if (mat.illum == 5) {
            os << "        let bsdf = make_mirror_bsdf(surf, make_color(" << mat.tf.x << "f, " << mat.tf.y << "f, " << mat.tf.z << "f));\n";
        } else if (mat.illum == 7) {
            os << "        let bsdf = make_glass_bsdf(surf, 1.0f, " << mat.ni << "f, make_color(" << mat.tf.x << "f, " << mat.tf.y << "f, " << mat.tf.z << "f));\n";
        } else {
            bool has_diffuse  = mat.kd != rgb(0.0f) || mat.map_kd != "";
            bool has_specular = mat.ks != rgb(0.0f) || mat.map_ks != "";

            if (has_diffuse) {
                if (mat.map_kd != "") {
                    os << "        let diffuse_texture = make_texture(math, make_repeat_border(), make_bilinear_filter(), scene.images(" << images[mat.map_kd] << "));\n";
                    os << "        let kd = diffuse_texture(vec4_to_2(surf.attr(0)));\n";
                } else {
                    os << "        let kd = make_color(" << mat.kd.x << "f, " << mat.kd.y << "f, " << mat.kd.z << "f);\n";
                }
                os << "        let diffuse = make_diffuse_bsdf(surf, kd);\n";
            }
            if (has_specular) {
                if (mat.map_ks != "") {
                    os << "        let specular_texture = make_texture(math, make_repeat_border(), make_bilinear_filter(), scene.images(" << images[mat.map_ks] << "));\n";
                    os << "        let ks = specular_texture(vec4_to_2(surf.attr(0)));\n";
                } else {
                    os << "        let ks = make_color(" << mat.ks.x << "f, " << mat.ks.y << "f, " << mat.ks.z << "f);\n";
                }
                os << "        let ns = " << mat.ns << "f;\n";
                os << "        let specular = make_phong_bsdf(surf, ks, ns);\n";
            }
            os << "        let bsdf = ";
            if (has_diffuse && has_specular) {
                os << "make_mix_bsdf(diffuse, specular, color_luminance(ks) / (color_luminance(ks) + color_luminance(kd)));\n";
            } else if (has_diffuse || has_specular) {
                if (has_specular) os << "specular;\n";
                else              os << "diffuse;\n";
            } else {
                os << "make_black_bsdf();\n";
            }
        }
        if (has_emission) {
            os << "        make_emissive_material(surf, bsdf, lights(light_ids(hit.prim_id)))\n";
        } else {
            os << "        make_material(bsdf)\n";
        }
        os << "    };\n";
    }
    os << "    let shaders = @ |i| match i {\n";
    for (size_t i = 0; i < obj_file.materials.size(); i++) {
        auto& mtl_name = obj_file.materials[i];
        auto it = mtl_lib.find(mtl_name);
        if (mtl_name == "" || it == mtl_lib.end()) continue;
        os << "        " << i << " => shader_" << make_id(mtl_name) << ",\n";
    }
    os << "        _ => dummy_shader\n"
       << "    };\n";

    // Scene
    os << "\n    // Scene\n"
       << "    let scene = Scene {\n"
       << "        num_shaders:    " << mtl_lib.size() + 1 << ",\n"
       << "        num_geometries: " << 1 << ",\n"
       << "        num_images:     " << images.size() << ",\n"
       << "        num_lights:     " << num_lights << ",\n"
       << "        shaders:        shaders,\n"
       << "        geometries:     geometries,\n"
       << "        images:         images,\n"
       << "        lights:         lights,\n"
       << "        camera:         camera\n"
       << "    };\n";

    os << "\n    renderer(scene, device, iter);\n"
       << "      device.present();\n"
       << "}\n";

    info("Scene was converted successfully");
    return true;
}

static void usage() {
    std::cout << "converter [options] file\n"
              << "Available options:\n"
              << "    -h     --help       Shows this message\n"
              << "    -t     --target     Sets the target device (one of: sse42, avx, avx2, asimd, nvvm)\n"
              << std::flush;
}

static bool check_option(int i, int argc, char** argv) {
    if (i + 1 >= argc) {
        std::cerr << "Missing argument for '" << argv[i] << "'. Aborting." << std::endl;
        return false;
    }
    return true;
}

int main(int argc, char** argv) {
    if (argc <= 1) {
        std::cerr << "Not enough arguments. Run with --help to get a list of options." << std::endl;
        return 1;
    }

    std::string obj_file;
    auto target = Target(0);
    for (int i = 1; i < argc; ++i) {
        if (argv[i][0] == '-') {
            if (!strcmp(argv[i], "-h") || !strcmp(argv[i], "--help")) {
                usage();
                return 0;
            } else if (!strcmp(argv[i], "-t") || !strcmp(argv[i], "--target")) {
                if (!check_option(i, argc, argv)) return 1;
                i++;
                if (!strcmp(argv[i], "sse42"))
                    target = Target::SSE42;
                else if (!strcmp(argv[i], "avx"))
                    target = Target::AVX;
                else if (!strcmp(argv[i], "avx2"))
                    target = Target::AVX2;
                else if (!strcmp(argv[i], "asimd"))
                    target = Target::ASIMD;
                else if (!strcmp(argv[i], "nvvm"))
                    target = Target::NVVM;
            } else {
                std::cerr << "Unknown option '" << argv[i] << "'. Aborting." << std::endl;
                return 1;
            }
        } else {
            if (obj_file != "") {
                std::cerr << "Only one OBJ file can be converted. Aborting." << std::endl;
                return 1;
            }
            obj_file = argv[i];
        }
    }

    if (target == Target(0)) {
        target = cpuid();
        if (target == Target(0)) {
            std::cerr << "No vector instruction set detected. Aborting." << std::endl;
            return 1;
        }
    }

    std::ofstream of("main.impala");
    if (!convert_obj(obj_file, target, of))
        return 1;
    return 0;
}
