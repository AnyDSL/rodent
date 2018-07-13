#include <unordered_map>
#include <memory>
#include <string>
#include <fstream>
#include <cstring>

#include <anydsl_runtime.hpp>
#include <png.h>

#include "interface.h"
#include "load_obj.h"
#include "bvh.h"

// Triangle Meshes -----------------------------------------------------------------

struct TriIdx {
    int32_t v0, v1, v2, m;
    TriIdx(int32_t v0, int32_t v1, int32_t v2, int32_t m)
        : v0(v0), v1(v1), v2(v2), m(m)
    {}
};

struct HashIndex {
    size_t operator () (const obj::Index& i) const {
        unsigned h = 0, g;

        h = (h << 4) + i.v;
        g = h & 0xF0000000;
        h = g ? (h ^ (g >> 24)) : h;
        h &= ~g;

        h = (h << 4) + i.t;
        g = h & 0xF0000000;
        h = g ? (h ^ (g >> 24)) : h;
        h &= ~g;

        h = (h << 4) + i.n;
        g = h & 0xF0000000;
        h = g ? (h ^ (g >> 24)) : h;
        h &= ~g;

        return h;
    }
};

struct CompareIndex {
    bool operator () (const obj::Index& a, const obj::Index& b) const {
        return a.v == b.v && a.t == b.t && a.n == b.n;
    }
};

static void compute_face_normals(const std::vector<uint32_t>& indices,
                                 const std::vector<float3>& vertices,
                                 std::vector<float3>& face_normals,
                                 size_t first_index) {
    for (auto i = first_index, k = indices.size(); i < k; i += 4) {
        const float3& v0 = vertices[indices[i + 0]];
        const float3& v1 = vertices[indices[i + 1]];
        const float3& v2 = vertices[indices[i + 2]];
        face_normals[i / 4] = normalize(cross(v1 - v0, v2 - v0));
    }
}

static void compute_vertex_normals(const std::vector<uint32_t>& indices,
                                   const std::vector<float3>& face_normals,
                                   std::vector<float3>& normals,
                                   size_t first_index) {
    for (auto i = first_index, k = indices.size(); i < k; i += 4) {
        float3& n0 = normals[indices[i + 0]];
        float3& n1 = normals[indices[i + 1]];
        float3& n2 = normals[indices[i + 2]];
        const float3& n = face_normals[i / 4];
        n0 += n;
        n1 += n;
        n2 += n;
    }
}

static TriMesh load_tri_mesh(int32_t dev, std::string file_name, std::vector<Tri>& tris) {
    static size_t mtl_offset = 0;

    obj::File obj_file;
    if (!load_obj(file_name, obj_file)) {
        error("Cannot load file '", file_name, "'.");
        abort();
    }

    std::vector<uint32_t> indices;
    std::vector<float3>   vertices;
    std::vector<float3>   normals;
    std::vector<float3>   face_normals;
    std::vector<float2>   texcoords;

    for (auto& obj: obj_file.objects) {
        // Convert the faces to triangles & build the new list of indices
        std::vector<TriIdx> triangles;
        std::unordered_map<obj::Index, size_t, HashIndex, CompareIndex> mapping;

        bool has_normals = false;
        bool has_texcoords = false;
        for (auto& group : obj.groups) {
            for (auto& face : group.faces) {
                for (int i = 0; i < face.index_count; i++) {
                    auto map = mapping.find(face.indices[i]);
                    if (map == mapping.end()) {
                        has_normals |= (face.indices[i].n != 0);
                        has_texcoords |= (face.indices[i].t != 0);

                        mapping.insert(std::make_pair(face.indices[i], mapping.size()));
                    }
                }

                auto v0 = mapping[face.indices[0]];
                auto prev = mapping[face.indices[1]];

                for (int i = 1; i < face.index_count - 1; i++) {
                    auto next = mapping[face.indices[i + 1]];
                    triangles.emplace_back(v0, prev, next, face.material + mtl_offset);
                    prev = next;
                }
            }
        }

        if (triangles.size() == 0) continue;

        // Add this object to the mesh
        auto vtx_offset = vertices.size();
        auto idx_offset = indices.size();
        indices.resize(idx_offset + 4 * triangles.size());
        vertices.resize(vtx_offset + mapping.size());
        texcoords.resize(vtx_offset + mapping.size());
        normals.resize(vtx_offset + mapping.size());

        for (int i = 0, n = triangles.size(); i < n; i++) {
            auto& t = triangles[i];
            indices[idx_offset + i * 4 + 0] = t.v0 + vtx_offset;
            indices[idx_offset + i * 4 + 1] = t.v1 + vtx_offset;
            indices[idx_offset + i * 4 + 2] = t.v2 + vtx_offset;
            indices[idx_offset + i * 4 + 3] = t.m;
        }

        for (auto& p : mapping) {
            vertices[vtx_offset + p.second] = obj_file.vertices[p.first.v];
        }

        if (has_texcoords) {
            for (auto& p : mapping) {
                texcoords[vtx_offset + p.second] = obj_file.texcoords[p.first.t];
            }
        } else {
            warn("No texture coordinates are present, using default value.");
            std::fill(texcoords.begin() + vtx_offset, texcoords.end(), float2(0.0f));
        }

        // Compute the geometric normals for this mesh
        face_normals.resize(face_normals.size() + triangles.size());
        compute_face_normals(indices, vertices, face_normals, idx_offset);

        if (has_normals) {
            // Set up mesh normals
            for (auto& p : mapping) {
                const auto& n = obj_file.normals[p.first.n];
                normals[vtx_offset + p.second] = n;
            }
        } else {
            // Recompute normals
            warn("No normals are present, recomputing smooth normals from geometry.");
            std::fill(normals.begin() + vtx_offset, normals.end(), float3(0.0f));
            compute_vertex_normals(indices, face_normals, normals, idx_offset);
        }
    }

    // Create triangles for the BVH
    for (size_t i = 0; i < indices.size(); i += 4) {
        auto& v0 = vertices[indices[i + 0]];
        auto& v1 = vertices[indices[i + 1]];
        auto& v2 = vertices[indices[i + 2]];
        tris.emplace_back(v0, v1, v2);
    }

    mtl_offset += obj_file.materials.size();

    // Re-normalize all the values in the OBJ file to handle invalid meshes
    for (auto& n : normals)
        n = normalize(n);

    auto normals_ptr      = reinterpret_cast<Vec3*>(anydsl_alloc(dev, sizeof(Vec3) * normals.size()));
    auto face_normals_ptr = reinterpret_cast<Vec3*>(anydsl_alloc(dev, sizeof(Vec3) * face_normals.size()));
    auto uvs_ptr          = reinterpret_cast<Vec2*>(anydsl_alloc(dev, sizeof(Vec2) * texcoords.size()));
    auto ids_ptr          = reinterpret_cast<int32_t*>(anydsl_alloc(dev, sizeof(int32_t) * indices.size()));

    anydsl_copy(0, normals.data(),      0, dev, normals_ptr,      0, sizeof(Vec3)    * normals.size());
    anydsl_copy(0, face_normals.data(), 0, dev, face_normals_ptr, 0, sizeof(Vec3)    * face_normals.size());
    anydsl_copy(0, texcoords.data(),    0, dev, uvs_ptr,          0, sizeof(Vec2)    * texcoords.size());
    anydsl_copy(0, indices.data(),      0, dev, ids_ptr,          0, sizeof(int32_t) * indices.size());

    int32_t num_tris = indices.size() / 4;
    return TriMesh {
        normals_ptr,
        face_normals_ptr,
        uvs_ptr,
        ids_ptr,
        num_tris
    };
}

static void release_tri_mesh(int32_t dev, TriMesh tri_mesh) {
    anydsl_release(dev, const_cast<Vec3*>(tri_mesh.normals));
    anydsl_release(dev, const_cast<Vec3*>(tri_mesh.face_normals));
    anydsl_release(dev, const_cast<Vec2*>(tri_mesh.uvs));
    anydsl_release(dev, const_cast<int32_t*>(tri_mesh.ids));
}

// Images --------------------------------------------------------------------------

static void read_from_stream(png_structp png_ptr, png_bytep data, png_size_t length) {
    png_voidp a = png_get_io_ptr(png_ptr);
    ((std::istream*)a)->read((char*)data, length);
}

static bool load_png(int32_t dev, std::string file_name, PixelData& pixel_data) {
    std::ifstream file(file_name, std::ifstream::binary);
    if (!file)
        return false;

    // Read signature
    char sig[8];
    file.read(sig, 8);
    if (!png_check_sig((unsigned char*)sig, 8))
        return false;

    png_structp png_ptr = png_create_read_struct(PNG_LIBPNG_VER_STRING, nullptr, nullptr, nullptr);
    if (!png_ptr)
        return false;

    png_infop info_ptr = png_create_info_struct(png_ptr);
    if (!info_ptr) {
        png_destroy_read_struct(&png_ptr, nullptr, nullptr);
        return false;
    }

    if (setjmp(png_jmpbuf(png_ptr))) {
        png_destroy_read_struct(&png_ptr, &info_ptr, nullptr);
        return false;
    }

    png_set_sig_bytes(png_ptr, 8);
    png_set_read_fn(png_ptr, (png_voidp)&file, read_from_stream);
    png_read_info(png_ptr, info_ptr);

    size_t width  = png_get_image_width(png_ptr, info_ptr);
    size_t height = png_get_image_height(png_ptr, info_ptr);

    png_uint_32 color_type = png_get_color_type(png_ptr, info_ptr);
    png_uint_32 bit_depth  = png_get_bit_depth(png_ptr, info_ptr);

    // Expand paletted and grayscale images to RGB
    if (color_type == PNG_COLOR_TYPE_PALETTE) {
        png_set_palette_to_rgb(png_ptr);
    } else if (color_type == PNG_COLOR_TYPE_GRAY ||
               color_type == PNG_COLOR_TYPE_GRAY_ALPHA) {
        png_set_gray_to_rgb(png_ptr);
    }

    // Transform to 8 bit per channel
    if (bit_depth == 16)
        png_set_strip_16(png_ptr);

    // Get alpha channel when there is one
    if (png_get_valid(png_ptr, info_ptr, PNG_INFO_tRNS))
        png_set_tRNS_to_alpha(png_ptr);

    // Otherwise add an opaque alpha channel
    else
        png_set_filler(png_ptr, 0xFF, PNG_FILLER_AFTER);

    std::vector<Color> pixels(width * height);
    std::vector<png_byte> row_bytes(width * 4);
    for (size_t y = 0; y < height; y++) {
        png_read_row(png_ptr, row_bytes.data(), nullptr);
        Color* img_row = pixels.data() + width * y;
        for (size_t x = 0; x < width; x++) {
            img_row[x].r = row_bytes[x * 4 + 0] / 255.0f;
            img_row[x].g = row_bytes[x * 4 + 1] / 255.0f;
            img_row[x].b = row_bytes[x * 4 + 2] / 255.0f;
        }
    }

    png_destroy_read_struct(&png_ptr, &info_ptr, nullptr);

    auto pixels_ptr = reinterpret_cast<Color*>(anydsl_alloc(dev, sizeof(Color) * width * height));
    anydsl_copy(0, pixels.data(), 0, dev, pixels_ptr, 0, sizeof(Color) * width * height);

    pixel_data.pixels = pixels_ptr;
    pixel_data.width  = static_cast<int>(width);
    pixel_data.height = static_cast<int>(height);
    return true;
}

static PixelData load_image(int32_t dev, std::string file_name) {
    PixelData image;
    if (!load_png(dev, file_name, image))
        error("Cannot load file '", file_name, "'.");
    return image;
}

static void release_image(int32_t dev, PixelData pixel_data) {
    anydsl_release(dev, const_cast<Color*>(pixel_data.pixels));
}

// BVH -----------------------------------------------------------------------------

class Bvh8Tri4Adapter {
    struct CostFn {
        static float leaf_cost(int count, float area) {
            return ((count - 1) / 4 + 1) * area;
        }
        static float traversal_cost(float area) {
            return area * 0.5f;
        }
    };

    struct StackElem {
        int parent, child;
        StackElem() {}
        StackElem(int parent, int child) : parent(parent), child(child) {}
    };

    using BvhBuilder = SplitBvhBuilder<8, CostFn>;
    using Adapter    = Bvh8Tri4Adapter;

    std::vector<Bvh8Node>& nodes_;
    std::vector<Bvh4Tri>&  tris_;
    Stack<StackElem>       stack_;
    BvhBuilder             builder_;

    const Tri* in_tris;

public:
    Bvh8Tri4Adapter(std::vector<Bvh8Node>& nodes, std::vector<Bvh4Tri>& tris)
        : nodes_(nodes), tris_(tris)
    {}

    void build(const std::vector<Tri>& tris) {
        in_tris = tris.data();
        builder_.build(tris, NodeWriter(*this), LeafWriter(*this), 4);
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
        void operator() (const BBox& parent_bb, int count, BBoxFn bboxes) {
            auto& nodes = adapter.nodes_;
            auto& stack = adapter.stack_;

            int i = nodes.size();
            nodes.emplace_back();

            if (!stack.is_empty()) {
                StackElem elem = stack.pop();
                nodes[elem.parent].child[elem.child] = i + 1;
            }

            assert(count >= 2 && count <= 8);

            for (int j = count - 1; j >= 0; j--) {
                const BBox& bbox = bboxes(j);
                nodes[i].bounds[0][j] = bbox.min.x;
                nodes[i].bounds[2][j] = bbox.min.y;
                nodes[i].bounds[4][j] = bbox.min.z;

                nodes[i].bounds[1][j] = bbox.max.x;
                nodes[i].bounds[3][j] = bbox.max.y;
                nodes[i].bounds[5][j] = bbox.max.z;

                stack.push(i, j);
            }

            for (int j = 3; j >= count; j--) {
                nodes[i].bounds[0][j] = 0.0f;
                nodes[i].bounds[2][j] = 0.0f;
                nodes[i].bounds[4][j] = 0.0f;

                nodes[i].bounds[1][j] = -0.0f;
                nodes[i].bounds[3][j] = -0.0f;
                nodes[i].bounds[5][j] = -0.0f;

                nodes[i].child[j] = 0;
            }
        }
    };

    struct LeafWriter {
        Adapter& adapter;

        LeafWriter(Adapter& adapter)
            : adapter(adapter)
        {}

        static void fill_dummy_parent(Bvh8Node& node, const BBox& leaf_bb, int index) {
            node.child[0] = index;

            node.bounds[0][0] = leaf_bb.min.x;
            node.bounds[2][0] = leaf_bb.min.y;
            node.bounds[4][0] = leaf_bb.min.z;

            node.bounds[1][0] = leaf_bb.max.x;
            node.bounds[3][0] = leaf_bb.max.y;
            node.bounds[5][0] = leaf_bb.max.z;

            for (int i = 1; i < 8; ++i) {
                node.child[i] = 0;

                node.bounds[0][0] = 0.0f;
                node.bounds[2][0] = 0.0f;
                node.bounds[4][0] = 0.0f;

                node.bounds[1][0] = -0.0f;
                node.bounds[3][0] = -0.0f;
                node.bounds[5][0] = -0.0f;
            }
        }

        template <typename RefFn>
        void operator() (const BBox& leaf_bb, int ref_count, RefFn refs) {
            auto& nodes = adapter.nodes_;
            auto& stack = adapter.stack_;
            auto& tris = adapter.tris_;
            auto  in_tris = adapter.in_tris;

            if (stack.is_empty()) {
                nodes.emplace_back();
                fill_dummy_parent(nodes.back(), leaf_bb, ~tris.size());
            } else {
                const StackElem& elem = stack.pop();
                nodes[elem.parent].child[elem.child] = ~tris.size();
            }

            // Group triangles by packets of 4
            for (int i = 0; i < ref_count; i += 4) {
                const int c = i + 4 <= ref_count ? 4 : ref_count - i;
                Bvh4Tri bvh4tri;
                memset(&bvh4tri, 0, sizeof(Bvh4Tri));
                for (int j = 0; j < c; j++) {
                    const int id = refs(i + j);
                    const Tri& tri = in_tris[id];
                    const float3 e1 = tri.v0 - tri.v1;
                    const float3 e2 = tri.v2 - tri.v0;
                    const float3 n = cross(e1, e2);
                    bvh4tri.v0[0][j] = tri.v0.x;
                    bvh4tri.v0[1][j] = tri.v0.y;
                    bvh4tri.v0[2][j] = tri.v0.z;

                    bvh4tri.e1[0][j] = e1.x;
                    bvh4tri.e1[1][j] = e1.y;
                    bvh4tri.e1[2][j] = e1.z;

                    bvh4tri.e2[0][j] = e2.x;
                    bvh4tri.e2[1][j] = e2.y;
                    bvh4tri.e2[2][j] = e2.z;

                    bvh4tri.n[0][j] = n.x;
                    bvh4tri.n[1][j] = n.y;
                    bvh4tri.n[2][j] = n.z;

                    bvh4tri.id[j] = id;
                }

                for (int j = c; j < 4; j++)
                    bvh4tri.id[j] = 0xFFFFFFFF;

                tris.emplace_back(bvh4tri);
            }
            assert(!tris.empty());
            tris.back().id[3] |= 0x80000000;
        }
    };
};

template <typename BvhType>
struct BvhTraits {};

template <>
struct BvhTraits<Bvh8Tri4> {
    using Node = Bvh8Node;
    using Tri  = Bvh4Tri;
    using Adapter = Bvh8Tri4Adapter;
};

template <typename BvhType>
BvhType build_bvh(int32_t dev, const std::vector<Tri>& in_tris) {
    using Traits  = BvhTraits<BvhType>;
    using Adapter = typename Traits::Adapter;
    using Node = typename Traits::Node;
    using Tri  = typename Traits::Tri;

    std::vector<Node> nodes;
    std::vector<Tri>  tris;
    Adapter adapter(nodes, tris);
    adapter.build(in_tris);
    info("BVH built with ", nodes.size(), " node(s), ", tris.size(), " triangle(s)");

    auto nodes_ptr = reinterpret_cast<Node*>(anydsl_alloc(dev, sizeof(Node) * nodes.size()));
    auto tris_ptr  = reinterpret_cast<Tri*> (anydsl_alloc(dev, sizeof(Tri)  * tris.size()));
    anydsl_copy(0, nodes.data(), 0, dev, nodes_ptr, 0, sizeof(Node) * nodes.size());
    anydsl_copy(0, tris.data(),  0, dev, tris_ptr,  0, sizeof(Tri)  * tris.size());

    return BvhType {
        nodes_ptr,
        tris_ptr
    };
}

// Interface -----------------------------------------------------------------------

template <typename BvhType>
class Interface {
public:
    Interface(int32_t dev, size_t width, size_t height)
        : dev_(dev), width_(width), height_(height)
    {}

    ~Interface() {
        if (film_data_)
            anydsl_release(dev_, film_data_->pixels);
        for (auto& tri_mesh : tri_meshes_)
            release_tri_mesh(dev_, tri_mesh.second);
        for (auto& image : images_)
            release_image(dev_, image.second);
    }

    PixelData image(const char* file) {
        auto it = images_.find(file);
        if (it != images_.end())
            return it->second;
        return images_[file] = load_image(dev_, file);
    }

    TriMesh tri_mesh(const char* file) {
        auto it = tri_meshes_.find(file);
        if (it != tri_meshes_.end())
            return it->second;
        return tri_meshes_[file] = load_tri_mesh(dev_, file, tris_);
    }

    PixelData film_data() {
        if (film_data_)
            return *film_data_;
        auto pixels = anydsl_alloc(dev_, sizeof(Color) * width_ * height_);
        film_data_.reset(new PixelData {
            reinterpret_cast<Color*>(pixels),
            static_cast<int>(width_),
            static_cast<int>(height_)
        });
        return *film_data_;
    }

    BvhType bvh() {
        if (bvh_)
            return *bvh_;
        bvh_.reset(new BvhType(build_bvh<BvhType>(dev_, tris_)));
        std::vector<Tri>().swap(tris_);
        return *bvh_;
    }

private:
    std::unordered_map<std::string, PixelData> images_;
    std::unordered_map<std::string, TriMesh>   tri_meshes_;
    std::unique_ptr<PixelData> film_data_;
    std::unique_ptr<BvhType> bvh_;
    std::vector<Tri> tris_;
    size_t width_, height_;
    int32_t dev_;
};

// CPU Interface -------------------------------------------------------------------

static std::unique_ptr<Interface<Bvh8Tri4>> cpu_interface;

void setup_cpu_interface(size_t width, size_t height) {
    cpu_interface.reset(new Interface<Bvh8Tri4>(0, width, height));
}

Color* get_cpu_pixels() {
    return cpu_interface->film_data().pixels;
}

void cleanup_cpu_interface() {
    cpu_interface.reset();
}

extern "C" void rodent_cpu_get_bvh8_tri4(Bvh8Tri4* bvh) {
    *bvh = cpu_interface->bvh();
}

extern "C" void rodent_cpu_get_film_data(PixelData* film_data) {
    *film_data = cpu_interface->film_data();
}

extern "C" void rodent_cpu_load_tri_mesh(const char* file, TriMesh* tri_mesh) {
    *tri_mesh = cpu_interface->tri_mesh(file);
}

extern "C" void rodent_cpu_load_pixel_data(const char* file, PixelData* pixel_data) {
    *pixel_data = cpu_interface->image(file);
}

static void cpu_get_ray_stream(RayStream& rays, float* ptr, size_t capacity) {
    rays.id = (int*)ptr + 0 * capacity;
    rays.org_x = ptr + 1 * capacity;
    rays.org_y = ptr + 2 * capacity;
    rays.org_z = ptr + 3 * capacity;
    rays.dir_x = ptr + 4 * capacity;
    rays.dir_y = ptr + 5 * capacity;
    rays.dir_z = ptr + 6 * capacity;
    rays.tmin  = ptr + 7 * capacity;
    rays.tmax  = ptr + 8 * capacity;
}

extern "C" void rodent_cpu_get_primary_stream(PrimaryStream* primary, int size) {
    static thread_local anydsl::Array<float> array;
    size_t capacity = (size & ~((1 << 5) - 1)) + 32; // round to 32
    if (array.size() < capacity)
        array = std::move(anydsl::Array<float>(capacity * 21));

    primary->size = 0;
    cpu_get_ray_stream(primary->rays, array.data(), capacity);
    primary->shader_id = (int*)array.data() +  9 * capacity;
    primary->geom_id   = (int*)array.data() + 10 * capacity;
    primary->prim_id   = (int*)array.data() + 11 * capacity;
    primary->t         = array.data() + 12 * capacity;
    primary->u         = array.data() + 13 * capacity;
    primary->v         = array.data() + 14 * capacity;
    primary->rnd       = (unsigned int*)array.data() + 15 * capacity;
    primary->mis       = array.data() + 16 * capacity;
    primary->contrib_r = array.data() + 17 * capacity;
    primary->contrib_g = array.data() + 18 * capacity;
    primary->contrib_b = array.data() + 19 * capacity;
    primary->depth     = (int*)array.data() + 20 * capacity;
}

extern "C" void rodent_cpu_get_secondary_stream(SecondaryStream* secondary, int size) {
    static thread_local anydsl::Array<float> array;
    size_t capacity = (size & ~((1 << 5) - 1)) + 32; // round to 32
    if (array.size() < capacity)
        array = std::move(anydsl::Array<float>(capacity * 13));

    secondary->size = 0;
    cpu_get_ray_stream(secondary->rays, array.data(), capacity);
    secondary->prim_id   = (int*)array.data() + 9 * capacity;
    secondary->color_r   = array.data() + 10 * capacity;
    secondary->color_g   = array.data() + 11 * capacity;
    secondary->color_b   = array.data() + 12 * capacity;
}

// GPU Interface -------------------------------------------------------------------

// TODO
