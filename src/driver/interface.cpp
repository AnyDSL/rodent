#include <unordered_map>
#include <memory>
#include <string>
#include <fstream>

#include <anydsl_runtime.hpp>
#include <png.h>

#include "interface.h"
#include "load_obj.h"

// Triangle Mesh -------------------------------------------------------------------

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

static TriMesh load_tri_mesh(int32_t dev, std::string file_name) {
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
            const auto& v = obj_file.vertices[p.first.v];
            vertices[vtx_offset + p.second].x = v.x;
            vertices[vtx_offset + p.second].y = v.y;
            vertices[vtx_offset + p.second].z = v.z;
        }

        if (has_texcoords) {
            for (auto& p : mapping) {
                const auto& t = obj_file.texcoords[p.first.t];
                texcoords[vtx_offset + p.second] = t;
            }
        } else
            std::fill(texcoords.begin() + vtx_offset, texcoords.end(), float2(0.0f));

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

    std::vector<Color> pixels;
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

// Interface -----------------------------------------------------------------------

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
        return tri_meshes_[file] = load_tri_mesh(dev_, file);
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
    }

    Bvh8Tri4 bvh() {
        if (bvh_)
            return *bvh_;
        bvh_ = build_bvh(dev_);
    }

private:
    std::unordered_map<std::string, PixelData> images_;
    std::unordered_map<std::string, TriMesh>   tri_meshes_;
    std::unique_ptr<PixelData> film_data_;
    std::unique_ptr<Bvh8Tri4>  bvh_;
    size_t width_, height_;
    int32_t dev_;
};

// CPU Interface -------------------------------------------------------------------

static std::unique_ptr<Interface> cpu_interface;

void setup_cpu_interface(size_t width, size_t height) {
    cpu_interface.reset(new Interface(0, width, height));
}

extern "C" Bvh8Tri4 rodent_cpu_get_bvh8_tri4() {
    return cpu_interface->bvh();
}

extern "C" PixelData rodent_cpu_get_film_data() {
    return cpu_interface->film_data();
}

extern "C" TriMesh rodent_cpu_load_tri_mesh(const char* file) {
    return cpu_interface->tri_mesh(file);
}

extern "C" PixelData rodent_cpu_load_pixel_data(const char* file) {
    return cpu_interface->image(file);
}

// GPU Interface -------------------------------------------------------------------

// TODO
