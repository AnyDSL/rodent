#include <fstream>
#include <iostream> 
#include <cstring>
#include <cstdlib>
#include <cctype>

#include "common.h"
#include "obj.h"

namespace obj {

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

inline void remove_eol(char* ptr) {
    int i = 0;
    while (ptr[i]) i++;
    i--;
    while (i > 0 && std::isspace(ptr[i])) {
        ptr[i] = '\0';
        i--;
    }
}

inline char* strip_text(char* ptr) {
    while (*ptr && !std::isspace(*ptr)) { ptr++; }
    return ptr;
}

inline char* strip_spaces(char* ptr) {
    while (std::isspace(*ptr)) { ptr++; }
    return ptr;
}

inline bool read_index(char** ptr, obj::Index& idx) {
    char* base = *ptr;

    // Detect end of line (negative indices are supported) 
    base = strip_spaces(base);
    if (!std::isdigit(*base) && *base != '-') return false;

    idx.v = 0;
    idx.t = 0;
    idx.n = 0;

    idx.v = std::strtol(base, &base, 10);

    base = strip_spaces(base);

    if (*base == '/') {
        base++;

        // Handle the case when there is no texture coordinate
        if (*base != '/') {
            idx.t = std::strtol(base, &base, 10);
        }

        base = strip_spaces(base);

        if (*base == '/') {
            base++;
            idx.n = std::strtol(base, &base, 10);
        }
    }

    *ptr = base;

    return true;
}

static bool parse_obj(std::istream& stream, obj::File& file) {
    // Add an empty object to the scene
    int cur_object = 0;
    file.objects.emplace_back();

    // Add an empty group to this object
    int cur_group = 0;
    file.objects[0].groups.emplace_back();

    // Add an empty material to the scene
    int cur_mtl = 0;
    file.materials.emplace_back("");

    // Add dummy vertex, normal, and texcoord
    file.vertices.emplace_back();
    file.normals.emplace_back();
    file.texcoords.emplace_back();

    int err_count = 0, cur_line = 0;
    const int max_line = 1024;
    char line[max_line];

    while (stream.getline(line, max_line)) {
        cur_line++;

        // Strip spaces
        char* ptr = strip_spaces(line);

        // Skip comments and empty lines
        if (*ptr == '\0' || *ptr == '#')
            continue;

        remove_eol(ptr);

        // Test each command in turn, the most frequent first
        if (*ptr == 'v') {
            switch (ptr[1]) {
                case ' ':
                case '\t':
                    {
                        float3 v;
                        v.x = std::strtof(ptr + 1, &ptr);
                        v.y = std::strtof(ptr, &ptr);
                        v.z = std::strtof(ptr, &ptr);
                        file.vertices.push_back(v);
                    }
                    break;
                case 'n':
                    {
                        float3 n;
                        n.x = std::strtof(ptr + 2, &ptr);
                        n.y = std::strtof(ptr, &ptr);
                        n.z = std::strtof(ptr, &ptr);
                        file.normals.push_back(n);
                    }
                    break;
                case 't':
                    {
                        float2 t;
                        t.x = std::strtof(ptr + 2, &ptr);
                        t.y = std::strtof(ptr, &ptr);
                        file.texcoords.push_back(t);
                    }
                    break;
                default:
                    error("Invalid vertex (line ", cur_line, ").");
                    err_count++;
                    break;
            }
        } else if (*ptr == 'f' && std::isspace(ptr[1])) {
            obj::Face f;

            f.material = cur_mtl;

            bool valid = true;
            ptr += 2;
            while (valid) {
                obj::Index index;
                valid = read_index(&ptr, index);
                if (valid)
                    f.indices.push_back(index);
            }

            if (f.indices.size() < 3) {
                error("Invalid face (line ", cur_line, ").");
                err_count++;
            } else {
                // Convert relative indices to absolute
                for (size_t i = 0; i < f.indices.size(); i++) {
                    f.indices[i].v = (f.indices[i].v < 0) ? file.vertices.size()  + f.indices[i].v : f.indices[i].v;
                    f.indices[i].t = (f.indices[i].t < 0) ? file.texcoords.size() + f.indices[i].t : f.indices[i].t;
                    f.indices[i].n = (f.indices[i].n < 0) ? file.normals.size()   + f.indices[i].n : f.indices[i].n;
                }

                // Check if the indices are valid or not
                valid = true;
                for (size_t i = 0; i < f.indices.size(); i++) {
                    if (f.indices[i].v <= 0 || f.indices[i].t < 0 || f.indices[i].n < 0) {
                        valid = false;
                        break;
                    }
                }

                if (valid) {
                    file.objects[cur_object].groups[cur_group].faces.push_back(f);
                } else {
                    error("Invalid indices in face definition (line ", cur_line, ").");
                    err_count++;
                }
            }
        } else if (*ptr == 'g' && std::isspace(ptr[1])) {
            file.objects[cur_object].groups.emplace_back();
            cur_group++;
        } else if (*ptr == 'o' && std::isspace(ptr[1])) {
            file.objects.emplace_back();
            cur_object++;

            file.objects[cur_object].groups.emplace_back();
            cur_group = 0;
        } else if (!std::strncmp(ptr, "usemtl", 6) && std::isspace(ptr[6])) {
            ptr += 6;

            ptr = strip_spaces(ptr);
            char* base = ptr;
            ptr = strip_text(ptr);

            const std::string mtl_name(base, ptr);

            cur_mtl = std::find(file.materials.begin(), file.materials.end(), mtl_name) - file.materials.begin();
            if (cur_mtl == (int)file.materials.size()) {
                file.materials.push_back(mtl_name);
            }
        } else if (!std::strncmp(ptr, "mtllib", 6) && std::isspace(ptr[6])) {
            ptr += 6;

            ptr = strip_spaces(ptr);
            char* base = ptr;
            ptr = strip_text(ptr);

            const std::string lib_name(base, ptr);

            file.mtl_libs.push_back(lib_name);
        } else if (*ptr == 's' && std::isspace(ptr[1])) {
            // Ignore smooth commands
        } else {
            error("Unknown command '", ptr, "' (line ", cur_line, ").");
            err_count++;
        }
    }

    return (err_count == 0);
}

static bool parse_mtl(std::istream& stream, obj::MaterialLib& mtl_lib) {
    const int max_line = 1024;
    int err_count = 0, cur_line = 0;
    char line[max_line];

    std::string mtl_name;
    auto current_material = [&] () -> obj::Material& {
        return mtl_lib[mtl_name];
    };

    while (stream.getline(line, max_line)) {
        cur_line++;

        // Strip spaces
        char* ptr = strip_spaces(line);

        // Skip comments and empty lines
        if (*ptr == '\0' || *ptr == '#')
            continue;

        remove_eol(ptr);

        if (!std::strncmp(ptr, "newmtl", 6) && std::isspace(ptr[6])) {
            ptr = strip_spaces(ptr + 7);
            char* base = ptr;
            ptr = strip_text(ptr);

            mtl_name = std::string(base, ptr);
            if (mtl_lib.find(mtl_name) != mtl_lib.end()) {
                error("Material redefinition for '", mtl_name, "' (line ", cur_line, ").");
                err_count++;
            }
        } else if (ptr[0] == 'K') {
            if (ptr[1] == 'a' && std::isspace(ptr[2])) {
                auto& mat = current_material();
                mat.ka[0] = std::strtof(ptr + 3, &ptr);
                mat.ka[1] = std::strtof(ptr, &ptr);
                mat.ka[2] = std::strtof(ptr, &ptr);
            } else if (ptr[1] == 'd' && std::isspace(ptr[2])) {
                auto& mat = current_material();
                mat.kd[0] = std::strtof(ptr + 3, &ptr);
                mat.kd[1] = std::strtof(ptr, &ptr);
                mat.kd[2] = std::strtof(ptr, &ptr);
            } else if (ptr[1] == 's' && std::isspace(ptr[2])) {
                auto& mat = current_material();
                mat.ks[0] = std::strtof(ptr + 3, &ptr);
                mat.ks[1] = std::strtof(ptr, &ptr);
                mat.ks[2] = std::strtof(ptr, &ptr);
            } else if (ptr[1] == 'e' && std::isspace(ptr[2])) {
                auto& mat = current_material();
                mat.ke[0] = std::strtof(ptr + 3, &ptr);
                mat.ke[1] = std::strtof(ptr, &ptr);
                mat.ke[2] = std::strtof(ptr, &ptr);
            } else {
                error("Invalid command '", ptr, "' (line ", cur_line , ").");
                err_count++;
            }
        } else if (ptr[0] == 'N') {
            if (ptr[1] == 's' && std::isspace(ptr[2])) {
                auto& mat = current_material();
                mat.ns = std::strtof(ptr + 3, &ptr);
            } else if (ptr[1] == 'i' && std::isspace(ptr[2])) {
                auto& mat = current_material();
                mat.ni = std::strtof(ptr + 3, &ptr);
            } else {
                error("Invalid command '", ptr, "' (line ", cur_line , ").");
                err_count++;
            }
        } else if (ptr[0] == 'T') {
            if (ptr[1] == 'f' && std::isspace(ptr[2])) {
                auto& mat = current_material();
                mat.tf.x = std::strtof(ptr + 3, &ptr);
                mat.tf.y = std::strtof(ptr, &ptr);
                mat.tf.z = std::strtof(ptr, &ptr);
            } else if (ptr[1] == 'r' && std::isspace(ptr[2])) {
                auto& mat = current_material();
                mat.tr = std::strtof(ptr + 3, &ptr);
            } else {
                error("Invalid command '", ptr, "' (line ", cur_line , ").");
                err_count++;
            }
        } else if (ptr[0] == 'd' && std::isspace(ptr[1])) {
            auto& mat = current_material();
            mat.d = std::strtof(ptr + 2, &ptr);
        } else if (!std::strncmp(ptr, "illum", 5) && std::isspace(ptr[5])) {
            auto& mat = current_material();
            mat.illum = std::strtof(ptr + 6, &ptr);
        } else if (!std::strncmp(ptr, "map_Ka", 6) && std::isspace(ptr[6])) {
            auto& mat = current_material();
            mat.map_ka = std::string(strip_spaces(ptr + 7));
        } else if (!std::strncmp(ptr, "map_Kd", 6) && std::isspace(ptr[6])) {
            auto& mat = current_material();
            mat.map_kd = std::string(strip_spaces(ptr + 7));
        } else if (!std::strncmp(ptr, "map_Ks", 6) && std::isspace(ptr[6])) {
            auto& mat = current_material();
            mat.map_ks = std::string(strip_spaces(ptr + 7));
        } else if (!std::strncmp(ptr, "map_Ke", 6) && std::isspace(ptr[6])) {
            auto& mat = current_material();
            mat.map_ke = std::string(strip_spaces(ptr + 7));
        } else if (!std::strncmp(ptr, "map_bump", 8) && std::isspace(ptr[8])) {
            auto& mat = current_material();
            mat.map_bump = std::string(strip_spaces(ptr + 9));
        } else if (!std::strncmp(ptr, "bump", 4) && std::isspace(ptr[4])) {
            auto& mat = current_material();
            mat.map_bump = std::string(strip_spaces(ptr + 5));
        } else if (!std::strncmp(ptr, "map_d", 5) && std::isspace(ptr[5])) {
            auto& mat = current_material();
            mat.map_d = std::string(strip_spaces(ptr + 6));
        } else {
            warn("Unknown command '", ptr, "' (line ", cur_line , ").");
        }
    }

    return (err_count == 0);
}

bool load_obj(const FilePath& path, obj::File& obj_file) {
    // Parse the OBJ file
    std::ifstream stream(path);
    return stream && parse_obj(stream, obj_file);
}

bool load_mtl(const FilePath& path, obj::MaterialLib& mtl_lib) {
    // Parse the MTL file
    std::ifstream stream(path);
    return stream && parse_mtl(stream, mtl_lib);
}

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

TriMesh compute_tri_mesh(const File& obj_file, const MaterialLib& /*mtl_lib*/, size_t mtl_offset) {
    TriMesh tri_mesh;

    for (auto& obj: obj_file.objects) {
        // Convert the faces to triangles & build the new list of indices
        std::vector<TriIdx> triangles;
        std::unordered_map<obj::Index, size_t, HashIndex, CompareIndex> mapping;

        bool has_normals = false;
        bool has_texcoords = false;
        for (auto& group : obj.groups) {
            for (auto& face : group.faces) {
                for (size_t i = 0; i < face.indices.size(); i++) {
                    auto map = mapping.find(face.indices[i]);
                    if (map == mapping.end()) {
                        has_normals |= (face.indices[i].n != 0);
                        has_texcoords |= (face.indices[i].t != 0);

                        mapping.insert(std::make_pair(face.indices[i], mapping.size()));
                    }
                }

                auto v0 = mapping[face.indices[0]];
                auto prev = mapping[face.indices[1]];

                for (size_t i = 1; i < face.indices.size() - 1; i++) {
                    auto next = mapping[face.indices[i + 1]];
                    triangles.emplace_back(v0, prev, next, face.material + mtl_offset);
                    prev = next;
                }
            }
        }

        if (triangles.size() == 0) continue;

        // Add this object to the mesh
        auto vtx_offset = tri_mesh.vertices.size();
        auto idx_offset = tri_mesh.indices.size();
        tri_mesh.indices.resize(idx_offset + 4 * triangles.size());
        tri_mesh.vertices.resize(vtx_offset + mapping.size());
        tri_mesh.texcoords.resize(vtx_offset + mapping.size());
        tri_mesh.normals.resize(vtx_offset + mapping.size());

        for (size_t i = 0, n = triangles.size(); i < n; i++) {
            auto& t = triangles[i];
            tri_mesh.indices[idx_offset + i * 4 + 0] = t.v0 + vtx_offset;
            tri_mesh.indices[idx_offset + i * 4 + 1] = t.v1 + vtx_offset;
            tri_mesh.indices[idx_offset + i * 4 + 2] = t.v2 + vtx_offset;
            tri_mesh.indices[idx_offset + i * 4 + 3] = t.m;
        }

        for (auto& p : mapping) {
            tri_mesh.vertices[vtx_offset + p.second] = obj_file.vertices[p.first.v];
        }

        if (has_texcoords) {
            for (auto& p : mapping) {
                tri_mesh.texcoords[vtx_offset + p.second] = obj_file.texcoords[p.first.t];
            }
        } else {
            warn("No texture coordinates are present, using default value.");
            std::fill(tri_mesh.texcoords.begin() + vtx_offset, tri_mesh.texcoords.end(), float2(0.0f));
        }

        // Compute the geometric normals for this mesh
        tri_mesh.face_normals.resize(tri_mesh.face_normals.size() + triangles.size());
        compute_face_normals(tri_mesh.indices, tri_mesh.vertices, tri_mesh.face_normals, idx_offset);

        if (has_normals) {
            // Set up mesh normals
            for (auto& p : mapping) {
                const auto& n = obj_file.normals[p.first.n];
                tri_mesh.normals[vtx_offset + p.second] = n;
            }
        } else {
            // Recompute normals
            warn("No normals are present, recomputing smooth normals from geometry.");
            std::fill(tri_mesh.normals.begin() + vtx_offset, tri_mesh.normals.end(), float3(0.0f));
            compute_vertex_normals(tri_mesh.indices, tri_mesh.face_normals, tri_mesh.normals, idx_offset);
        }
    }

    // Re-normalize all the values in the OBJ file to handle invalid meshes
    bool fixed_normals = false;
    for (auto& n : tri_mesh.normals) {
        auto len2 = lensqr(n);
        if (len2 <= std::numeric_limits<float>::epsilon() || std::isnan(len2)) {
            fixed_normals = true;
            n = float3(0.0f, 1.0f, 0.0f);
        } else
            n = n * (1.0f / std::sqrt(len2));
    }

    if (fixed_normals)
        warn("Some normals were incorrect and thus had to be replaced with arbitrary values.");

    return tri_mesh;
}

} // namespace obj
