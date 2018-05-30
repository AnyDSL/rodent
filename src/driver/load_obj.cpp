#include <fstream>
#include <iostream> 
#include <cstring>
#include <cstdlib>
#include <cctype>

#include "common.h"
#include "load_obj.h"

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

            f.index_count = 0;
            f.material = cur_mtl;

            bool valid = true;
            ptr += 2;
            while(f.index_count < obj::Face::max_indices) {
                obj::Index index;
                valid = read_index(&ptr, index);

                if (valid) {
                    f.indices[f.index_count++] = index;
                } else {
                    break;
                }
            }

            if (f.index_count < 3) {
                error("Invalid face (line ", cur_line, ").");
                err_count++;
            } else {
                // Convert relative indices to absolute
                for (int i = 0; i < f.index_count; i++) {
                    f.indices[i].v = (f.indices[i].v < 0) ? file.vertices.size()  + f.indices[i].v : f.indices[i].v;
                    f.indices[i].t = (f.indices[i].t < 0) ? file.texcoords.size() + f.indices[i].t : f.indices[i].t;
                    f.indices[i].n = (f.indices[i].n < 0) ? file.normals.size()   + f.indices[i].n : f.indices[i].n;
                }

                // Check if the indices are valid or not
                valid = true;
                for (int i = 0; i < f.index_count; i++) {
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
