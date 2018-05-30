#ifndef LOAD_OBJ_H
#define LOAD_OBJ_H

#include <vector>
#include <string>
#include <unordered_map>

#include "float3.h"
#include "color.h"
#include "file_path.h"

namespace obj {

/// A reference to a vertex/normal/texture coord. of the model.
struct Index {
    int v, n, t;        ///< Vertex, normal and texture indices (0 means not present)
};

struct Face {
    static constexpr int max_indices = 8;
    Index indices[max_indices];             ///< Indices of the face
    int index_count;                        ///< Number of indices in the face
    int material;                           ///< Index into the material names of the model
};

/// A group of faces in the model.
struct Group {
    std::vector<Face> faces;
};

/// A object in the model, made of several groups.
struct Object {
    std::vector<Group> groups;
};

struct Material {
    rgb ka;                     ///< Ambient term
    rgb kd;                     ///< Diffuse term
    rgb ks;                     ///< Specular term
    rgb ke;                     ///< Emitting term
    float ns;                   ///< Specular index
    float ni;                   ///< Medium index
    rgb tf;                     ///< Transmittance
    float tr;                   ///< Transparency
    float d;                    ///< Dissolve factor
    int illum;                  ///< Illumination model
    std::string map_ka;         ///< Ambient texture
    std::string map_kd;         ///< Diffuse texture
    std::string map_ks;         ///< Specular texture
    std::string map_ke;         ///< Emitting texture
    std::string map_bump;       ///< Bump mapping texture
    std::string map_d;          ///< Dissolve texture
};

struct File {
    std::vector<Object>      objects;       ///< List of objects in the model
    std::vector<float3>      vertices;      ///< List of vertices in the model
    std::vector<float3>      normals;       ///< List of normals in the model
    std::vector<float2>      texcoords;     ///< List of texture coordinates in the model
    std::vector<std::string> materials;     ///< List of material names referenced in the model
    std::vector<std::string> mtl_libs;      ///< List of MTL files referenced in the model
};

typedef std::unordered_map<std::string, Material> MaterialLib;

} // namespace obj

/// Loads an OBJ model from a file.
bool load_obj(const FilePath&, obj::File&);
/// Loads an MTL file from a file.
bool load_mtl(const FilePath&, obj::MaterialLib&);

#endif // LOAD_OBJ_H
