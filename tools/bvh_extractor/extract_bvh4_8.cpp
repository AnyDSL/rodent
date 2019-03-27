#include <fstream>
#include <limits>

#include "traversal.h"
#include "load_bvh.h"
#include "driver/embree_bvh.h"
#include "driver/obj.h"

template <size_t N, typename BvhNode, typename BvhTri>
void write_embree_bvh(std::ofstream& out, const std::vector<BvhNode>& nodes, const std::vector<BvhTri>& tris) {
    uint64_t offset = sizeof(uint32_t) * 3 +
        sizeof(BvhNode) * nodes.size() +
        sizeof(BvhTri)  * tris.size();
    uint32_t block_type = uint32_t(N == 4 ? BvhType::BVH4_TRI4 : BvhType::BVH8_TRI4);
    uint32_t num_nodes = nodes.size();
    uint32_t num_tris  = tris.size();

    out.write((char*)&offset,      sizeof(uint64_t));
    out.write((char*)&block_type,  sizeof(uint32_t));
    out.write((char*)&num_nodes,   sizeof(uint32_t));
    out.write((char*)&num_tris,    sizeof(uint32_t));
    out.write((char*)nodes.data(), sizeof(BvhNode) * nodes.size());
    out.write((char*)tris.data(),  sizeof(BvhTri)  * tris.size());
}

size_t build_bvh4(std::ofstream& out, const obj::TriMesh& tri_mesh) {
    std::vector<Node4> nodes;
    std::vector<Tri4> tris;
    if (!build_embree_bvh<4>(tri_mesh, nodes, tris))
        return 0;
    write_embree_bvh<4>(out, nodes, tris);
    return nodes.size();
}

size_t build_bvh8(std::ofstream& out, const obj::TriMesh& tri_mesh) {
    std::vector<Node8> nodes;
    std::vector<Tri4> tris;
    if (!build_embree_bvh<8>(tri_mesh, nodes, tris))
        return 0;
    write_embree_bvh<8>(out, nodes, tris);
    return nodes.size();
}
