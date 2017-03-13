#include <embree2/rtcore.h>

#include <config.h>
#include <kernels/bvh/bvh.h>
#include <kernels/geometry/trianglev.h>

#include <fstream>

#include "../../src/traversal/frontend/traversal.h"
#include "tri.h"

using namespace embree;

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

void extract_bvh4_leaf(BVH4::NodeRef leaf, std::vector<Bvh4Tri>& new_tris) {
    size_t num; 
    auto tris = (const Triangle4v*)leaf.leaf(num);
    for (size_t i = 0; i < num; i++) {
        Bvh4Tri new_tri;
        
        size_t j = 0;
        for (; j < tris[i].size(); j++) {
            new_tri.v0[0][j] = tris[i].v0.x[j];
            new_tri.v0[1][j] = tris[i].v0.y[j];
            new_tri.v0[2][j] = tris[i].v0.z[j];
            new_tri.e1[0][j] = tris[i].v0.x[j] - tris[i].v1.x[j];
            new_tri.e1[1][j] = tris[i].v0.y[j] - tris[i].v1.y[j];
            new_tri.e1[2][j] = tris[i].v0.z[j] - tris[i].v1.z[j];
            new_tri.e2[0][j] = tris[i].v2.x[j] - tris[i].v0.x[j];
            new_tri.e2[1][j] = tris[i].v2.y[j] - tris[i].v0.y[j];
            new_tri.e2[2][j] = tris[i].v2.z[j] - tris[i].v0.z[j];
            new_tri.n[0][j] = new_tri.e1[1][j] * new_tri.e2[2][j] - new_tri.e1[2][j] * new_tri.e2[1][j];
            new_tri.n[1][j] = new_tri.e1[2][j] * new_tri.e2[0][j] - new_tri.e1[0][j] * new_tri.e2[2][j];
            new_tri.n[2][j] = new_tri.e1[0][j] * new_tri.e2[1][j] - new_tri.e1[1][j] * new_tri.e2[0][j];
            new_tri.id[j] = tris[i].primID(j);
        }
        for (; j < 4; j++) {
            new_tri.v0[0][j] = 0.0f;
            new_tri.v0[1][j] = 0.0f;
            new_tri.v0[2][j] = 0.0f;
            new_tri.e1[0][j] = 0.0f;
            new_tri.e1[1][j] = 0.0f;
            new_tri.e1[2][j] = 0.0f;
            new_tri.e2[0][j] = 0.0f;
            new_tri.e2[1][j] = 0.0f;
            new_tri.e2[2][j] = 0.0f;
            new_tri.n[0][j] = 0.0f;
            new_tri.n[1][j] = 0.0f;
            new_tri.n[2][j] = 0.0f;
            new_tri.id[j] = 0x80000000;
        }

        new_tris.push_back(new_tri);
    }
    new_tris.back().id[3] |= 0x80000000;
}

void extract_bvh4_node(BVH4::NodeRef node,
                       int index,
                       std::vector<Bvh4Node>& new_nodes,
                       std::vector<Bvh4Tri>&  new_tris) {
    assert(node.isAlignedNode());

    BVH4::AlignedNode* n = node.alignedNode();
    Bvh4Node new_node;

    int child_count = 0;
    int leaf_count = 0;
    for (size_t i = 0; i < 4; i++) {
        child_count += n->child(i) != BVH4::emptyNode;
        leaf_count  += n->child(i) != BVH4::emptyNode &&
                       !n->child(i).isAlignedNode();
    }

    int first_child = new_nodes.size();
    new_nodes.resize(new_nodes.size() + child_count - leaf_count);

    size_t c = 0;
    for (size_t i = 0; c < child_count; i++) {
        if (n->child(i) == BVH4::emptyNode) continue;

        new_node.min[0][c] = n->bounds(i).lower.x;
        new_node.min[1][c] = n->bounds(i).lower.y;
        new_node.min[2][c] = n->bounds(i).lower.z;
        new_node.max[0][c] = n->bounds(i).upper.x;
        new_node.max[1][c] = n->bounds(i).upper.y;
        new_node.max[2][c] = n->bounds(i).upper.z;

        if (n->child(i).isAlignedNode()) {
            new_node.child[c] = first_child;
            extract_bvh4_node(n->child(i), first_child++, new_nodes, new_tris);
        } else {
            new_node.child[c] = ~new_tris.size();
            extract_bvh4_leaf(n->child(i), new_tris);
        }
        c++;
    }
    for (; c < 4; c++) {
        new_node.min[0][c] = 0.0f;
        new_node.min[1][c] = 0.0f;
        new_node.min[2][c] = 0.0f;
        new_node.max[0][c] = 0.0f;
        new_node.max[1][c] = 0.0f;
        new_node.max[2][c] = 0.0f;
        new_node.child[c]  = 0;
    }
    new_nodes[index] = new_node;
}

bool build_bvh4(std::ofstream& out, const std::vector<Tri>& tris) {
    auto device = rtcNewDevice("tri_accel=bvh4.triangle4v");
    error_handler(rtcDeviceGetError(device), "");
    rtcDeviceSetErrorFunction(device, error_handler);

    auto scene = rtcDeviceNewScene(device, RTC_SCENE_STATIC, RTC_INTERSECT1);
    auto mesh = rtcNewTriangleMesh(scene, RTC_GEOMETRY_STATIC, tris.size(), tris.size() * 3);

    auto vertices = (Vec3fa*)rtcMapBuffer(scene, mesh, RTC_VERTEX_BUFFER); 
    for (int i = 0; i < tris.size(); i++) {
        vertices[3 * i + 0] = Vec3fa(tris[i].v0.x, tris[i].v0.y, tris[i].v0.z);
        vertices[3 * i + 1] = Vec3fa(tris[i].v1.x, tris[i].v1.y, tris[i].v1.z);
        vertices[3 * i + 2] = Vec3fa(tris[i].v2.x, tris[i].v2.y, tris[i].v2.z);
    }
    rtcUnmapBuffer(scene, mesh, RTC_VERTEX_BUFFER); 

    auto triangles = (int*)rtcMapBuffer(scene, mesh, RTC_INDEX_BUFFER);
    for (int i = 0; i < tris.size(); i++) {
        triangles[i * 3 + 0] = i * 3 + 0;
        triangles[i * 3 + 1] = i * 3 + 1;
        triangles[i * 3 + 2] = i * 3 + 2;
    }
    rtcUnmapBuffer(scene, mesh, RTC_INDEX_BUFFER);
    
    rtcCommit(scene);

    BVH4* bvh4 = nullptr;
    AccelData* accel = ((Accel*)scene)->intersectors.ptr;
    if (accel->type == AccelData::TY_BVH4) 
        bvh4 = (BVH4*)accel;
    else
        return false;

    std::vector<Bvh4Node> new_nodes;
    std::vector<Bvh4Tri>  new_tris;
    new_nodes.emplace_back();
    extract_bvh4_node(bvh4->root, 0, new_nodes, new_tris);

    uint64_t offset = sizeof(uint32_t) * 3 +
        sizeof(Bvh4Node) * new_nodes.size() +
        sizeof(Bvh4Tri)  * new_tris.size();
    uint32_t block_type = 2;
    uint32_t num_nodes = new_nodes.size();
    uint32_t num_tris  = new_tris.size();

    out.write((char*)&offset,     sizeof(uint64_t));
    out.write((char*)&block_type, sizeof(uint32_t));
    out.write((char*)&num_nodes,  sizeof(uint32_t));
    out.write((char*)&num_tris,   sizeof(uint32_t));
    out.write((char*)new_nodes.data(), sizeof(Bvh4Node) * new_nodes.size());
    out.write((char*)new_tris.data(),  sizeof(Bvh4Tri)  * new_tris.size());

    rtcDeleteScene(scene);
    rtcDeleteDevice(device);

    return true;
}
