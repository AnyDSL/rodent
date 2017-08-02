#include <fstream>

#include <embree2/rtcore.h>

#include <config.h>
#include <kernels/bvh/bvh.h>
#include <kernels/geometry/triangle.h>

#include "traversal.h"
#include "tri.h"

using namespace embree;

static void error_handler(const RTCError code, const char* str) {
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

    abort();
}

template <int N, typename NodeRef, typename BvhTri>
void extract_bvh_leaf(NodeRef leaf, std::vector<BvhTri>& new_tris) {
    size_t num; 
    auto tris = (const Triangle4*)leaf.leaf(num);

    BvhTri new_tri;
    size_t cur = 0;
    for (size_t i = 0; i < num; i++) {
        for (size_t j = 0; j < tris[i].size(); j++) {
            new_tri.v0[0][cur] = tris[i].v0.x[j];
            new_tri.v0[1][cur] = tris[i].v0.y[j];
            new_tri.v0[2][cur] = tris[i].v0.z[j];
            new_tri.e1[0][cur] = tris[i].e1.x[j];
            new_tri.e1[1][cur] = tris[i].e1.y[j];
            new_tri.e1[2][cur] = tris[i].e1.z[j];
            new_tri.e2[0][cur] = tris[i].e2.x[j];
            new_tri.e2[1][cur] = tris[i].e2.y[j];
            new_tri.e2[2][cur] = tris[i].e2.z[j];
            new_tri. n[0][cur] = new_tri.e1[1][cur] * new_tri.e2[2][cur] - new_tri.e1[2][cur] * new_tri.e2[1][cur];
            new_tri. n[1][cur] = new_tri.e1[2][cur] * new_tri.e2[0][cur] - new_tri.e1[0][cur] * new_tri.e2[2][cur];
            new_tri. n[2][cur] = new_tri.e1[0][cur] * new_tri.e2[1][cur] - new_tri.e1[1][cur] * new_tri.e2[0][cur];
            new_tri.id[cur] = tris[i].primID(j);
            cur++;

            if (cur >= N) {
                new_tris.push_back(new_tri);
                cur = 0;
            }
        }
    }
    if (cur > 0) {
        for (size_t j = cur; j < N; j++) {
            new_tri.v0[0][j] = 0.0f;
            new_tri.v0[1][j] = 0.0f;
            new_tri.v0[2][j] = 0.0f;
            new_tri.e1[0][j] = 0.0f;
            new_tri.e1[1][j] = 0.0f;
            new_tri.e1[2][j] = 0.0f;
            new_tri.e2[0][j] = 0.0f;
            new_tri.e2[1][j] = 0.0f;
            new_tri.e2[2][j] = 0.0f;
            new_tri.n[0][j]  = 0.0f;
            new_tri.n[1][j]  = 0.0f;
            new_tri.n[2][j]  = 0.0f;
            new_tri.id[j]    = 0xFFFFFFFF;
        }
        new_tris.push_back(new_tri);
    }
    new_tris.back().id[N - 1] |= 0x80000000;
}

template <int N, typename Bvh, typename NodeRef, typename BvhNode, typename BvhTri>
void extract_bvh_node(NodeRef node, int index,
                      std::vector<BvhNode>& new_nodes,
                      std::vector<BvhTri>&  new_tris) {
    assert(node.isAlignedNode());

    auto n = node.alignedNode();
    BvhNode new_node;

    size_t node_count = 0;
    for (size_t i = 0; i < N; i++)
        node_count += n->child(i) != Bvh::emptyNode && n->child(i).isAlignedNode();

    int first_child = new_nodes.size();
    new_nodes.resize(new_nodes.size() + node_count);

    size_t c = 0;
    for (size_t i = 0; i < N; i++) {
        if (n->child(i) == Bvh::emptyNode) continue;

        new_node.bounds[0][c] = n->bounds(i).lower.x;
        new_node.bounds[1][c] = n->bounds(i).upper.x;
        new_node.bounds[2][c] = n->bounds(i).lower.y;
        new_node.bounds[3][c] = n->bounds(i).upper.y;
        new_node.bounds[4][c] = n->bounds(i).lower.z;
        new_node.bounds[5][c] = n->bounds(i).upper.z;

        if (n->child(i).isAlignedNode()) {
            new_node.child[c] = first_child + 1;
            extract_bvh_node<N, Bvh>(n->child(i), first_child++, new_nodes, new_tris);
        } else if (n->child(i).isLeaf()) {
            new_node.child[c] = ~new_tris.size();
            extract_bvh_leaf<N>(n->child(i), new_tris);
        } else {
            assert(false);
            continue;
        }
        c++;
    }
    for (; c < N; c++) {
        for (int i = 0; i < 3; i++) {
            new_node.bounds[i * 2 + 0][c] =  0.0f;
            new_node.bounds[i * 2 + 1][c] = -0.0f;
        }
        new_node.child[c] = 0;
    }
    new_nodes[index] = new_node;
}

template <int N, typename Bvh, typename BvhNode, typename BvhTri>
int build_embree_bvh(std::ofstream& out, const std::vector<Tri>& tris) {
    static_assert(N == 4 || N == 8, "N must be 4 or 8");

    const char* init = N == 4 ? "tri_accel=bvh4.triangle4" : "tri_accel=bvh8.triangle4";
    auto device = rtcNewDevice(init);
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

    Bvh* bvh = nullptr;
    AccelData* accel = ((Accel*)scene)->intersectors.ptr;
    if (N == 4 && accel->type == AccelData::TY_BVH4) 
        bvh = (Bvh*)accel;
    else if (N == 8 && accel->type == AccelData::TY_BVH8) 
        bvh = (Bvh*)accel;
    else
        return 0;

    std::vector<BvhNode> new_nodes;
    std::vector<BvhTri>  new_tris;
    new_nodes.emplace_back();
    extract_bvh_node<N, Bvh>(bvh->root, 0, new_nodes, new_tris);

    uint64_t offset = sizeof(uint32_t) * 3 +
        sizeof(BvhNode) * new_nodes.size() +
        sizeof(BvhTri)  * new_tris.size();
    uint32_t block_type = N == 4 ? 2 : 3;
    uint32_t num_nodes = new_nodes.size();
    uint32_t num_tris  = new_tris.size();

    out.write((char*)&offset,     sizeof(uint64_t));
    out.write((char*)&block_type, sizeof(uint32_t));
    out.write((char*)&num_nodes,  sizeof(uint32_t));
    out.write((char*)&num_tris,   sizeof(uint32_t));
    out.write((char*)new_nodes.data(), sizeof(BvhNode) * new_nodes.size());
    out.write((char*)new_tris.data(),  sizeof(BvhTri)  * new_tris.size());

    rtcDeleteScene(scene);
    rtcDeleteDevice(device);

    return new_nodes.size();
}

void build_bvh4(std::ofstream& out, const std::vector<Tri>& tris) {
    build_embree_bvh<4, BVH4, Bvh4Node, Bvh4Tri>(out, tris);
}

void build_bvh8(std::ofstream& out, const std::vector<Tri>& tris) {
    build_embree_bvh<8, BVH8, Bvh8Node, Bvh8Tri>(out, tris);
}
