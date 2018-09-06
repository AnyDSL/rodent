#include <fstream>
#include <limits>

#include <embree3/rtcore.h>

#include <kernels/bvh/bvh.h>
#include <kernels/geometry/triangle.h>

#include "traversal.h"
#include "load_bvh.h"
#include "driver/tri.h"

using namespace embree;

static void error_handler(void*, const RTCError code, const char* str) {
    if (code == RTC_ERROR_NONE)
        return;

    std::cerr << "Embree error: ";
    switch (code) {
        case RTC_ERROR_UNKNOWN:           std::cerr << "RTC_ERROR_UNKNOWN";           break;
        case RTC_ERROR_INVALID_ARGUMENT:  std::cerr << "RTC_ERROR_INVALID_ARGUMENT";  break;
        case RTC_ERROR_INVALID_OPERATION: std::cerr << "RTC_ERROR_INVALID_OPERATION"; break;
        case RTC_ERROR_OUT_OF_MEMORY:     std::cerr << "RTC_ERROR_OUT_OF_MEMORY";     break;
        case RTC_ERROR_UNSUPPORTED_CPU:   std::cerr << "RTC_ERROR_UNSUPPORTED_CPU";   break;
        case RTC_ERROR_CANCELLED:         std::cerr << "RTC_ERROR_CANCELLED";         break;
        default:                          std::cerr << "invalid error code";          break;
    }

    if (str) std::cerr << " (" << str << ")";
    std::cerr << std::endl;

    exit(1);
}

template <int M, typename NodeRef, typename BvhTri>
void extract_bvh_leaf(NodeRef leaf, std::vector<BvhTri>& new_tris) {
    size_t num; 
    auto tris = (const Triangle4*)leaf.leaf(num);

    BvhTri new_tri;
    size_t cur = 0;
    for (size_t i = 0; i < num; i++) {
        for (size_t j = 0; j < tris[i].size(); j++) {
            if (cur >= M) {
                new_tris.push_back(new_tri);
                cur = 0;
            }
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
            new_tri.prim_id[cur] = tris[i].primID(j);
            new_tri.geom_id[cur] = tris[i].geomID(j);
            cur++;
        }
    }
    if (cur > 0) {
        for (size_t j = cur; j < M; j++) {
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
            new_tri.prim_id[j] = 0xFFFFFFFF;
            new_tri.geom_id[j] = 0xFFFFFFFF;
        }
        new_tris.push_back(new_tri);
    }
    new_tris.back().prim_id[M - 1] |= 0x80000000;
}

template <int N, int M, typename Bvh, typename NodeRef, typename BvhNode, typename BvhTri>
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
            extract_bvh_node<N, M, Bvh>(n->child(i), first_child++, new_nodes, new_tris);
        } else if (n->child(i).isLeaf()) {
            new_node.child[c] = ~new_tris.size();
            extract_bvh_leaf<M>(n->child(i), new_tris);
        } else {
            assert(false);
            continue;
        }
        c++;
    }
    for (; c < N; c++) {
        for (int i = 0; i < 3; i++) {
            new_node.bounds[i * 2 + 0][c] =  std::numeric_limits<float>::infinity();
            new_node.bounds[i * 2 + 1][c] = -std::numeric_limits<float>::infinity();
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
    error_handler(nullptr, rtcGetDeviceError(device), "");
    rtcSetDeviceErrorFunction(device, error_handler, nullptr);

    auto scene = rtcNewScene(device);
    auto mesh = rtcNewGeometry(device, RTC_GEOMETRY_TYPE_TRIANGLE);

    auto vertices = (float*)rtcSetNewGeometryBuffer(mesh, RTC_BUFFER_TYPE_VERTEX, 0, RTC_FORMAT_FLOAT3, sizeof(float) * 4, tris.size() * 3);
    for (int i = 0; i < tris.size(); i++) {
        vertices[12 * i +  0] = tris[i].v0.x;
        vertices[12 * i +  1] = tris[i].v0.y;
        vertices[12 * i +  2] = tris[i].v0.z;
        vertices[12 * i +  3] = 1.0f;

        vertices[12 * i +  4] = tris[i].v1.x;
        vertices[12 * i +  5] = tris[i].v1.y;
        vertices[12 * i +  6] = tris[i].v1.z;
        vertices[12 * i +  7] = 1.0f;

        vertices[12 * i +  8] = tris[i].v2.x;
        vertices[12 * i +  9] = tris[i].v2.y;
        vertices[12 * i + 10] = tris[i].v2.z;
        vertices[12 * i + 11] = 1.0f;
    }

    auto indices = (int*)rtcSetNewGeometryBuffer(mesh, RTC_BUFFER_TYPE_INDEX, 0, RTC_FORMAT_UINT3, sizeof(int) * 3, tris.size());
    for (int i = 0; i < tris.size(); i++) {
        indices[i * 3 + 0] = i * 3 + 0;
        indices[i * 3 + 1] = i * 3 + 1;
        indices[i * 3 + 2] = i * 3 + 2;
    }

    rtcCommitGeometry(mesh);
    rtcAttachGeometry(scene, mesh);
    rtcReleaseGeometry(mesh);
    rtcCommitScene(scene);

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
    extract_bvh_node<N, 4, Bvh>(bvh->root, 0, new_nodes, new_tris);

    uint64_t offset = sizeof(uint32_t) * 3 +
        sizeof(BvhNode) * new_nodes.size() +
        sizeof(BvhTri)  * new_tris.size();
    uint32_t block_type = uint32_t(N == 4 ? BvhType::BVH4_TRI4 : BvhType::BVH8_TRI4);
    uint32_t num_nodes = new_nodes.size();
    uint32_t num_tris  = new_tris.size();

    out.write((char*)&offset,     sizeof(uint64_t));
    out.write((char*)&block_type, sizeof(uint32_t));
    out.write((char*)&num_nodes,  sizeof(uint32_t));
    out.write((char*)&num_tris,   sizeof(uint32_t));
    out.write((char*)new_nodes.data(), sizeof(BvhNode) * new_nodes.size());
    out.write((char*)new_tris.data(),  sizeof(BvhTri)  * new_tris.size());

    rtcReleaseScene(scene);
    rtcReleaseDevice(device);

    return new_nodes.size();
}

void build_bvh4(std::ofstream& out, const std::vector<Tri>& tris) {
    build_embree_bvh<4, BVH4, Node4, Tri4>(out, tris);
}

void build_bvh8(std::ofstream& out, const std::vector<Tri>& tris) {
    build_embree_bvh<8, BVH8, Node8, Tri4>(out, tris);
}
