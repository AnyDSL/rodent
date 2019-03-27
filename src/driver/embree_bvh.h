#if EMBREE_VERSION == 3
#include <embree3/rtcore.h>
#else
#include <embree2/rtcore.h>
#endif

#include <kernels/bvh/bvh.h>
#include <kernels/geometry/triangle.h>

#include "obj.h"

template <size_t N>
struct BvhFromArity {};

template <>
struct BvhFromArity<4> {
    using Type = embree::BVH4;
};

template <>
struct BvhFromArity<8> {
    using Type = embree::BVH8;
};

static void error_handler(void*, const RTCError code, const char* str) {
#if EMBREE_VERSION == 3
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
#else
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
#endif

    if (str) std::cerr << " (" << str << ")";
    std::cerr << std::endl;

    exit(1);
}

template <size_t M, typename NodeRef, typename BvhTri>
void extract_bvh_leaf(const obj::TriMesh& tri_mesh, NodeRef leaf, std::vector<BvhTri>& new_tris) {
    using namespace embree;

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
            auto prim_id = tris[i].primID(j);
            new_tri.prim_id[cur] = prim_id;
            new_tri.geom_id[cur] = tri_mesh.indices[prim_id * 4 + 3];
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

template <size_t N, size_t M, typename Bvh, typename NodeRef, typename BvhNode, typename BvhTri>
void extract_bvh_node(const obj::TriMesh& tri_mesh,
                      NodeRef node, int index,
                      std::vector<BvhNode>& new_nodes,
                      std::vector<BvhTri>&  new_tris) {
    using namespace embree;
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
            extract_bvh_node<N, M, Bvh>(tri_mesh, n->child(i), first_child++, new_nodes, new_tris);
        } else if (n->child(i).isLeaf()) {
            new_node.child[c] = ~new_tris.size();
            extract_bvh_leaf<M>(tri_mesh, n->child(i), new_tris);
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

template <size_t N, typename BvhNode, typename BvhTri>
bool build_embree_bvh(const obj::TriMesh& tri_mesh, std::vector<BvhNode>& new_nodes, std::vector<BvhTri>& new_tris) {
    using namespace embree;
    typedef typename BvhFromArity<N>::Type Bvh;
    static_assert(N == 4 || N == 8, "N must be 4 or 8");

    const char* init = N == 4 ? "tri_accel=bvh4.triangle4" : "tri_accel=bvh8.triangle4";
    auto device = rtcNewDevice(init);
#if EMBREE_VERSION == 3
    error_handler(nullptr, rtcGetDeviceError(device), "");
    rtcSetDeviceErrorFunction(device, error_handler, nullptr);

    auto scene = rtcNewScene(device);
    auto mesh = rtcNewGeometry(device, RTC_GEOMETRY_TYPE_TRIANGLE);

    auto vertices = (float*)rtcSetNewGeometryBuffer(mesh, RTC_BUFFER_TYPE_VERTEX, 0, RTC_FORMAT_FLOAT3, sizeof(float) * 4, tri_mesh.vertices.size());
    for (size_t i = 0; i < tri_mesh.vertices.size(); i++) {
        auto& v = tri_mesh.vertices[i];
        vertices[4 * i +  0] = v.x;
        vertices[4 * i +  1] = v.y;
        vertices[4 * i +  2] = v.z;
        vertices[4 * i +  3] = 1.0f;
    }

    auto indices = (int*)rtcSetNewGeometryBuffer(mesh, RTC_BUFFER_TYPE_INDEX, 0, RTC_FORMAT_UINT3, sizeof(int) * 3, tri_mesh.indices.size() / 4);
    for (size_t i = 0, j = 0; i < tri_mesh.indices.size(); i += 4, j += 3) {
        indices[j + 0] = tri_mesh.indices[i + 0];
        indices[j + 1] = tri_mesh.indices[i + 1];
        indices[j + 2] = tri_mesh.indices[i + 2];
    }

    rtcCommitGeometry(mesh);
    rtcAttachGeometry(scene, mesh);
    rtcReleaseGeometry(mesh);
    rtcCommitScene(scene);
#else
    error_handler(nullptr, rtcDeviceGetError(device), "");
    rtcDeviceSetErrorFunction2(device, error_handler, nullptr);

    auto scene = rtcDeviceNewScene(device, RTC_SCENE_STATIC, RTC_INTERSECT8 | RTC_INTERSECT4 | RTC_INTERSECT1);
    auto mesh = rtcNewTriangleMesh(scene, RTC_GEOMETRY_STATIC, tri_mesh.indices.size() / 4, tri_mesh.vertices.size());
    auto vertices = (float*)rtcMapBuffer(scene, mesh, RTC_VERTEX_BUFFER);
    for (size_t i = 0; i < tri_mesh.vertices.size(); i++) {
        auto& v = tri_mesh.vertices[i];
        vertices[4 * i +  0] = v.x;
        vertices[4 * i +  1] = v.y;
        vertices[4 * i +  2] = v.z;
        vertices[4 * i +  3] = 1.0f;
    }
    rtcUnmapBuffer(scene, mesh, RTC_VERTEX_BUFFER);

    auto indices = (int*)rtcMapBuffer(scene, mesh, RTC_INDEX_BUFFER);
    for (size_t i = 0, j = 0; i < tri_mesh.indices.size(); i += 4, j += 3) {
        indices[j + 0] = tri_mesh.indices[i + 0];
        indices[j + 1] = tri_mesh.indices[i + 1];
        indices[j + 2] = tri_mesh.indices[i + 2];
    }
    rtcUnmapBuffer(scene, mesh, RTC_INDEX_BUFFER);
    rtcCommit(scene);
#endif

    Bvh* bvh = nullptr;
    AccelData* accel = ((Accel*)scene)->intersectors.ptr;
    if (N == 4 && accel->type == AccelData::TY_BVH4) 
        bvh = (Bvh*)accel;
    else if (N == 8 && accel->type == AccelData::TY_BVH8) 
        bvh = (Bvh*)accel;
    else
        return false;

    new_nodes.emplace_back();
    extract_bvh_node<N, 4, Bvh>(tri_mesh, bvh->root, 0, new_nodes, new_tris);

#if EMBREE_VERSION == 3
    rtcReleaseScene(scene);
    rtcReleaseDevice(device);
#else
    rtcDeleteScene(scene);
    rtcDeleteDevice(device);
#endif
    return true;
}
