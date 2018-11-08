#include <fstream>

#include "traversal.h"
#include "driver/bvh.h"
#include "driver/obj.h"

class Bvh2Builder {
public:
    Bvh2Builder(std::vector<Node2>& nodes, std::vector<Tri1>& tris)
        : nodes_(nodes), tris_(tris)
    {}

    void build(const std::vector<Tri>& tris) {
        builder_.build(tris, NodeWriter(this), LeafWriter(this, tris), 2);
    }

#ifdef STATISTICS
    void print_stats() const { builder_.print_stats(); }
#endif

private:
    struct CostFn {
        static float leaf_cost(int count, float area) {
            return count * area;
        }
        static float traversal_cost(float area) {
            return area * 1.0f;
        }
    };

    struct NodeWriter {
        Bvh2Builder* builder;

        NodeWriter(Bvh2Builder* builder)
            : builder(builder)
        {}

        template <typename BBoxFn>
        int operator() (int parent, int child, const BBox& parent_bb, int count, BBoxFn bboxes) {
            auto& nodes = builder->nodes_;

            int i = nodes.size();
            nodes.emplace_back();

            if (parent >= 0 && child >= 0)
                nodes[parent].child[child] = i + 1;

            assert(count == 2);

            const BBox& bbox1 = bboxes(0);
            nodes[i].bounds[0] = bbox1.min.x;
            nodes[i].bounds[2] = bbox1.min.y;
            nodes[i].bounds[4] = bbox1.min.z;
            nodes[i].bounds[1] = bbox1.max.x;
            nodes[i].bounds[3] = bbox1.max.y;
            nodes[i].bounds[5] = bbox1.max.z;

            const BBox& bbox2 = bboxes(1);
            nodes[i].bounds[ 6] = bbox2.min.x;
            nodes[i].bounds[ 8] = bbox2.min.y;
            nodes[i].bounds[10] = bbox2.min.z;
            nodes[i].bounds[ 7] = bbox2.max.x;
            nodes[i].bounds[ 9] = bbox2.max.y;
            nodes[i].bounds[11] = bbox2.max.z;

            return i;
        }
    };

    struct LeafWriter {
        Bvh2Builder* builder;
        const std::vector<Tri>& ref_tris;

        LeafWriter(Bvh2Builder* builder, const std::vector<Tri>& ref_tris)
            : builder(builder), ref_tris(ref_tris)
        {}

        template <typename RefFn>
        void operator() (int parent, int child, const BBox& leaf_bb, int ref_count, RefFn refs) {
            auto& nodes = builder->nodes_;
            auto& tris  = builder->tris_;

            nodes[parent].child[child] = ~tris.size();

            for (int i = 0; i < ref_count; i++) {
                const int ref = refs(i);
                const Tri& tri = ref_tris[ref];
                auto e1 = tri.v0 - tri.v1;
                auto e2 = tri.v2 - tri.v0;
                auto n = cross(e1, e2);
                tris.emplace_back(Tri1 {
                    { tri.v0.x, tri.v0.y, tri.v0.z}, 0,
                    { e1.x, e1.y, e1.z}, 0,
                    { e2.x, e2.y, e2.z}, ref
                });
            }

            // Add sentinel
            tris.back().prim_id |= 0x80000000;
        }
    };

    SplitBvhBuilder<2, CostFn> builder_;
    std::vector<Node2>& nodes_;
    std::vector<Tri1>& tris_;
};

size_t build_bvh2(std::ofstream& out, const obj::TriMesh& tri_mesh) {
    std::vector<Tri> tris;
    for (size_t i = 0; i < tri_mesh.indices.size(); i += 4) {
        auto& v0 = tri_mesh.vertices[tri_mesh.indices[i + 0]];
        auto& v1 = tri_mesh.vertices[tri_mesh.indices[i + 1]];
        auto& v2 = tri_mesh.vertices[tri_mesh.indices[i + 2]];
        tris.emplace_back(v0, v1, v2);
    }

    std::vector<Node2> new_nodes;
    std::vector<Tri1>  new_tris;
    Bvh2Builder builder(new_nodes, new_tris);

    builder.build(tris);

    uint64_t offset = sizeof(uint32_t) * 3 +
        sizeof(Node2) * new_nodes.size() +
        sizeof(Tri1)  * new_tris.size();
    uint32_t block_type = 1;
    uint32_t num_nodes = new_nodes.size();
    uint32_t num_tris  = new_tris.size();

    out.write((char*)&offset,     sizeof(uint64_t));
    out.write((char*)&block_type, sizeof(uint32_t));
    out.write((char*)&num_nodes,  sizeof(uint32_t));
    out.write((char*)&num_tris,   sizeof(uint32_t));
    out.write((char*)new_nodes.data(), sizeof(Node2) * new_nodes.size());
    out.write((char*)new_tris.data(),  sizeof(Tri1)  * new_tris.size());

    return new_nodes.size();
}
