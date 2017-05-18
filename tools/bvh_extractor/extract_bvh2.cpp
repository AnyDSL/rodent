#include <fstream>

#include "traversal.h"
#include "sbvh_builder.h"

static void fill_dummy_parent(Bvh2Node& node, const BBox& leaf_bb, int index) {
    node.left  = index;
    node.right = 0x76543210;

    node.left_bb.lo_x = leaf_bb.min.x;
    node.left_bb.lo_y = leaf_bb.min.y;
    node.left_bb.lo_z = leaf_bb.min.z;
    node.left_bb.hi_x = leaf_bb.max.x;
    node.left_bb.hi_y = leaf_bb.max.y;
    node.left_bb.hi_z = leaf_bb.max.z;

    node.right_bb.lo_x = 0.0f;
    node.right_bb.lo_y = 0.0f;
    node.right_bb.lo_z = 0.0f;
    node.right_bb.hi_x = -0.0f;
    node.right_bb.hi_y = -0.0f;
    node.right_bb.hi_z = -0.0f;
}

class Bvh2Builder {
public:
    Bvh2Builder(std::vector<Bvh2Node>& nodes, std::vector<Bvh2Tri>& tris)
        : nodes_(nodes), tris_(tris)
    {}

    void build(const Tri* tris, size_t tri_count) {
        builder_.build(tris, tri_count, NodeWriter(this), LeafWriter(this, tris), 2);
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
        void operator() (const BBox& parent_bb, int count, BBoxFn bboxes) {
            auto& nodes = builder->nodes_;
            auto& stack = builder->stack_;

            int i = nodes.size();
            nodes.emplace_back();

            if (!stack.is_empty()) {
                StackElem elem = stack.pop();
                *(&nodes[elem.parent].left + elem.child) = i;
            }

            assert(count == 2);

            const BBox& left_bb = bboxes(0);
            nodes[i].left_bb.lo_x = left_bb.min.x;
            nodes[i].left_bb.lo_y = left_bb.min.y;
            nodes[i].left_bb.lo_z = left_bb.min.z;
            nodes[i].left_bb.hi_x = left_bb.max.x;
            nodes[i].left_bb.hi_y = left_bb.max.y;
            nodes[i].left_bb.hi_z = left_bb.max.z;

            const BBox& right_bb = bboxes(1);
            nodes[i].right_bb.lo_x = right_bb.min.x;
            nodes[i].right_bb.lo_y = right_bb.min.y;
            nodes[i].right_bb.lo_z = right_bb.min.z;
            nodes[i].right_bb.hi_x = right_bb.max.x;
            nodes[i].right_bb.hi_y = right_bb.max.y;
            nodes[i].right_bb.hi_z = right_bb.max.z;

            stack.push(i, 1);
            stack.push(i, 0);
        }
    };

    struct LeafWriter {
        Bvh2Builder* builder;
        const Tri* ref_tris;

        LeafWriter(Bvh2Builder* builder, const Tri* ref_tris)
            : builder(builder), ref_tris(ref_tris)
        {}

        template <typename RefFn>
        void operator() (const BBox& leaf_bb, int ref_count, RefFn refs) {
            auto& nodes = builder->nodes_;
            auto& stack = builder->stack_;
            auto& tris  = builder->tris_;

            if (stack.is_empty()) {
                nodes.emplace_back();
                fill_dummy_parent(nodes.back(), leaf_bb, ~tris.size());
            } else {
                const StackElem& elem = stack.pop();
                *(&nodes[elem.parent].left + elem.child) = ~tris.size();
            }

            for (int i = 0; i < ref_count; i++) {
                const int ref = refs(i);
                const Tri& tri = ref_tris[ref];
                auto e1 = tri.v0 - tri.v1;
                auto e2 = tri.v2 - tri.v0;
                auto n = cross(e1, e2);
                Bvh2Tri new_tri{
                    { tri.v0.x, tri.v0.y, tri.v0.z}, n.x,
                    { e1.x, e1.y, e1.z}, n.y,
                    { e2.x, e2.y, e2.z}, ref
                };
                tris.emplace_back(new_tri);
            }

            // Add sentinel
            tris.back().id |= 0x80000000;
        }
    };

    struct StackElem {
        int parent, child;
        StackElem() {}
        StackElem(int parent, int child) : parent(parent), child(child) {}
    };

    Stack<StackElem> stack_;
    SplitBvhBuilder<2, CostFn> builder_;
    std::vector<Bvh2Node>& nodes_;
    std::vector<Bvh2Tri>& tris_;
};

int build_bvh2(std::ofstream& out, const std::vector<Tri>& tris) {
    std::vector<Bvh2Node> new_nodes;
    std::vector<Bvh2Tri>  new_tris;
    Bvh2Builder builder(new_nodes, new_tris);

    builder.build(tris.data(), tris.size());

    uint64_t offset = sizeof(uint32_t) * 3 +
        sizeof(Bvh2Node) * new_nodes.size() +
        sizeof(Bvh2Tri)  * new_tris.size();
    uint32_t block_type = 1;
    uint32_t num_nodes = new_nodes.size();
    uint32_t num_tris  = new_tris.size();

    out.write((char*)&offset,     sizeof(uint64_t));
    out.write((char*)&block_type, sizeof(uint32_t));
    out.write((char*)&num_nodes,  sizeof(uint32_t));
    out.write((char*)&num_tris,   sizeof(uint32_t));
    out.write((char*)new_nodes.data(), sizeof(Bvh2Node) * new_nodes.size());
    out.write((char*)new_tris.data(),  sizeof(Bvh2Tri)  * new_tris.size());

    return new_nodes.size();
}
