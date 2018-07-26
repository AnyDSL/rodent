#include <fstream>

#include "traversal.h"
#include "sbvh_builder.h"

static void fill_dummy_parent(Node2& node, const BBox& leaf_bb, int index) {
    node.left  = index;
    node.right = 0;

    node.min1[0] = leaf_bb.min.x;
    node.min1[1] = leaf_bb.min.y;
    node.min1[2] = leaf_bb.min.z;
    node.max1[0] = leaf_bb.max.x;
    node.max1[1] = leaf_bb.max.y;
    node.max1[2] = leaf_bb.max.z;

    node.min2[0] = 0.0f;
    node.min2[1] = 0.0f;
    node.min2[2] = 0.0f;
    node.max2[0] = -0.0f;
    node.max2[1] = -0.0f;
    node.max2[2] = -0.0f;
}

class Bvh2Builder {
public:
    Bvh2Builder(std::vector<Node2>& nodes, std::vector<Tri1>& tris)
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
                *(&nodes[elem.parent].left + elem.child) = i + 1;
            }

            assert(count == 2);

            const BBox& left_bb = bboxes(0);
            nodes[i].min1[0] = left_bb.min.x;
            nodes[i].min1[1] = left_bb.min.y;
            nodes[i].min1[2] = left_bb.min.z;
            nodes[i].max1[0] = left_bb.max.x;
            nodes[i].max1[1] = left_bb.max.y;
            nodes[i].max1[2] = left_bb.max.z;

            const BBox& right_bb = bboxes(1);
            nodes[i].min2[0] = right_bb.min.x;
            nodes[i].min2[1] = right_bb.min.y;
            nodes[i].min2[2] = right_bb.min.z;
            nodes[i].max2[0] = right_bb.max.x;
            nodes[i].max2[1] = right_bb.max.y;
            nodes[i].max2[2] = right_bb.max.z;

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
                Tri1 new_tri{
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
    std::vector<Node2>& nodes_;
    std::vector<Tri1>& tris_;
};

int build_bvh2(std::ofstream& out, const std::vector<Tri>& tris) {
    std::vector<Node2> new_nodes;
    std::vector<Tri1>  new_tris;
    Bvh2Builder builder(new_nodes, new_tris);

    builder.build(tris.data(), tris.size());

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
