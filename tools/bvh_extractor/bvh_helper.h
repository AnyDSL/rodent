#ifndef BVH_HELPER_H
#define BVH_HELPER_H

#include <memory>
#include <vector>
#include <algorithm>

#include "float3.h"
#include "bbox.h"

template <typename Allocator = std::allocator<uint8_t> >
class MemoryPool {
public:
    ~MemoryPool() {
        cleanup();
    }

    template <typename T>
    T* alloc(size_t count) {
        size_t size = count * sizeof(T);
        chunks_.emplace_back(alloc_.allocate(size), size);
        return reinterpret_cast<T*>(chunks_.back().first);
    }

    void cleanup() {
        for (auto chunk: chunks_)
            alloc_.deallocate(chunk.first, chunk.second);
        chunks_.clear();
    }

private:
    typedef std::pair<uint8_t*, size_t> Chunk;

    std::vector<Chunk> chunks_;
    Allocator alloc_;
};

template <typename Node, int N>
struct MultiNode {
    Node nodes[N];
    BBox bbox;
    int count;

    MultiNode(const Node& node) {
        nodes[0] = node;
        bbox = node.bbox;
        count = 1;
    }

    bool is_full() const { return count == N; }
    bool is_leaf() const { return count == 1; }

    void sort_nodes() {
        std::sort(nodes, nodes + count, [] (const Node& a, const Node& b) {
            return a.size() < b.size();
        });
    }

    int next_node() const {
        assert(node_available());
        if (N == 2)
            return 0;
        else {
            float min_cost = FLT_MAX;
            int min_idx = 0;
            for (int i = 0; i < count; i++) {
                if (!nodes[i].tested && min_cost > nodes[i].cost) {
                    min_idx = i;
                    min_cost = nodes[i].cost;
                }
            }
            return min_idx;
        }
    }

    bool node_available() const {
        for (int i = 0; i < count; i++) {
            if (!nodes[i].tested) return true;
        }
        return false;
    }

    void split_node(int i, const Node& left, const Node& right) {
        assert(count < N);
        nodes[i] = left;
        nodes[count++] = right;
    }
};

template <typename T, int N = 128>
struct Stack {
    static constexpr int capacity() { return N; }

    T elems[N];
    int top;

    Stack() : top(-1) {}

    template <typename... Args>
    void push(Args... args) {
        assert(!is_full());
        elems[++top] = T(args...);
    }

    T pop() {
        assert(!is_empty());
        return elems[top--];
    }

    bool is_empty() const { return top < 0; }
    bool is_full() const { return top >= N - 1; }
    int size() const { return top + 1; }
};

#endif // BVH_HELPER_H
