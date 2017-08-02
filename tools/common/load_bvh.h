#ifndef LOAD_BVH_H
#define LOAD_BVH_H

#include <fstream>
#include <anydsl_runtime.hpp>
#include "traversal.h"

namespace detail {

enum class BvhType : uint32_t {
    BVH2 = 1,
    BVH4 = 2,
    BVH8 = 3
};

struct BvhHeader {
    uint32_t node_count;
    uint32_t tri_count;
};

inline bool check_header(std::istream& is) {
    uint32_t magic;
    is.read((char*)&magic, sizeof(uint32_t));
    return magic == 0x95CBED1F;
}

inline bool locate_block(std::istream& is, BvhType type) {
    uint32_t block_type;
    uint64_t offset = 0;
    do {
        is.seekg(offset, std::istream::cur);

        is.read((char*)&offset, sizeof(uint64_t));
        if (is.gcount() != sizeof(std::uint64_t)) return false;
        is.read((char*)&block_type, sizeof(uint32_t));
        if (is.gcount() != sizeof(uint32_t)) return false;

        offset -= sizeof(uint32_t);
    } while (!is.eof() && block_type != (uint32_t)type);

    return static_cast<bool>(is);
}

}

template <typename Node, typename Tri>
inline bool load_bvh(const std::string& filename,
                     anydsl::Array<Node>& nodes,
                     anydsl::Array<Tri>& tris,
                     bool use_gpu) {
    std::ifstream in(filename, std::ifstream::binary);
    if (!in ||
        !detail::check_header(in) ||
        !detail::locate_block(in, use_gpu ? detail::BvhType::BVH2 : detail::BvhType::BVH4))
        return false;

    detail::BvhHeader header;
    in.read((char*)&header, sizeof(detail::BvhHeader));
    auto host_nodes = std::move(anydsl::Array<Node>(header.node_count));
    auto host_tris  = std::move(anydsl::Array<Tri >(header.tri_count ));
    in.read((char*)host_nodes.data(), sizeof(Node) * header.node_count);
    in.read((char*)host_tris.data(),  sizeof(Tri)  * header.tri_count);

    if (use_gpu) {
        nodes = std::move(anydsl::Array<Node>(anydsl::Platform::Cuda, anydsl::Device(0), header.node_count));
        tris  = std::move(anydsl::Array<Tri >(anydsl::Platform::Cuda, anydsl::Device(0), header.tri_count ));
        anydsl::copy(host_nodes, nodes);
        anydsl::copy(host_tris,  tris);
    } else {
        nodes = std::move(host_nodes);
        tris  = std::move(host_tris);
    }
    return true;
}

#endif // LOAD_BVH_H
