#ifndef LOAD_BVH_H
#define LOAD_BVH_H

#include <fstream>
#include <anydsl_runtime.hpp>
#include "traversal.h"

enum class BvhType : uint32_t {
    BVH2_TRI1 = 1,
    BVH4_TRI4 = 2,
    BVH8_TRI4 = 3
};

namespace detail {

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

} // namespace detail

template <typename Node, typename Tri>
inline bool load_bvh(const std::string& filename,
                     anydsl::Array<Node>& nodes,
                     anydsl::Array<Tri>& tris,
                     BvhType bvh_type,
                     anydsl::Platform platform,
                     anydsl::Device device) {
    std::ifstream in(filename, std::ifstream::binary);
    if (!in || !detail::check_header(in) || !detail::locate_block(in, bvh_type))
        return false;

    detail::BvhHeader header;
    in.read((char*)&header, sizeof(detail::BvhHeader));
    auto host_nodes = std::move(anydsl::Array<Node>(header.node_count));
    auto host_tris  = std::move(anydsl::Array<Tri >(header.tri_count ));
    in.read((char*)host_nodes.data(), sizeof(Node) * header.node_count);
    in.read((char*)host_tris.data(),  sizeof(Tri)  * header.tri_count);

    if (platform != anydsl::Platform::Host) {
        nodes = std::move(anydsl::Array<Node>(platform, device, header.node_count));
        tris  = std::move(anydsl::Array<Tri >(platform, device, header.tri_count ));
        anydsl::copy(host_nodes, nodes);
        anydsl::copy(host_tris,  tris);
    } else {
        nodes = std::move(host_nodes);
        tris  = std::move(host_tris);
    }
    return true;
}

#endif // LOAD_BVH_H
