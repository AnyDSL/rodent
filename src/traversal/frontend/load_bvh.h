#ifndef LOAD_BVH_H
#define LOAD_BVH_H

#include <fstream>
#include <anydsl_runtime.hpp>
#include "traversal.h"

namespace detail {

enum BlockType : uint32_t {
    BVH4 = 2
};

struct Bvh4Header {
    uint32_t node_count;
    uint32_t tri_count;
};

inline bool check_header(std::istream& is) {
    uint32_t magic;
    is.read((char*)&magic, sizeof(uint32_t));
    return magic == 0x313F1A57;
}

inline bool locate_block(std::istream& is, BlockType type) {
    uint32_t block_type;
    uint64_t offset = 0;
    do {
        is.seekg(offset, std::istream::cur);

        is.read((char*)&offset, sizeof(uint64_t));
        if (is.gcount() != sizeof(std::uint64_t)) return false;
        is.read((char*)&block_type, sizeof(uint32_t));
        if (is.gcount() != sizeof(uint32_t)) return false;

        offset -= sizeof(BlockType);
    } while (!is.eof() && block_type != (uint32_t)type);

    return static_cast<bool>(is);
}

}

inline bool load_bvh(const std::string& filename,
                     anydsl::Array<Bvh4Node>& nodes,
                     anydsl::Array<Bvh4Tri>& tris) {
    std::ifstream in(filename, std::ifstream::binary);
    if (!in ||
        !detail::check_header(in) ||
        !detail::locate_block(in, detail::BlockType::BVH4))
        return false;

    detail::Bvh4Header header;
    in.read((char*)&header, sizeof(detail::Bvh4Header));
    nodes = std::move(anydsl::Array<Bvh4Node>(header.node_count));
    tris  = std::move(anydsl::Array<Bvh4Tri >(header.tri_count ));
    in.read((char*)nodes.data(), sizeof(Bvh4Node) * header.node_count);
    in.read((char*)tris.data(),  sizeof(Bvh4Tri)  * header.tri_count);
    return true;
}

#endif // LOAD_BVH_H
