#ifndef LOAD_PNG_H
#define LOAD_PNG_H

#include <memory>

#include "../file_path.h"

namespace png {

struct Image {
    std::unique_ptr<uint8_t[]> pixels;
    size_t width, height;
    size_t channels;
};

bool load_png(const FilePath&, size_t, Image&);

} // namespace png

#endif // LOAD_PNG_H
