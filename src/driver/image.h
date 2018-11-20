#ifndef IMAGE_H
#define IMAGE_H

#include "file_path.h"

struct ImageRgba32 {
    std::unique_ptr<uint8_t[]> pixels;
    size_t width, height;
};

bool load_png(const FilePath&, ImageRgba32&);
bool load_jpg(const FilePath&, ImageRgba32&);
bool save_png(const FilePath&, const ImageRgba32&);

#endif // IMAGE_H
