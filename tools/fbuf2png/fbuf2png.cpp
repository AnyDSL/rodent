#include <iostream>
#include <fstream>
#include <vector>
#include <algorithm>
#include <png.h>
#include <cstring>

static void write_to_stream(png_structp png_ptr, png_bytep data, png_size_t length) {
    png_voidp a = png_get_io_ptr(png_ptr);
    ((std::ostream*)a)->write((const char*)data, length);
}

static void flush_stream(png_structp png_ptr) {
    // Nothing to do
}

inline void check_argument(int i, int argc, char** argv) {
    if (i + 1 >= argc) {
        std::cerr << "Missing argument for " << argv[i] << std::endl;
        exit(1);
    }
}

inline void usage() {
    std::cout << "Usage: fbuf2png [options] input output\n"
                 "Available options:\n"
                 "  -sx      --width        Sets the width of the image (default: 1024)\n"
                 "  -sy      --height       Sets the height of the image (default: 1024)\n"
                 "  -n       --normalize    Normalizes the values contained in the image (disabled by default)\n";
}

int main(int argc, char** argv) {
    bool normalize = false;
    int width = 1024;
    int height = 1024;
    std::vector<std::string> files;

    for (int i = 1; i < argc; i++) {
        auto arg = argv[i];
        if (arg[0] == '-') {
            if (!strcmp(arg, "-h") || !strcmp(arg, "--help")) {
                usage();
                return 0;
            } else if (!strcmp(arg, "-n") || !strcmp(arg, "--normalize")) {
                normalize = true;
            } else if (!strcmp(arg, "-sx") || !strcmp(arg, "--width")) {
                check_argument(i, argc, argv);
                width = strtol(argv[++i], nullptr, 10);
            } else if (!strcmp(arg, "-sy") || !strcmp(arg, "--height")) {
                check_argument(i, argc, argv);
                height = strtol(argv[++i], nullptr, 10);
            } else {
                std::cerr << "Unknown option '" << arg << "'" << std::endl;
                return 1;
            }
        } else {
            files.push_back(argv[i]);
        }
    }

    if (files.size() < 2) {
        std::cerr << "Missing input or output file" << std::endl;
        return 1;
    }
    if (files.size() > 2) {
        std::cerr << "Too many arguments" << std::endl;
        return 1;
    }

    std::ifstream fbuf_file(files[0], std::ofstream::binary);
    std::ofstream png_file(files[1], std::ofstream::binary);
    if (!fbuf_file || !png_file)
        return 1;

    // Read fbuf file and convert it to an image
    std::vector<float> float_image(width * height);
    if (!fbuf_file.read((char*)float_image.data(), width * height * sizeof(float))) {
        std::cerr << "Not enough data in the float buffer" << std::endl;
        return 1;
    }
    const float tmax = normalize ? *std::max_element(float_image.begin(), float_image.end()) : 1.0f;

    png_structp png_ptr = png_create_write_struct(PNG_LIBPNG_VER_STRING, nullptr, nullptr, nullptr);
    if (!png_ptr) {
        return 1;
    }

    png_infop info_ptr = png_create_info_struct(png_ptr);
    if (!info_ptr) {
        png_destroy_read_struct(&png_ptr, nullptr, nullptr);
        return 1;
    }

    std::vector<uint8_t> row(width * 4);
    if (setjmp(png_jmpbuf(png_ptr))) {
        png_destroy_write_struct(&png_ptr, &info_ptr);
        return 1;
    }

    png_set_write_fn(png_ptr, &png_file, write_to_stream, flush_stream);

    png_set_IHDR(png_ptr, info_ptr, width, height,
                 8, PNG_COLOR_TYPE_RGBA, PNG_INTERLACE_NONE,
                 PNG_COMPRESSION_TYPE_BASE, PNG_FILTER_TYPE_BASE);

    png_write_info(png_ptr, info_ptr);

    for (int y = 0; y < height; y++) {
        for (int x = 0; x < width; x++) {
            uint8_t c = 255.0f * float_image[y * width + x] / tmax;
            row[x * 4 + 0] = c;
            row[x * 4 + 1] = c;
            row[x * 4 + 2] = c;
            row[x * 4 + 3] = 255.0f;
        }
        png_write_row(png_ptr, row.data());
    }

    png_write_end(png_ptr, info_ptr);
    png_destroy_write_struct(&png_ptr, &info_ptr);

    return 0;
}
