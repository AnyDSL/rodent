#include <memory>
#include <fstream>
#include <cmath>

#include <png.h>
#include <jpeglib.h>

#include "image.h"

static void gamma_correct(ImageRgba32& img) {
    for (size_t y = 0; y < img.height; ++y) {
        for (size_t x = 0; x < img.width; ++x) {
            auto* pix = &img.pixels[4 * (y * img.width + x)];
            for (int i = 0; i < 3; ++i)
                pix[i] = std::pow(pix[i] * (1.0f / 255.0f), 2.2f) * 255.0f;
        }
    }
}

static void read_from_stream(png_structp png_ptr, png_bytep data, png_size_t length) {
    png_voidp a = png_get_io_ptr(png_ptr);
    ((std::istream*)a)->read((char*)data, length);
}

bool load_png(const FilePath& path, ImageRgba32& img) {
    std::ifstream file(path, std::ifstream::binary);
    if (!file)
        return false;

    // Read signature
    char sig[8];
    file.read(sig, 8);
    if (!png_check_sig((unsigned char*)sig, 8))
        return false;

    png_structp png_ptr = png_create_read_struct(PNG_LIBPNG_VER_STRING, nullptr, nullptr, nullptr);
    if (!png_ptr)
        return false;

    png_infop info_ptr = png_create_info_struct(png_ptr);
    if (!info_ptr) {
        png_destroy_read_struct(&png_ptr, nullptr, nullptr);
        return false;
    }

    if (setjmp(png_jmpbuf(png_ptr))) {
        png_destroy_read_struct(&png_ptr, &info_ptr, nullptr);
        return false;
    }

    png_set_sig_bytes(png_ptr, 8);
    png_set_read_fn(png_ptr, (png_voidp)&file, read_from_stream);
    png_read_info(png_ptr, info_ptr);

    img.width    = png_get_image_width(png_ptr, info_ptr);
    img.height   = png_get_image_height(png_ptr, info_ptr);

    png_uint_32 color_type = png_get_color_type(png_ptr, info_ptr);
    png_uint_32 bit_depth  = png_get_bit_depth(png_ptr, info_ptr);

    // Expand paletted and grayscale images to RGB
    if (color_type == PNG_COLOR_TYPE_PALETTE) {
        png_set_palette_to_rgb(png_ptr);
    } else if (color_type == PNG_COLOR_TYPE_GRAY ||
               color_type == PNG_COLOR_TYPE_GRAY_ALPHA) {
        png_set_gray_to_rgb(png_ptr);
    }

    // Transform to 8 bit per channel
    if (bit_depth == 16)
        png_set_strip_16(png_ptr);

    // Get alpha channel when there is one
    if (png_get_valid(png_ptr, info_ptr, PNG_INFO_tRNS))
        png_set_tRNS_to_alpha(png_ptr);

    // Otherwise add an opaque alpha channel
    else
        png_set_filler(png_ptr, 0xFF, PNG_FILLER_AFTER);

    img.pixels.reset(new uint8_t[4 * img.width * img.height]);
    std::unique_ptr<png_byte[]> row_bytes(new png_byte[img.width * 4]);
    for (size_t y = 0; y < img.height; y++) {
        png_read_row(png_ptr, row_bytes.get(), nullptr);
        uint8_t* img_row = img.pixels.get() + 4 * img.width * (img.height - 1 - y);
        for (size_t x = 0; x < img.width; x++) {
            for (size_t c = 0; c < 4; ++c)
                img_row[x * 4 + c] = row_bytes[x * 4 + c];
        }
    }

    png_destroy_read_struct(&png_ptr, &info_ptr, nullptr);
    gamma_correct(img);
    return true;
}

static void png_write_to_stream(png_structp png_ptr, png_bytep data, png_size_t length) {
    png_voidp a = png_get_io_ptr(png_ptr);
    ((std::ostream*)a)->write((const char*)data, length);
}

static void png_flush_stream(png_structp) {
    // Nothing to do
}

bool save_png(const FilePath& path, const ImageRgba32& img) {
    std::ofstream file(path, std::ofstream::binary);
    if (!file)
        return false;

    png_structp png_ptr = png_create_write_struct(PNG_LIBPNG_VER_STRING, nullptr, nullptr, nullptr);
    if (!png_ptr)
        return false;

    png_infop info_ptr = png_create_info_struct(png_ptr);
    if (!info_ptr) {
        png_destroy_read_struct(&png_ptr, nullptr, nullptr);
        return false;
    }

    std::unique_ptr<uint8_t[]> row_bytes(new uint8_t[img.width * 4]);
    if (setjmp(png_jmpbuf(png_ptr))) {
        png_destroy_write_struct(&png_ptr, &info_ptr);
        return false;
    }

    png_set_write_fn(png_ptr, &file, png_write_to_stream, png_flush_stream);

    png_set_IHDR(png_ptr, info_ptr, img.width, img.height,
                 8, PNG_COLOR_TYPE_RGBA, PNG_INTERLACE_NONE,
                 PNG_COMPRESSION_TYPE_BASE, PNG_FILTER_TYPE_BASE);

    png_write_info(png_ptr, info_ptr);

    for (size_t y = 0; y < img.height; y++) {
        const uint8_t* img_row = img.pixels.get() + img.width * 4 * y;
        for (size_t x = 0; x < img.width; x++) {
            for (size_t c = 0; c < 4; ++c)
                row_bytes[x * 4 + c] = img_row[x * 4 + c];
        }
        png_write_row(png_ptr, row_bytes.get());
    }

    png_write_end(png_ptr, info_ptr);
    png_destroy_write_struct(&png_ptr, &info_ptr);
    return true;
}

struct enhanced_jpeg_decompress_struct : jpeg_decompress_struct {
    jmp_buf jmp;
    std::istream* is;
    JOCTET src_buf[1024];
};

static void jpeg_error_exit(j_common_ptr cinfo) {
    cinfo->err->output_message(cinfo);
    longjmp(reinterpret_cast<enhanced_jpeg_decompress_struct*>(cinfo)->jmp, 1);
}

static void jpeg_output_message(j_common_ptr) {}

static void jpeg_no_op(j_decompress_ptr) {}

static boolean jpeg_fill_input_buffer(j_decompress_ptr cinfo) {
    auto enhanced = static_cast<enhanced_jpeg_decompress_struct*>(cinfo);
    enhanced->is->read((char*)enhanced->src_buf, 1024);
    cinfo->src->bytes_in_buffer = enhanced->is->gcount();
    cinfo->src->next_input_byte = enhanced->src_buf;
    return TRUE;
}

static void jpeg_skip_input_data(j_decompress_ptr cinfo, long num_bytes) {
    auto enhanced = static_cast<enhanced_jpeg_decompress_struct*>(cinfo);
    if (num_bytes != 0) {
        if (num_bytes < long(cinfo->src->bytes_in_buffer)) {
            cinfo->src->next_input_byte += num_bytes;
            cinfo->src->bytes_in_buffer -= num_bytes;
        } else {
            enhanced->is->seekg(num_bytes - cinfo->src->bytes_in_buffer, std::ios_base::cur);
            cinfo->src->bytes_in_buffer = 0;
        }
    }
}

bool load_jpg(const FilePath& path, ImageRgba32& image) {
    std::ifstream file(path, std::ifstream::binary);
    if (!file)
        return false;

    enhanced_jpeg_decompress_struct cinfo;
    cinfo.is = &file;
    jpeg_error_mgr jerr;

    cinfo.err           = jpeg_std_error(&jerr);
    jerr.error_exit     = jpeg_error_exit;
    jerr.output_message = jpeg_output_message;
    jpeg_create_decompress(&cinfo);

    if (setjmp(cinfo.jmp)) {
        jpeg_abort_decompress(&cinfo);
        jpeg_destroy_decompress(&cinfo);
        return false;
    }

    jpeg_source_mgr src;
    src.init_source       = jpeg_no_op;
    src.fill_input_buffer = jpeg_fill_input_buffer;
    src.skip_input_data   = jpeg_skip_input_data;
    src.resync_to_restart = jpeg_resync_to_restart;
    src.term_source       = jpeg_no_op;
    src.bytes_in_buffer   = 0;
    cinfo.src = &src;

    jpeg_read_header(&cinfo, TRUE);
    jpeg_start_decompress(&cinfo);
    image.width  = cinfo.output_width;
    image.height = cinfo.output_height;
    auto image_size = image.width * image.height * 4;
    image.pixels.reset(new uint8_t[image_size]);
    std::fill(image.pixels.get(), image.pixels.get() + image_size, 0);
    auto channels = cinfo.output_components;

    std::unique_ptr<JSAMPLE[]> row(new JSAMPLE[image.width * channels]);
    for (size_t y = 0; y < image.height; y++) {
        auto src_ptr = row.get();
        auto dst_ptr = &image.pixels[(image.height - 1 - y) * image.width * 4];
        jpeg_read_scanlines(&cinfo, &src_ptr, 1);
        for (size_t x = 0; x < image.width; ++x, src_ptr += channels, dst_ptr += 4) {
            for (size_t c = 0; c < channels; c++)
                dst_ptr[c] = src_ptr[c];
        }
    }

    jpeg_finish_decompress(&cinfo);
    jpeg_destroy_decompress(&cinfo);
    gamma_correct(image);
    return true;
}
