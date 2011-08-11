//
// (c) Copyright 2011 Ilyes Gouta, ilyes.gouta@gmail.com
//
// This is a libjpeg-based parser for MPO files.
//
// This library is free software; you can redistribute it and/or
// modify it under the terms of the GNU Lesser General Public
// License as published by the Free Software Foundation; either
// version 2 of the License, or (at your option) any later version.
//
// This library is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
// Lesser General Public License for more details.
//
// You should have received a copy of the GNU Lesser General Public
// License along with this library; if not, write to the
// Free Software Foundation, Inc., 59 Temple Place - Suite 330,
// Boston, MA 02111-1307, USA.
//

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <setjmp.h>
#include <math.h>

#include <jpeglib.h>

#include <iostream>

using namespace std;

static struct jpeg_decompress_struct cinfo;

struct my_error_mgr {
     struct jpeg_error_mgr pub;
     jmp_buf setjmp_buffer;
};

static struct my_error_mgr jerr;

static FILE* input;

static void jpeglib_panic(j_common_ptr cinfo)
{
     struct my_error_mgr *myerr = (struct my_error_mgr*) cinfo->err;
     longjmp(myerr->setjmp_buffer, 1);
}

// APP2 max. size is 64k
#define APP2 0xe2

#define PMF_MAGIC       0x4d504600
#define DISPARITY_MAGIC 0x00020002

enum APP2_err_codes {
    could_not_find_MPF_magic,
    bad_byte_ordering,
    bad_IFD_index_offset,
    bad_format_version_tag,
    bad_count_images_tag,
    non_stereo_pictures,
    non_disparity_image
};

template<int S = 0> struct EXIFtypes {
    typedef unsigned char UNDEFINED_ARRAY[S];
    typedef unsigned char ASCII_ARRAY[S];
    typedef unsigned char UNDEFINED;
    typedef unsigned char ASCII;
    typedef unsigned int LONG;
    typedef unsigned int OFFSET;
};

template<typename T>
union InteropEXIF {
    struct __attribute__((packed)) {
        unsigned short tag;
        unsigned short type;
        unsigned int count;
        T value_offset;
    } EXIFtag;
    unsigned char array[12];
};

typedef struct MP_Entry_ {
    unsigned int image_attribute;
    unsigned int image_size;
    unsigned int image_offset;
    unsigned short sibling_entry_1;
    unsigned short sibling_entry_2;
} MP_Entry;

typedef struct MP_Index_IFD_ {
    struct __attribute__((packed)) MP_index_ {
        struct __attribute__((packed)) {
            unsigned short count;
            InteropEXIF<EXIFtypes<4>::ASCII_ARRAY> MP_format_version;
            InteropEXIF<EXIFtypes<>::LONG> MP_count_images;
            InteropEXIF<EXIFtypes<>::LONG> MP_entry;
        } common;
        struct __attribute__((packed)) {
            InteropEXIF<EXIFtypes<4>::UNDEFINED_ARRAY> MP_imageUID;
            InteropEXIF<EXIFtypes<>::LONG> MP_total_frames;
        } optional;
        unsigned int MP_next_IFD;
    } MP_index;
    struct MP_data_ {
        MP_Entry *MP_entries;
        unsigned int absolute_MPhdr_offset;
    } MP_data;
} MP_Index_IFD;

typedef struct MP_Attributes_IFD {
} MP_Attributes_IFD;

static MP_Index_IFD MP_index;
static MP_Attributes_IFD MP_attributes;

static unsigned int jpeg_getc(j_decompress_ptr cinfo)
{
    struct jpeg_source_mgr * datasrc = cinfo->src;

    if (datasrc->bytes_in_buffer == 0) {
        if (!(*datasrc->fill_input_buffer) (cinfo))
            return -1;
    }

    datasrc->bytes_in_buffer--;
    return (*datasrc->next_input_byte++);
}

static int copy_buffer(j_decompress_ptr cinfo, unsigned char* dest, int size)
{
    struct jpeg_source_mgr * datasrc = cinfo->src;

    while (size > 0)
    {
        int x = datasrc->bytes_in_buffer <= size ? datasrc->bytes_in_buffer : size;

        memcpy(dest, datasrc->next_input_byte, x);
        dest += x;

        datasrc->bytes_in_buffer -= x;
        datasrc->next_input_byte += x;

        if (!datasrc->bytes_in_buffer) {
            if (!(*datasrc->fill_input_buffer) (cinfo))
                return -1;
        }

        size -= x;
    }
}

static inline unsigned int APP2_get32be(unsigned char* buf)
{
    return ((buf[0] << 24) | (buf[1] << 16) | (buf[2] << 8) | buf[3]);
}

static inline unsigned int APP2_get32le(unsigned char* buf)
{
    return ((buf[3] << 24) | (buf[2] << 16) | (buf[1] << 8) | buf[0]);
}

typedef unsigned int (*APPx_fetch_32)(unsigned char*);

APPx_fetch_32 APP2_get32;

static int parse_index_IFD(unsigned char* buf, MP_Index_IFD* index)
{
    unsigned char* base = buf - 8;

    memcpy(&index->MP_index.common, buf, sizeof(MP_Index_IFD::MP_index.common));
    buf += sizeof(MP_Index_IFD::MP_index.common);

    if (index->MP_index.common.MP_format_version.EXIFtag.tag != 0xb000)
        throw bad_format_version_tag;

    if (index->MP_index.common.MP_count_images.EXIFtag.tag != 0xb001)
        throw bad_count_images_tag;

    cout << "index_IFD_version: " << index->MP_index.common.MP_format_version.EXIFtag.value_offset[0]
                                  << index->MP_index.common.MP_format_version.EXIFtag.value_offset[1]
                                  << index->MP_index.common.MP_format_version.EXIFtag.value_offset[2]
                                  << index->MP_index.common.MP_format_version.EXIFtag.value_offset[3] << "\n";

    cout << "index_IFD_number_images: " << index->MP_index.common.MP_count_images.EXIFtag.value_offset << "\n";

    if (index->MP_index.common.MP_count_images.EXIFtag.value_offset != 2) /* Right now, we only support stereo JPEGs. */
        throw non_stereo_pictures;

    if (index->MP_index.common.MP_entry.EXIFtag.value_offset > 50) {
        memcpy(&index->MP_index.optional, buf, sizeof(MP_Index_IFD::MP_index.optional));
        buf += sizeof(MP_Index_IFD::MP_index.optional);
    }

    memcpy(&index->MP_index.MP_next_IFD, buf, sizeof(MP_Index_IFD::MP_index.MP_next_IFD));

    index->MP_data.MP_entries = new MP_Entry[index->MP_index.common.MP_count_images.EXIFtag.value_offset];

    for (int i = 0; i < index->MP_index.common.MP_count_images.EXIFtag.value_offset; i++) {
        memcpy(&index->MP_data.MP_entries[i], base + index->MP_index.common.MP_entry.EXIFtag.value_offset + i * sizeof(MP_Entry), sizeof(MP_Entry));

        printf("image type: 0x%08x\n", index->MP_data.MP_entries[i].image_attribute);
        cout << "image offset: " << index->MP_data.MP_entries[i].image_offset << "\n";
        cout << "image size: " << index->MP_data.MP_entries[i].image_size << "\n";

        if ((index->MP_data.MP_entries[i].image_attribute & 0xffffff) != DISPARITY_MAGIC)
            throw non_disparity_image;
    }

    return 0;
}

static int parse_attributes_IFD(unsigned char* buf, MP_Attributes_IFD* index)
{
    return 0;
}

static boolean parse_app2_marker(j_decompress_ptr cinfo)
{
    unsigned char* APP2_buf;
    unsigned int ret, length;

    length = jpeg_getc(cinfo) << 8;
    length += jpeg_getc(cinfo);
    length -= 2;

    try {
        APP2_buf = new unsigned char[length];

        copy_buffer(cinfo, APP2_buf, length);

        // Get the MP format identifier
        if (APP2_get32be(APP2_buf) != PMF_MAGIC)
            throw could_not_find_MPF_magic;

        // APP2_buf + 4 is the origin
        ret = APP2_get32be(APP2_buf + 4);

        switch (ret) {
        case 0x49492a00:
            APP2_get32 = APP2_get32le;
            break;
        case 0x4d4d002a:
            APP2_get32 = APP2_get32be;
            break;
        default:
            throw bad_byte_ordering;
        }

        int index_IFD_offset = APP2_get32(APP2_buf + 8);

        if (index_IFD_offset != 8)
            throw bad_IFD_index_offset;

        memset(&MP_index, 0, sizeof(MP_index));

        MP_index.MP_data.absolute_MPhdr_offset = ftell(input) - cinfo->src->bytes_in_buffer - length + 4;

        parse_index_IFD(APP2_buf + 4 + index_IFD_offset, &MP_index);

        //memset(&attributes, 0, sizeof(attributes));
        //parse_attributes_IFD(APP2_buf + index_IFD_offset, &attributes);
    }
    catch(int err) {
        printf("APP2_parse_error: %d\n", err);
    }

    delete [] APP2_buf;
    return 1;
}

int main(int argc, char** argv)
{
    unsigned char* image;
    JSAMPARRAY buffer;
    int row_stride;
    unsigned char *row_ptr;
    FILE *output;
    char buf[256];

    if (argc < 2)
        return -1;

    input = fopen(argv[1], "rb");
    if (!input)
        return -2;

    cinfo.err = jpeg_std_error(&jerr.pub);
    jerr.pub.error_exit = jpeglib_panic;

    if (setjmp(jerr.setjmp_buffer)) {
         jpeg_destroy_decompress(&cinfo);
         fclose(input);
         return -3;
    }

    jpeg_create_decompress(&cinfo);
    jpeg_stdio_src(&cinfo, input);

    jpeg_set_marker_processor(&cinfo, APP2, parse_app2_marker);

    if (jpeg_read_header(&cinfo, TRUE) != JPEG_HEADER_OK) {
        cout << "couldn't parse left JPEG header!\n";
        goto fini;
    }

    cout << "image #0 is: " << cinfo.image_width << "x" << cinfo.image_height << "\n";

    cinfo.output_components = 3;
    cinfo.out_color_space = JCS_RGB;

    jpeg_calc_output_dimensions(&cinfo);

    jpeg_start_decompress(&cinfo);

    row_stride = cinfo.output_width * 3;
    buffer = (*cinfo.mem->alloc_sarray)((j_common_ptr) &cinfo,
                                        JPOOL_IMAGE, row_stride, 1);

    image = (unsigned char*)calloc(1, cinfo.output_width * cinfo.output_height * 3);
    row_ptr = image;

    while (cinfo.output_scanline < cinfo.output_height) {
         jpeg_read_scanlines(&cinfo, buffer, 1);
         memcpy(row_ptr, buffer[0], cinfo.output_width * 3);
         row_ptr += cinfo.output_width * 3;
    }

    jpeg_finish_decompress(&cinfo);
    jpeg_destroy_decompress(&cinfo);

    sprintf(buf, "image_0_%dx%d_rgb24.raw", cinfo.output_width, cinfo.output_height);
    output = fopen(buf, "wb");
    fwrite(image, 1, cinfo.output_width * cinfo.output_height * 3, output);
    fclose(output);

    free(image);

    jpeg_create_decompress(&cinfo);

    fseek(input, MP_index.MP_data.absolute_MPhdr_offset + MP_index.MP_data.MP_entries[1].image_offset, SEEK_SET);
    jpeg_stdio_src(&cinfo, input);

    if (jpeg_read_header(&cinfo, TRUE) != JPEG_HEADER_OK) {
        cout << "couldn't parse right JPEG header!\n";
        goto fini;
    }

    cout << "image #1 is: " << cinfo.image_width << "x" << cinfo.image_height << "\n";

    cinfo.output_components = 3;
    cinfo.out_color_space = JCS_RGB;

    jpeg_calc_output_dimensions(&cinfo);

    jpeg_start_decompress(&cinfo);

    row_stride = cinfo.output_width * 3;
    buffer = (*cinfo.mem->alloc_sarray)((j_common_ptr) &cinfo,
                                        JPOOL_IMAGE, row_stride, 1);

    image = (unsigned char*)calloc(1, cinfo.output_width * cinfo.output_height * 3);
    row_ptr = image;

    while (cinfo.output_scanline < cinfo.output_height) {
         jpeg_read_scanlines(&cinfo, buffer, 1);
         memcpy(row_ptr, buffer[0], cinfo.output_width * 3);
         row_ptr += cinfo.output_width * 3;
    }

    jpeg_finish_decompress(&cinfo);
    jpeg_destroy_decompress(&cinfo);

    sprintf(buf, "image_1_%dx%d_rgb24.raw", cinfo.output_width, cinfo.output_height);
    output = fopen(buf, "wb");
    fwrite(image, 1, cinfo.output_width * cinfo.output_height * 3, output);
    fclose(output);

    free(image);

fini:
    fclose(input);
    return 0;
}
