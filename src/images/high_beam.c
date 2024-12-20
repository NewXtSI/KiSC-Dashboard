#ifdef __has_include
    #if __has_include("lvgl.h")
        #ifndef LV_LVGL_H_INCLUDE_SIMPLE
            #define LV_LVGL_H_INCLUDE_SIMPLE
        #endif
    #endif
#endif

#if defined(LV_LVGL_H_INCLUDE_SIMPLE)
    #include "lvgl.h"
#else
    #include "lvgl/lvgl.h"
#endif


#ifndef LV_ATTRIBUTE_MEM_ALIGN
#define LV_ATTRIBUTE_MEM_ALIGN
#endif

#ifndef LV_ATTRIBUTE_IMAGE_HIGH_BEAM
#define LV_ATTRIBUTE_IMAGE_HIGH_BEAM
#endif

const LV_ATTRIBUTE_MEM_ALIGN LV_ATTRIBUTE_LARGE_CONST LV_ATTRIBUTE_IMAGE_HIGH_BEAM uint8_t high_beam_map[] = {
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xa7, 0x00, 0x10, 0x12, 0x52, 0x12, 0xd4, 0x22, 0x15, 0x23, 0x93, 0x1a, 0x72, 0x12, 0x11, 0x0a, 0x4d, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0x09, 0x00, 0x6b, 0x00, 0x2b, 0x00, 0x0b, 0x00, 0x0b, 0x00, 0x2b, 0x00, 0x8c, 0x00, 0xec, 0x00, 0x0a, 0x01, 0xe9, 0x08, 0x00, 0x30, 0x10, 0x12, 0xd4, 0x12, 0x99, 0x1b, 0xbb, 0x1b, 0xdc, 0x1b, 0xdc, 0x1b, 0xbc, 0x23, 0x9c, 0x23, 0x9c, 0x23, 0x5a, 0x23, 0xd4, 0x22, 0x8a, 0x09, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x91, 0x22, 0x75, 0x3b, 0x97, 0x33, 0x77, 0x33, 0x57, 0x33, 0x57, 0x33, 0x58, 0x33, 0x78, 0x2b, 0x78, 0x2b, 0x97, 0x2b, 0x76, 0x33, 0x90, 0x21, 0xd6, 0x22, 0xfc, 0x23, 0xfe, 0x03, 0x1f, 0x0c, 0x1f, 0x0c, 0x1f, 0x0c, 0xdf, 0x13, 0xdf, 0x1b, 0xdf, 0x1b, 0xdf, 0x23, 0xbb, 0x2b, 0x57, 0x2b, 0x52, 0x1a, 0x07, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x90, 0x22, 0x74, 0x33, 0x55, 0x2b, 0x76, 0x2b, 0x76, 0x2b, 0x76, 0x2b, 0x56, 0x2b, 0x57, 0x2b, 0x76, 0x2b, 0x75, 0x2b, 0x54, 0x33, 0xaf, 0x19, 0x38, 0x23, 0x1e, 0x14, 0x1f, 0x04, 0x1f, 0x04, 0xff, 0x0b, 0xff, 0x0b, 0xff, 0x0b, 0xff, 0x13, 0xff, 0x13, 0xff, 0x0b, 0xfe, 0x0b, 0xde, 0x23, 0x7b, 0x2b, 0x73, 0x1a, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xc8, 0x00, 0x36, 0x23, 0xfd, 0x13, 0x3f, 0x04, 0x1f, 0x04, 0x1f, 0x04, 0x1f, 0x04, 0xff, 0x03, 0xff, 0x0b, 0xff, 0x03, 0x1f, 0x04, 0x3e, 0x04, 0xff, 0x03, 0xbe, 0x13, 0x99, 0x23, 0xd3, 0x22, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x08, 0x00, 0xcc, 0x19, 0x4f, 0x22, 0x51, 0x12, 0x32, 0x12, 0x31, 0x1a, 0x31, 0x1a, 0x31, 0x12, 0x34, 0x1a, 0x34, 0x1a, 0x53, 0x12, 0x32, 0x1a, 0x0d, 0x09, 0x58, 0x2b, 0xfd, 0x13, 0x1f, 0x04, 0x1f, 0x04, 0x1f, 0x04, 0x1f, 0x04, 0x1f, 0x04, 0x1f, 0x04, 0x1f, 0x04, 0x3f, 0x04, 0x3e, 0x04, 0x1f, 0x04, 0x1e, 0x0c, 0x1c, 0x1c, 0x99, 0x23, 0xf0, 0x11, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x90, 0x22, 0xd6, 0x33, 0xf8, 0x23, 0xd9, 0x23, 0xb8, 0x2b, 0xb8, 0x2b, 0xb8, 0x2b, 0x9b, 0x2b, 0x9b, 0x2b, 0xba, 0x2b, 0x99, 0x33, 0xd2, 0x21, 0x59, 0x2b, 0xfd, 0x13, 0x1f, 0x04, 0x3f, 0x04, 0x1f, 0x04, 0x1f, 0x04, 0x1f, 0x04, 0x1f, 0x04, 0x1f, 0x04, 0x1f, 0x04, 0x1e, 0x04, 0x1e, 0x04, 0x3e, 0x04, 0x1e, 0x04, 0xde, 0x23, 0xb7, 0x22, 0x08, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x8f, 0x22, 0xd0, 0x2a, 0x13, 0x1b, 0x13, 0x1b, 0xf3, 0x22, 0xf3, 0x22, 0xf3, 0x22, 0xd5, 0x22, 0xd5, 0x22, 0xd4, 0x22, 0xd2, 0x2a, 0x6d, 0x11, 0x57, 0x2b, 0xfd, 0x13, 0x1f, 0x04, 0x1f, 0x04, 0x1f, 0x04, 0x1f, 0x04, 0x1f, 0x04, 0x1f, 0x04, 0x1f, 0x04, 0xff, 0x0b, 0x1f, 0x0c, 0x1e, 0x04, 0x3f, 0x04, 0x3f, 0x04, 0xdf, 0x0b, 0x7b, 0x2b, 0x4f, 0x1a, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x06, 0x00, 0x77, 0x2b, 0xfd, 0x13, 0xff, 0x03, 0x1f, 0x04, 0x1f, 0x04, 0x1f, 0x04, 0x1f, 0x04, 0x1f, 0x04, 0x1f, 0x04, 0x1f, 0x0c, 0xff, 0x0b, 0x1f, 0x04, 0x1f, 0x04, 0x3f, 0x04, 0x1f, 0x04, 0xdc, 0x23, 0xd2, 0x22, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0xf0, 0x19, 0x15, 0x23, 0x37, 0x13, 0x37, 0x13, 0x37, 0x13, 0x37, 0x1b, 0x37, 0x1b, 0x38, 0x1b, 0xf7, 0x22, 0x16, 0x2b, 0x15, 0x23, 0x71, 0x19, 0x59, 0x2b, 0xfe, 0x0b, 0xff, 0x0b, 0x1f, 0x04, 0x1f, 0x04, 0x1f, 0x04, 0x1f, 0x04, 0x1f, 0x04, 0x1f, 0x04, 0x1f, 0x04, 0x1f, 0x04, 0x1f, 0x04, 0x1f, 0x04, 0x1f, 0x04, 0x1f, 0x04, 0xfc, 0x23, 0x34, 0x2b, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x30, 0x22, 0xd8, 0x2b, 0x1a, 0x24, 0x1a, 0x24, 0xfa, 0x23, 0xfa, 0x23, 0xfa, 0x23, 0xb9, 0x2b, 0x99, 0x3b, 0x97, 0x3b, 0xb7, 0x3b, 0xd2, 0x29, 0x39, 0x23, 0x1e, 0x0c, 0xff, 0x0b, 0x1f, 0x04, 0x1f, 0x04, 0x1f, 0x04, 0x1f, 0x04, 0x1f, 0x04, 0x1f, 0x04, 0x1f, 0x04, 0x1f, 0x04, 0xff, 0x03, 0x1f, 0x04, 0x1f, 0x04, 0xff, 0x03, 0xdc, 0x23, 0x55, 0x2b, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0xcb, 0x21, 0xcb, 0x19, 0x0d, 0x12, 0xed, 0x09, 0xcc, 0x09, 0xac, 0x09, 0xac, 0x09, 0xac, 0x11, 0x8b, 0x19, 0x89, 0x19, 0xa9, 0x11, 0xcb, 0x08, 0x77, 0x23, 0xfd, 0x0b, 0xff, 0x0b, 0x1f, 0x04, 0x1f, 0x04, 0x1f, 0x04, 0x1f, 0x04, 0x1f, 0x04, 0x1f, 0x04, 0x1f, 0x04, 0x1f, 0x04, 0xff, 0x0b, 0x1f, 0x0c, 0x1e, 0x04, 0x1f, 0x0c, 0xdc, 0x23, 0x14, 0x2b, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x2b, 0x11, 0xcf, 0x11, 0xb2, 0x01, 0xd3, 0x09, 0xf2, 0x09, 0x12, 0x0a, 0x13, 0x0a, 0x13, 0x0a, 0x33, 0x02, 0x52, 0x02, 0x71, 0x12, 0xcd, 0x08, 0x98, 0x23, 0x1d, 0x0c, 0xff, 0x03, 0x1f, 0x04, 0x1f, 0x04, 0x1f, 0x04, 0x1f, 0x04, 0x1f, 0x04, 0x1f, 0x04, 0x1f, 0x04, 0x1f, 0x04, 0xff, 0x0b, 0x1f, 0x0c, 0x1e, 0x04, 0x1f, 0x14, 0x9c, 0x2b, 0x31, 0x1a, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0xb1, 0x22, 0xb7, 0x2b, 0xb9, 0x1b, 0x99, 0x1b, 0x98, 0x1b, 0x98, 0x1b, 0x98, 0x1b, 0x59, 0x23, 0x59, 0x23, 0x78, 0x2b, 0x77, 0x33, 0x91, 0x21, 0x7a, 0x23, 0x1e, 0x0c, 0xff, 0x03, 0x1f, 0x04, 0x1f, 0x04, 0x1f, 0x04, 0x1f, 0x04, 0x1f, 0x04, 0x1f, 0x04, 0x1f, 0x04, 0x1f, 0x04, 0xff, 0x0b, 0x1f, 0x04, 0x1e, 0x04, 0xfe, 0x1b, 0x39, 0x2b, 0xcc, 0x08, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0xf0, 0x2a, 0x38, 0x34, 0x5a, 0x24, 0x3a, 0x24, 0x19, 0x24, 0x19, 0x24, 0x19, 0x24, 0xda, 0x33, 0xb9, 0x3b, 0xf8, 0x3b, 0xb7, 0x3b, 0x90, 0x19, 0x78, 0x1b, 0x1e, 0x04, 0xff, 0x03, 0x1f, 0x04, 0x1f, 0x04, 0x1f, 0x04, 0x1f, 0x04, 0x1f, 0x04, 0x1f, 0x04, 0x1f, 0x04, 0x1f, 0x04, 0xff, 0x0b, 0x1f, 0x04, 0x1e, 0x04, 0xdd, 0x1b, 0x55, 0x12, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x06, 0x00, 0x58, 0x23, 0xff, 0x13, 0xff, 0x03, 0x1f, 0x04, 0x1f, 0x04, 0x1f, 0x04, 0x1f, 0x04, 0x1f, 0x04, 0x1f, 0x04, 0x1f, 0x04, 0x1f, 0x04, 0x1f, 0x0c, 0x1f, 0x0c, 0xfd, 0x1b, 0xf7, 0x1a, 0x0e, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x07, 0x00, 0xef, 0x19, 0x16, 0x2b, 0x37, 0x1b, 0x38, 0x1b, 0x18, 0x1b, 0x18, 0x1b, 0x18, 0x1b, 0x19, 0x1b, 0x18, 0x1b, 0x16, 0x23, 0x13, 0x23, 0xad, 0x11, 0x79, 0x2b, 0xff, 0x1b, 0xff, 0x03, 0x1f, 0x04, 0x1f, 0x04, 0x1f, 0x04, 0x1f, 0x04, 0xff, 0x0b, 0xff, 0x0b, 0xff, 0x0b, 0xff, 0x0b, 0xfe, 0x13, 0x1b, 0x24, 0x36, 0x23, 0x2c, 0x09, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x05, 0x00, 0xf0, 0x11, 0x98, 0x33, 0xda, 0x23, 0xbb, 0x23, 0x9b, 0x23, 0x9b, 0x2b, 0x9b, 0x23, 0x9d, 0x23, 0x9c, 0x23, 0x9a, 0x2b, 0x97, 0x33, 0x2f, 0x22, 0x36, 0x23, 0x1c, 0x24, 0xfe, 0x03, 0x1f, 0x04, 0x1f, 0x04, 0x1f, 0x0c, 0xff, 0x0b, 0xff, 0x0b, 0xff, 0x0b, 0xff, 0x0b, 0xdf, 0x1b, 0xdb, 0x2b, 0x12, 0x23, 0xea, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x05, 0x00, 0x0e, 0x1a, 0x55, 0x33, 0x77, 0x2b, 0x77, 0x23, 0x79, 0x2b, 0x72, 0x1a, 0x31, 0x12, 0x12, 0x1a, 0x12, 0x1a, 0x30, 0x1a, 0x71, 0x22, 0x6c, 0x19, 0x30, 0x12, 0x77, 0x23, 0x1c, 0x1c, 0x3e, 0x0c, 0x3e, 0x0c, 0x1e, 0x0c, 0x1e, 0x14, 0x3e, 0x14, 0x1e, 0x14, 0xdd, 0x13, 0x59, 0x1b, 0x93, 0x1a, 0x06, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0f, 0x12, 0xb3, 0x12, 0x36, 0x13, 0x78, 0x13, 0x98, 0x1b, 0x78, 0x1b, 0x77, 0x1b, 0xf6, 0x12, 0x32, 0x0a, 0xae, 0x09, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xe8, 0x00, 0xcc, 0x11, 0x8b, 0x09, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 

  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x14, 0x0b, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1b, 0x5e, 0x9f, 0x9a, 0x98, 0xe6, 0xd1, 0x98, 0x8a, 0x06, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x08, 0x3c, 0x3c, 0x3b, 0x3b, 0x3b, 0x3b, 0x3b, 0x3b, 0x3b, 0x3e, 0x0a, 0x48, 0xf6, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xcb, 0x54, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x91, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xd9, 0xf3, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xa4, 0x22, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x0a, 0x9c, 0xf9, 0xf6, 0xf5, 0xf5, 0xf5, 0xf5, 0xf5, 0xf5, 0xf5, 0xf4, 0xfb, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xdf, 0x07, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x0f, 0x0f, 0x0f, 0x0f, 0x0f, 0x0f, 0x0f, 0x0f, 0x0e, 0x0e, 0x4c, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x9d, 0x00, 0x00, 0x00, 0x00, 
  0x04, 0x3b, 0x8f, 0x8f, 0x8e, 0x8e, 0x8e, 0x8e, 0x8e, 0x8e, 0x8c, 0x9d, 0xc5, 0xfe, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x92, 0x00, 0x00, 0x00, 
  0x02, 0xc4, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xf8, 0x26, 0x00, 0x00, 
  0x00, 0x3d, 0xb9, 0xb5, 0xb5, 0xb5, 0xb5, 0xb5, 0xb5, 0xb5, 0xb5, 0xb3, 0xc4, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x66, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x33, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xbb, 0x00, 0x00, 
  0x00, 0x67, 0xe5, 0xe0, 0xe0, 0xe0, 0xe0, 0xe0, 0xe0, 0xe0, 0xe0, 0xdf, 0xe8, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xb7, 0x00, 0x00, 
  0x17, 0xe2, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xf8, 0xfe, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xb7, 0x00, 0x00, 
  0x00, 0x12, 0x56, 0x56, 0x56, 0x56, 0x56, 0x56, 0x56, 0x56, 0x56, 0x52, 0x52, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xba, 0x00, 0x00, 
  0x00, 0x26, 0x3a, 0x38, 0x38, 0x38, 0x38, 0x38, 0x38, 0x38, 0x38, 0x32, 0x65, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x99, 0x00, 0x00, 
  0x00, 0xb2, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x3d, 0x00, 0x00, 
  0x0c, 0x82, 0x95, 0x93, 0x93, 0x93, 0x93, 0x93, 0x93, 0x93, 0x91, 0xa0, 0xc7, 0xfe, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xd9, 0x06, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x3a, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x85, 0x00, 0x00, 0x00, 
  0x13, 0x96, 0x9b, 0x9a, 0x9a, 0x9a, 0x9a, 0x9a, 0x9a, 0x9a, 0x98, 0xa8, 0xcc, 0xfe, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xf7, 0x86, 0x00, 0x00, 0x00, 0x00, 
  0x22, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xeb, 0x32, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x07, 0x3a, 0x3c, 0x3b, 0x3b, 0x36, 0x82, 0xb5, 0xb0, 0xb0, 0xb3, 0x82, 0x56, 0xa4, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xc4, 0x32, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x57, 0xef, 0xf7, 0xf5, 0xf5, 0xf5, 0xf5, 0xf7, 0xeb, 0x4e, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x16, 0x1b, 0x1b, 0x1b, 0x1b, 0x1b, 0x1b, 0x14, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
};

const lv_image_dsc_t high_beam = {
  .header.cf = LV_COLOR_FORMAT_RGB565A8,
  .header.magic = LV_IMAGE_HEADER_MAGIC,
  .header.w = 32,
  .header.h = 32,
  .data_size = 1024 * 3,
  .data = high_beam_map,
};
