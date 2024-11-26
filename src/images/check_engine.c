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

#ifndef LV_ATTRIBUTE_IMAGE_CHECK_ENGINE
#define LV_ATTRIBUTE_IMAGE_CHECK_ENGINE
#endif

const LV_ATTRIBUTE_MEM_ALIGN LV_ATTRIBUTE_LARGE_CONST LV_ATTRIBUTE_IMAGE_CHECK_ENGINE uint8_t check_engine_map[] = {
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xae, 0x94, 0xce, 0xb4, 0xce, 0xc4, 0xcd, 0xc4, 0xed, 0xc4, 0x0d, 0xbd, 0x0c, 0xbd, 0x0c, 0xc5, 0xeb, 0xdc, 0xa9, 0xcc, 0x28, 0xcd, 0xe9, 0xbc, 0xea, 0x93, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x8e, 0x94, 0x91, 0xcd, 0xcf, 0xe5, 0xce, 0xe5, 0xee, 0xe5, 0xec, 0xdd, 0xeb, 0xe5, 0xca, 0xed, 0xca, 0xf5, 0xac, 0xed, 0x0c, 0xe6, 0xad, 0xdd, 0xe8, 0x7a, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x10, 0xbd, 0x8c, 0xb4, 0x68, 0x93, 0x4a, 0xb4, 0xec, 0xed, 0xc9, 0xf5, 0xe9, 0xfd, 0x2a, 0xdd, 0x2a, 0xac, 0xf0, 0xe5, 0x2f, 0xc5, 0x01, 0x20, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x49, 0x83, 0x2c, 0xcd, 0x8c, 0xdd, 0x8b, 0xe5, 0x6a, 0xe5, 0xca, 0xfd, 0xc8, 0xfd, 0xc8, 0xfd, 0x89, 0xf5, 0x2a, 0xe5, 0x8c, 0xe5, 0x8d, 0xdd, 0x0d, 0xbd, 0x04, 0x4a, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x8a, 0x83, 0x89, 0x9b, 0x89, 0xa3, 0x8d, 0xe5, 0xea, 0xf5, 0xe9, 0xf5, 0xc9, 0xfd, 0xc9, 0xfd, 0xc8, 0xfd, 0xa8, 0xfd, 0xc8, 0xfd, 0xc8, 0xfd, 0xe7, 0xfd, 0xc8, 0xfd, 0xa9, 0xfd, 0xef, 0xf5, 0x0d, 0xa4, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0x54, 0xe6, 0x70, 0xc5, 0x8a, 0x7b, 0x2d, 0xbd, 0xcd, 0xed, 0xac, 0xf5, 0xe9, 0xfd, 0xe6, 0xfd, 0xc7, 0xfd, 0xa7, 0xfd, 0xa8, 0xfd, 0xa8, 0xfd, 0xa8, 0xfd, 0xa9, 0xfd, 0xa9, 0xfd, 0xc7, 0xfd, 0xc6, 0xfd, 0xa8, 0xfd, 0xcd, 0xfd, 0x6c, 0xbc, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0x50, 0xe6, 0x2f, 0xe6, 0xa9, 0x83, 0x2c, 0xcd, 0xc9, 0xfd, 0x88, 0xfd, 0xc7, 0xfd, 0xc6, 0xfd, 0xc7, 0xfd, 0xc8, 0xfd, 0xc8, 0xfd, 0xc8, 0xfd, 0xc8, 0xfd, 0xc8, 0xfd, 0xc8, 0xfd, 0xe6, 0xfd, 0xc7, 0xfd, 0x89, 0xfd, 0xc7, 0xfd, 0x87, 0xe5, 0x0b, 0xc5, 0x6b, 0xb4, 0x48, 0x8b, 0x2d, 0xcd, 0x6f, 0xd5, 0x4f, 0xcd, 0x6d, 0xac, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0x2e, 0xe6, 0xed, 0xdd, 0xa8, 0x93, 0x2b, 0xd5, 0xc9, 0xfd, 0x89, 0xfd, 0xa8, 0xfd, 0xa8, 0xfd, 0xa9, 0xfd, 0xa9, 0xfd, 0xa9, 0xfd, 0xc9, 0xfd, 0xa9, 0xfd, 0xa8, 0xfd, 0xc8, 0xfd, 0xc8, 0xfd, 0xa9, 0xfd, 0x89, 0xfd, 0xc7, 0xfd, 0xe7, 0xfd, 0x0a, 0xfe, 0x69, 0xe5, 0x29, 0xdd, 0xeb, 0xf5, 0xcb, 0xf5, 0xec, 0xf5, 0xad, 0xe5, 0xe8, 0x9b, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0x70, 0xee, 0x0e, 0xe6, 0x47, 0x8b, 0x0a, 0xd5, 0xe9, 0xfd, 0xa8, 0xfd, 0xa8, 0xfd, 0xa7, 0xfd, 0xa8, 0xfd, 0xa8, 0xfd, 0xa8, 0xfd, 0xc8, 0xfd, 0xa8, 0xfd, 0xa8, 0xfd, 0xa8, 0xfd, 0xa8, 0xfd, 0xa8, 0xfd, 0xa8, 0xfd, 0xc8, 0xfd, 0xa8, 0xfd, 0xa8, 0xfd, 0xa8, 0xfd, 0xc8, 0xfd, 0xa8, 0xfd, 0xc8, 0xfd, 0xc8, 0xfd, 0xca, 0xf5, 0x69, 0xac, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0x2e, 0xee, 0x0d, 0xee, 0x6a, 0xbc, 0x4a, 0xe5, 0xc8, 0xfd, 0xa8, 0xfd, 0xa8, 0xfd, 0xa7, 0xfd, 0xa8, 0xfd, 0xa8, 0xfd, 0xa8, 0xfd, 0xc8, 0xfd, 0xa8, 0xfd, 0xa8, 0xfd, 0xa8, 0xfd, 0xa8, 0xfd, 0xa8, 0xfd, 0xa8, 0xfd, 0xc8, 0xfd, 0xa8, 0xfd, 0xa8, 0xfd, 0xa8, 0xfd, 0xa8, 0xfd, 0xa8, 0xfd, 0xa8, 0xfd, 0xc8, 0xfd, 0xea, 0xfd, 0x0b, 0xc5, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0xca, 0xbc, 0x0b, 0xf6, 0xee, 0xfd, 0xab, 0xfd, 0xa6, 0xfd, 0xa9, 0xfd, 0xa8, 0xfd, 0xa8, 0xfd, 0xa8, 0xfd, 0xa8, 0xfd, 0xa8, 0xfd, 0xc8, 0xfd, 0xc8, 0xfd, 0xa8, 0xfd, 0xa8, 0xfd, 0xa8, 0xfd, 0xa8, 0xfd, 0xa8, 0xfd, 0xc8, 0xfd, 0xa8, 0xfd, 0xa8, 0xfd, 0xa8, 0xfd, 0xa8, 0xfd, 0xa8, 0xfd, 0xa8, 0xfd, 0xc8, 0xfd, 0xea, 0xfd, 0x2b, 0xcd, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0xed, 0xe5, 0xcb, 0xed, 0x48, 0xbc, 0x29, 0xe5, 0xa7, 0xfd, 0x89, 0xfd, 0xa9, 0xfd, 0xa8, 0xfd, 0xa8, 0xfd, 0xa8, 0xfd, 0xc8, 0xfd, 0xc8, 0xfd, 0xc8, 0xfd, 0xc8, 0xfd, 0xa8, 0xfd, 0xa8, 0xfd, 0xa8, 0xfd, 0xc8, 0xfd, 0xc8, 0xfd, 0xc8, 0xfd, 0xc8, 0xfd, 0xa8, 0xfd, 0xa8, 0xfd, 0xa8, 0xfd, 0xc8, 0xfd, 0xc8, 0xfd, 0xe9, 0xfd, 0x0a, 0xcd, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0x2f, 0xee, 0x0e, 0xee, 0xc5, 0x82, 0xc9, 0xd4, 0xc9, 0xfd, 0x89, 0xfd, 0xa9, 0xfd, 0xa8, 0xfd, 0xc8, 0xfd, 0xc8, 0xfd, 0xc8, 0xfd, 0xc8, 0xfd, 0xc8, 0xfd, 0xc8, 0xfd, 0xc8, 0xfd, 0xc8, 0xfd, 0xc8, 0xfd, 0xc8, 0xfd, 0xc8, 0xfd, 0xc8, 0xfd, 0xc8, 0xfd, 0xc8, 0xfd, 0xc8, 0xfd, 0xc8, 0xfd, 0xc8, 0xfd, 0xc8, 0xfd, 0xe9, 0xfd, 0x0a, 0xcd, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0x31, 0xee, 0xef, 0xe5, 0x88, 0x93, 0xeb, 0xd4, 0xca, 0xfd, 0x8a, 0xfd, 0x89, 0xfd, 0xa8, 0xfd, 0xc8, 0xfd, 0xc8, 0xfd, 0xc8, 0xfd, 0xc8, 0xfd, 0xc8, 0xfd, 0xc8, 0xfd, 0xc8, 0xfd, 0xc8, 0xfd, 0xc8, 0xfd, 0xc8, 0xfd, 0xc8, 0xfd, 0xc8, 0xfd, 0xc8, 0xfd, 0xc8, 0xfd, 0xc8, 0xfd, 0xc8, 0xfd, 0xc8, 0xfd, 0xc8, 0xfd, 0xe9, 0xfd, 0x0b, 0xcd, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0xb2, 0xee, 0x0f, 0xde, 0x87, 0x93, 0x0a, 0xd5, 0x09, 0xfe, 0xc8, 0xfd, 0xa8, 0xfd, 0xa8, 0xfd, 0xc8, 0xfd, 0xc8, 0xfd, 0xc8, 0xfd, 0xc8, 0xfd, 0xc8, 0xfd, 0xc8, 0xfd, 0xc8, 0xfd, 0xc8, 0xfd, 0xc8, 0xfd, 0xc8, 0xfd, 0xc8, 0xfd, 0xc8, 0xfd, 0xc8, 0xfd, 0xc9, 0xfd, 0xa9, 0xfd, 0xc7, 0xfd, 0xc6, 0xfd, 0xc7, 0xfd, 0xea, 0xfd, 0x0b, 0xcd, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0xf5, 0xde, 0x31, 0xde, 0x68, 0x8b, 0xcc, 0xc4, 0x2c, 0xee, 0xea, 0xf5, 0xea, 0xfd, 0xc9, 0xfd, 0xc8, 0xfd, 0xc8, 0xfd, 0xc8, 0xfd, 0xc8, 0xfd, 0xc8, 0xfd, 0xc8, 0xfd, 0xc8, 0xfd, 0xc8, 0xfd, 0xc8, 0xfd, 0xc8, 0xfd, 0xc8, 0xfd, 0xc8, 0xfd, 0xc7, 0xfd, 0xe9, 0xfd, 0xc9, 0xfd, 0xc6, 0xfd, 0xc6, 0xfd, 0xa8, 0xfd, 0xcb, 0xfd, 0xec, 0xc4, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0xff, 0xff, 0xbb, 0xde, 0xa5, 0x28, 0xac, 0x93, 0xac, 0xb4, 0xe8, 0xa3, 0x6c, 0xdd, 0xeb, 0xfd, 0xc8, 0xfd, 0xc8, 0xfd, 0xc8, 0xfd, 0xc8, 0xfd, 0xc8, 0xfd, 0xc8, 0xfd, 0xa8, 0xfd, 0xa8, 0xfd, 0xa8, 0xfd, 0xc8, 0xfd, 0xc7, 0xfd, 0xc7, 0xfd, 0xc8, 0xfd, 0x89, 0xf5, 0x6a, 0xed, 0xa8, 0xfd, 0xa7, 0xfd, 0xa9, 0xfd, 0x8c, 0xf5, 0x0a, 0xac, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x29, 0x83, 0x2b, 0xdd, 0xca, 0xfd, 0xa9, 0xfd, 0xa7, 0xfd, 0xa7, 0xfd, 0xa7, 0xfd, 0xa7, 0xfd, 0xa7, 0xfd, 0xa8, 0xfd, 0xa8, 0xfd, 0xa7, 0xfd, 0xa7, 0xfd, 0xa7, 0xfd, 0xc9, 0xfd, 0x69, 0xcc, 0x89, 0xab, 0xed, 0xfd, 0xca, 0xf5, 0xcb, 0xf5, 0xad, 0xe5, 0xeb, 0xa3, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xc2, 0x59, 0x29, 0xb4, 0xad, 0xed, 0xab, 0xfd, 0xc8, 0xfd, 0xc8, 0xfd, 0xc8, 0xfd, 0xc9, 0xfd, 0xc9, 0xfd, 0xc9, 0xfd, 0xc9, 0xfd, 0xc9, 0xfd, 0xc9, 0xfd, 0xec, 0xfd, 0xcc, 0xcc, 0x29, 0x8b, 0x4e, 0xcd, 0x6d, 0xd5, 0x8e, 0xcd, 0x8b, 0xac, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x8a, 0x93, 0x2e, 0xcd, 0x6c, 0xd5, 0x6c, 0xd5, 0x4b, 0xd5, 0x4b, 0xd5, 0x6b, 0xd5, 0x4b, 0xd5, 0x2b, 0xd5, 0x2b, 0xd5, 0x2b, 0xd5, 0x6e, 0xd5, 0x8d, 0xb4, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x6d, 0x7b, 0x70, 0x9c, 0x11, 0xad, 0xcc, 0x83, 0xa6, 0x5a, 0x49, 0x73, 0x00, 0x00, 0x00, 0x00, 0xc5, 0x39, 0x06, 0x42, 0x00, 0x08, 0x88, 0x62, 0x2b, 0x73, 0x4b, 0x73, 0x47, 0x52, 0x00, 0x00, 0x20, 0x10, 0x2a, 0x73, 0x2f, 0x94, 0x6f, 0x94, 0x6d, 0x52, 0xae, 0x73, 0x00, 0x00, 0x00, 0x00, 0xce, 0x7b, 0x6c, 0x6b, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x4a, 0x8b, 0x31, 0xbd, 0xf1, 0xac, 0x11, 0xad, 0x2c, 0x8c, 0x6c, 0xac, 0xce, 0xb4, 0x00, 0x00, 0x00, 0x00, 0x52, 0xde, 0x31, 0xd6, 0x2e, 0xa4, 0x2f, 0xc5, 0x8f, 0x9c, 0x8f, 0x9c, 0xac, 0x83, 0x88, 0x5a, 0x0e, 0xad, 0x0e, 0xb5, 0x8e, 0x9c, 0xaf, 0x9c, 0xd0, 0x9c, 0x34, 0xce, 0x00, 0x00, 0xf3, 0xcd, 0x52, 0xb5, 0x49, 0x42, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x32, 0xc5, 0xb0, 0xb4, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xee, 0xc4, 0xae, 0xbc, 0x00, 0x00, 0x00, 0x00, 0x52, 0xde, 0x52, 0xde, 0x0f, 0xac, 0xee, 0xcc, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x34, 0xc5, 0x33, 0xbd, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xd1, 0xee, 0x90, 0xe6, 0x8c, 0xac, 0x30, 0xbd, 0x0c, 0x8c, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0xae, 0xa4, 0x30, 0xb5, 0x66, 0x73, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xac, 0xbc, 0x4e, 0xcd, 0x6b, 0x94, 0x8d, 0x9c, 0x0e, 0xbd, 0x50, 0xde, 0xee, 0xb3, 0x4e, 0xd5, 0xcf, 0xac, 0x91, 0x9c, 0xa9, 0x62, 0x52, 0xcd, 0xad, 0x93, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x12, 0xf7, 0x0d, 0xde, 0x6f, 0xcd, 0x2b, 0x9c, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0xce, 0xac, 0x4f, 0xbd, 0xa7, 0x7b, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xec, 0xc4, 0x2e, 0xcd, 0xc9, 0x83, 0xeb, 0x83, 0x2d, 0xbd, 0x2f, 0xde, 0x0c, 0xb4, 0x4d, 0xd5, 0x2c, 0x94, 0x2f, 0x94, 0x89, 0x5a, 0xb1, 0xd5, 0x49, 0x8b, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xf5, 0xf6, 0xf0, 0xdd, 0x8c, 0xb4, 0x4f, 0xc5, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x32, 0xc5, 0x51, 0xbd, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xed, 0xc4, 0xef, 0xbc, 0x00, 0x00, 0x00, 0x00, 0x31, 0xe6, 0x10, 0xd6, 0x8f, 0xbc, 0x0e, 0xcd, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x31, 0xbd, 0xb0, 0xac, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x54, 0xde, 0x14, 0xd6, 0x00, 0x00, 0xef, 0xac, 0x31, 0xad, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x8f, 0xac, 0x71, 0xc5, 0xce, 0xb4, 0x0f, 0xad, 0x6e, 0x9c, 0x4d, 0x9c, 0xf0, 0xac, 0x00, 0x00, 0x00, 0x00, 0x51, 0xd6, 0x30, 0xce, 0xee, 0x93, 0x0e, 0xc5, 0x4d, 0xac, 0xae, 0xa4, 0xcb, 0x73, 0x47, 0x52, 0x30, 0xb5, 0xef, 0xb4, 0x4e, 0xa4, 0x0f, 0x94, 0xcd, 0x83, 0x35, 0xce, 0x00, 0x00, 0x0d, 0x8c, 0x13, 0xad, 0xd0, 0x83, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xa4, 0x6a, 0x0c, 0x9c, 0x6d, 0xa4, 0xcc, 0x83, 0x89, 0x5a, 0xeb, 0x6a, 0x00, 0x00, 0x00, 0x00, 0xaa, 0x7b, 0xeb, 0x7b, 0x89, 0x41, 0x29, 0x83, 0x8b, 0x93, 0x2c, 0x94, 0x8b, 0x6b, 0x00, 0x00, 0x80, 0x39, 0xec, 0x93, 0x4e, 0xa4, 0xcd, 0x8b, 0x2a, 0x53, 0x0e, 0x7c, 0xff, 0xff, 0x00, 0x00, 0xcc, 0x62, 0xed, 0x6a, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 

  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x07, 0x66, 0x72, 0x70, 0x70, 0x70, 0x70, 0x70, 0x70, 0x70, 0x70, 0x6f, 0x17, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x10, 0xf9, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x81, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0x3d, 0x6c, 0xb6, 0xe8, 0xff, 0xff, 0xff, 0xff, 0x73, 0x42, 0x4b, 0x25, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x61, 0xda, 0xd1, 0xce, 0xf1, 0xff, 0xff, 0xff, 0xff, 0xdc, 0xd0, 0xd2, 0xcd, 0x33, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x1f, 0x02, 0x44, 0x5f, 0xb0, 0xfa, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xb8, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x1b, 0xf0, 0x82, 0xde, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xd6, 0x2b, 0x00, 0x00, 0x05, 0x04, 0x05, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x29, 0xff, 0xa6, 0xee, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xf8, 0xc0, 0x9c, 0xe0, 0xdc, 0xe0, 0xbe, 0x0c, 0x00, 0x00, 
  0x00, 0x00, 0x27, 0xff, 0xa2, 0xeb, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x39, 0x00, 0x00, 
  0x00, 0x00, 0x27, 0xff, 0xa5, 0xec, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xa9, 0x00, 0x00, 
  0x00, 0x00, 0x26, 0xfd, 0xf4, 0xfc, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xe8, 0x0c, 0x00, 
  0x00, 0x00, 0x50, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xe5, 0x0b, 0x00, 
  0x00, 0x00, 0x2a, 0xfd, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xe5, 0x0b, 0x00, 
  0x00, 0x00, 0x27, 0xff, 0xd2, 0xf5, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xe5, 0x0b, 0x00, 
  0x00, 0x00, 0x27, 0xff, 0x9e, 0xea, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xe5, 0x0b, 0x00, 
  0x00, 0x00, 0x28, 0xff, 0xa5, 0xeb, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xe5, 0x0b, 0x00, 
  0x00, 0x00, 0x23, 0xe3, 0x94, 0xf7, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xe9, 0x0c, 0x00, 
  0x00, 0x00, 0x01, 0x06, 0x0e, 0x53, 0x60, 0xcb, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xaa, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02, 0x5b, 0xe1, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x3a, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x3c, 0xd8, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfb, 0xb5, 0xdf, 0xde, 0xe2, 0xc0, 0x0d, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x15, 0x99, 0xff, 0xfd, 0xff, 0xf7, 0xf8, 0xf7, 0xfd, 0xff, 0xff, 0xf9, 0xf7, 0x93, 0x00, 0x00, 0x06, 0x06, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0x13, 0x67, 0x6e, 0x52, 0x2e, 0x33, 0x00, 0x00, 0x52, 0x68, 0x45, 0x97, 0x9a, 0x9e, 0x62, 0x29, 0x46, 0x98, 0x98, 0x53, 0x0a, 0x46, 0x00, 0x00, 0x48, 0x13, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x62, 0xe8, 0xca, 0xae, 0xc4, 0x7f, 0xd7, 0x00, 0x00, 0x50, 0x8a, 0x2d, 0xff, 0xe2, 0xe9, 0x5e, 0x17, 0xdc, 0xe2, 0xe4, 0xc8, 0x28, 0xc2, 0x00, 0x60, 0xea, 0x24, 0x00, 0x00, 
  0x00, 0x00, 0x04, 0xd6, 0x95, 0x00, 0x00, 0x00, 0x66, 0xdf, 0x00, 0x00, 0x52, 0x8a, 0x34, 0xf3, 0x15, 0x00, 0x00, 0xa0, 0xb2, 0x00, 0x04, 0x00, 0x13, 0xb6, 0x5b, 0xfc, 0x42, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x2a, 0xf9, 0x1d, 0x00, 0x00, 0x00, 0x63, 0xff, 0xdb, 0xd9, 0xf0, 0x83, 0x31, 0xff, 0xdd, 0xd2, 0x64, 0xdf, 0x63, 0x00, 0x00, 0x00, 0x10, 0xdf, 0xff, 0x95, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x2a, 0xfa, 0x1d, 0x00, 0x00, 0x00, 0x65, 0xf3, 0x7a, 0x75, 0xab, 0x86, 0x32, 0xfb, 0x84, 0x71, 0x34, 0xe2, 0x76, 0x00, 0x00, 0x00, 0x0e, 0xea, 0xd6, 0xdd, 0x0a, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x1b, 0xf2, 0x86, 0x00, 0x00, 0x00, 0x66, 0xdc, 0x00, 0x00, 0x4b, 0x8b, 0x34, 0xf2, 0x07, 0x00, 0x00, 0xbd, 0xdd, 0x16, 0x00, 0x00, 0x13, 0xc8, 0x2d, 0xdb, 0x9b, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x41, 0xed, 0xdb, 0xb8, 0xba, 0x7d, 0xe5, 0x00, 0x00, 0x57, 0x91, 0x34, 0xff, 0xe1, 0xe5, 0x82, 0x3c, 0xf1, 0xf0, 0xdc, 0xd8, 0x3f, 0xbe, 0x00, 0x2e, 0xfc, 0x50, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0x1b, 0x74, 0x7b, 0x5f, 0x33, 0x60, 0x00, 0x00, 0x25, 0x3d, 0x15, 0x73, 0x76, 0x7b, 0x43, 0x00, 0x1a, 0x72, 0x79, 0x61, 0x0d, 0x52, 0x01, 0x00, 0x5a, 0x45, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
};

const lv_image_dsc_t check_engine = {
  .header.cf = LV_COLOR_FORMAT_RGB565A8,
  .header.magic = LV_IMAGE_HEADER_MAGIC,
  .header.w = 32,
  .header.h = 32,
  .data_size = 1024 * 3,
  .data = check_engine_map,
};