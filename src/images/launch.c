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

#ifndef LV_ATTRIBUTE_IMAGE_LAUNCH
#define LV_ATTRIBUTE_IMAGE_LAUNCH
#endif

const LV_ATTRIBUTE_MEM_ALIGN LV_ATTRIBUTE_LARGE_CONST LV_ATTRIBUTE_IMAGE_LAUNCH uint8_t launch_map[] = {
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xbb, 0xde, 0xfc, 0xde, 0xfc, 0xe6, 0x1c, 0xe7, 0x1c, 0xe7, 0xbb, 0xde, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1c, 0xdf, 0xfc, 0xe6, 0x1c, 0xe7, 0x1c, 0xdf, 0x1c, 0xdf, 0x1c, 0xe7, 0x1c, 0xe7, 0x1c, 0xe7, 0xfc, 0xde, 0xdb, 0xde, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xfb, 0xde, 0xfc, 0xe6, 0x1c, 0xdf, 0x1c, 0xdf, 0x1c, 0xdf, 0x1c, 0xdf, 0x1c, 0xe7, 0x1c, 0xdf, 0x00, 0x00, 0xdb, 0xde, 0xfb, 0xde, 0xfc, 0xe6, 0xfc, 0xe6, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xfb, 0xde, 0xfb, 0xde, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1a, 0xd7, 0x1c, 0xe7, 0xfb, 0xde, 0xfb, 0xde, 0x1c, 0xdf, 0x1c, 0xdf, 0x1c, 0xdf, 0x1c, 0xdf, 0x1c, 0xdf, 0xfc, 0xde, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xfc, 0xe6, 0x1c, 0xe7, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xdb, 0xde, 0x1c, 0xdf, 0x1c, 0xe7, 0xba, 0xe6, 0xfb, 0xde, 0xfb, 0xde, 0x1c, 0xe7, 0xfc, 0xde, 0x00, 0x00, 0xfc, 0xe6, 0x1c, 0xdf, 0x1c, 0xdf, 0x1c, 0xdf, 0x1c, 0xdf, 0x1c, 0xdf, 0xdb, 0xde, 0x00, 0x00, 0x00, 0x00, 0xfc, 0xe6, 0x1c, 0xe7, 0xbb, 0xde, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xfc, 0xe6, 0x1c, 0xdf, 0x1c, 0xe7, 0x1c, 0xe7, 0x1c, 0xe7, 0x1c, 0xdf, 0x1c, 0xdf, 0xdb, 0xde, 0x00, 0x00, 0x1b, 0xdf, 0x1c, 0xe7, 0x1c, 0xdf, 0x1c, 0xdf, 0x1c, 0xdf, 0xfc, 0xde, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xfc, 0xe6, 0xfc, 0xe6, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xbb, 0xde, 0x1c, 0xdf, 0x1c, 0xdf, 0x1c, 0xdf, 0x1c, 0xdf, 0x1c, 0xdf, 0x1c, 0xdf, 0x1c, 0xdf, 0x00, 0x00, 0x00, 0x00, 0x1c, 0xdf, 0x1c, 0xdf, 0x1c, 0xe7, 0x1c, 0xdf, 0x1c, 0xdf, 0xfc, 0xde, 0xfb, 0xde, 0x18, 0xc6, 0x00, 0x00, 0xfc, 0xe6, 0xfb, 0xde, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xfc, 0xde, 0x1c, 0xdf, 0x1c, 0xdf, 0x1c, 0xdf, 0x1c, 0xdf, 0x1c, 0xdf, 0x1c, 0xdf, 0xfc, 0xde, 0x00, 0x00, 0xdb, 0xde, 0x1c, 0xdf, 0x1c, 0xe7, 0xfc, 0xe6, 0xfc, 0xd6, 0xfb, 0xde, 0x1c, 0xdf, 0x1c, 0xe7, 0x1c, 0xe7, 0x1c, 0xdf, 0x1c, 0xdf, 0xdb, 0xde, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1c, 0xdf, 0x1c, 0xdf, 0x1c, 0xdf, 0x1c, 0xdf, 0x1c, 0xdf, 0x1c, 0xdf, 0x1c, 0xdf, 0xbb, 0xde, 0xfb, 0xde, 0x1c, 0xdf, 0xba, 0xd6, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xfc, 0xde, 0x1c, 0xdf, 0x1c, 0xdf, 0x1c, 0xe7, 0x1c, 0xe7, 0x1c, 0xdf, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1c, 0xdf, 0x1c, 0xdf, 0x1c, 0xdf, 0x1c, 0xdf, 0x1c, 0xdf, 0x1c, 0xdf, 0x1c, 0xdf, 0xfc, 0xde, 0x1c, 0xdf, 0x1c, 0xe7, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1c, 0xe7, 0x1c, 0xe7, 0x1c, 0xdf, 0x1c, 0xdf, 0x1c, 0xe7, 0x1c, 0xdf, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xdb, 0xe6, 0x1c, 0xe7, 0x1c, 0xdf, 0x1c, 0xdf, 0x1c, 0xe7, 0x1c, 0xdf, 0x1c, 0xdf, 0x1c, 0xdf, 0x1c, 0xdf, 0x1c, 0xdf, 0xfc, 0xde, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xdb, 0xde, 0x1c, 0xdf, 0x1c, 0xe7, 0x1c, 0xdf, 0x1c, 0xdf, 0x1c, 0xdf, 0xfb, 0xde, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xfc, 0xe6, 0x1c, 0xe7, 0x1c, 0xe7, 0x1c, 0xdf, 0xfc, 0xde, 0xfb, 0xde, 0xbc, 0xe6, 0x1c, 0xe7, 0x1c, 0xdf, 0x1c, 0xdf, 0x1c, 0xe7, 0x00, 0x00, 0xbb, 0xde, 0xbb, 0xde, 0xfb, 0xde, 0x1c, 0xdf, 0x1c, 0xdf, 0x1c, 0xdf, 0x1c, 0xdf, 0x1c, 0xdf, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1c, 0xdf, 0x1c, 0xe7, 0x1c, 0xdf, 0x7a, 0xd6, 0x00, 0x00, 0x00, 0x00, 0x1c, 0xdf, 0x1c, 0xe7, 0x1c, 0xdf, 0x1c, 0xdf, 0x1c, 0xdf, 0x1c, 0xdf, 0xfc, 0xde, 0x1c, 0xe7, 0x1c, 0xdf, 0x1c, 0xdf, 0x1c, 0xdf, 0x1c, 0xdf, 0x1c, 0xe7, 0xfc, 0xde, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xfc, 0xe6, 0x1c, 0xdf, 0x1c, 0xdf, 0xfc, 0xde, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1c, 0xdf, 0x3c, 0xe7, 0x1c, 0xe7, 0x1c, 0xe7, 0x1c, 0xe7, 0xfc, 0xde, 0xdb, 0xde, 0x3c, 0xe7, 0xff, 0xff, 0x9a, 0xd6, 0x1b, 0xdf, 0x1c, 0xe7, 0x1c, 0xe7, 0xfc, 0xde, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1c, 0xe7, 0x1c, 0xe7, 0x1c, 0xdf, 0xfc, 0xde, 0x00, 0x00, 0x00, 0x00, 0x9a, 0xd6, 0x1c, 0xe7, 0x1c, 0xe7, 0x1c, 0xe7, 0xfb, 0xde, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xfc, 0xde, 0x5d, 0xd7, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1c, 0xdf, 0x1c, 0xe7, 0x1c, 0xdf, 0xbb, 0xde, 0x00, 0x00, 0xfb, 0xde, 0xfc, 0xde, 0x1c, 0xdf, 0xfb, 0xde, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x9a, 0xd6, 0x1c, 0xdf, 0x1c, 0xe7, 0x1c, 0xdf, 0x1c, 0xdf, 0xfc, 0xde, 0x1c, 0xe7, 0xfc, 0xde, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xfb, 0xde, 0x1c, 0xdf, 0x1c, 0xdf, 0xfc, 0xde, 0xfc, 0xde, 0xfc, 0xe6, 0x75, 0xad, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1c, 0xdf, 0x1c, 0xdf, 0x1c, 0xe7, 0xdb, 0xde, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x75, 0xad, 0x1c, 0xdf, 0x1c, 0xdf, 0x1c, 0xdf, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xfb, 0xde, 0x1c, 0xe7, 0x1c, 0xe7, 0x1c, 0xdf, 0x00, 0x00, 0x00, 0x00, 0x75, 0xad, 0xfc, 0xde, 0x1c, 0xe7, 0x7a, 0xd6, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xfb, 0xde, 0xfc, 0xde, 0x1c, 0xdf, 0xfb, 0xde, 0xfb, 0xde, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xfc, 0xde, 0x1c, 0xe7, 0x1c, 0xe7, 0x1b, 0xdf, 0x00, 0x00, 0x00, 0x00, 0x18, 0xc6, 0xfc, 0xde, 0x1c, 0xe7, 0x7a, 0xd6, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xfb, 0xde, 0x1c, 0xdf, 0x1c, 0xdf, 0x1c, 0xdf, 0x1c, 0xe7, 0x1c, 0xe7, 0x1c, 0xdf, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1c, 0xe7, 0x1c, 0xe7, 0x1c, 0xe7, 0x1c, 0xc7, 0x00, 0x00, 0x00, 0x00, 0x18, 0xc6, 0xfc, 0xde, 0x1c, 0xe7, 0x3c, 0xe7, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xfc, 0xde, 0x1c, 0xdf, 0x1c, 0xdf, 0x7a, 0xd6, 0xfb, 0xde, 0x1c, 0xe7, 0x1c, 0xe7, 0xfb, 0xde, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0xdb, 0xde, 0x1c, 0xe7, 0x1c, 0xe7, 0x1c, 0xe7, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x18, 0xc6, 0xfc, 0xde, 0x1c, 0xe7, 0x3c, 0xe7, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1c, 0xd7, 0x1c, 0xdf, 0x1c, 0xdf, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xfb, 0xde, 0xfc, 0xde, 0x7f, 0xad, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0xfc, 0xde, 0x1c, 0xe7, 0x1c, 0xe7, 0xfb, 0xde, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x18, 0xc6, 0x1c, 0xdf, 0x1c, 0xe7, 0x3c, 0xe7, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xfb, 0xde, 0x1c, 0xdf, 0x1c, 0xdf, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0x1c, 0xe7, 0x1c, 0xe7, 0x1c, 0xdf, 0xba, 0xd6, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x18, 0xc6, 0x1c, 0xdf, 0x1c, 0xe7, 0x3c, 0xe7, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xfb, 0xde, 0x1c, 0xdf, 0x1c, 0xdf, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0xfc, 0xe6, 0x1c, 0xdf, 0xfc, 0xde, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x18, 0xc6, 0x1c, 0xdf, 0x1c, 0xe7, 0xbb, 0xde, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xbc, 0xde, 0x1c, 0xdf, 0x1c, 0xdf, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xfc, 0xde, 0xfc, 0xde, 0xfb, 0xde, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x18, 0xc6, 0xfc, 0xde, 0x1c, 0xdf, 0xdb, 0xde, 0x1c, 0xe7, 0x3d, 0xdf, 0x1c, 0xe7, 0x00, 0x00, 0x1c, 0xdf, 0x1c, 0xe7, 0x1c, 0xdf, 0x00, 0x00, 0xdb, 0xde, 0x1c, 0xe7, 0x1c, 0xe7, 0x18, 0xc6, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x18, 0xc6, 0x1c, 0xdf, 0x1c, 0xe7, 0x1c, 0xe7, 0x1c, 0xe7, 0x1c, 0xdf, 0x1c, 0xe7, 0xdb, 0xde, 0xfb, 0xde, 0x1c, 0xdf, 0x1c, 0xe7, 0x1c, 0xe7, 0x1c, 0xdf, 0x1c, 0xe7, 0x1c, 0xe7, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x18, 0xc6, 0xfc, 0xde, 0xfc, 0xe6, 0xfc, 0xe6, 0x1c, 0xdf, 0x1c, 0xdf, 0x1c, 0xe7, 0xfb, 0xde, 0x00, 0x00, 0xdb, 0xde, 0x1c, 0xdf, 0x1c, 0xe7, 0x1c, 0xdf, 0x1c, 0xdf, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 

  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x06, 0x24, 0x3a, 0x41, 0x38, 0x12, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x36, 0x90, 0xd5, 0xf7, 0xff, 0xf2, 0xb2, 0xc7, 0x9c, 0x21, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x3f, 0xb1, 0xf0, 0xff, 0xff, 0xff, 0xff, 0xa2, 0x00, 0x14, 0x76, 0xd5, 0x3a, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x31, 0x29, 0x00, 0x00, 0x00, 0x17, 0x9d, 0xc6, 0x47, 0xb8, 0xff, 0xff, 0xff, 0xff, 0x58, 0x00, 0x00, 0x00, 0xbe, 0x47, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x3d, 0xff, 0xec, 0x17, 0x15, 0x72, 0xeb, 0x8b, 0x00, 0x0f, 0xed, 0xff, 0xff, 0xff, 0xee, 0x1a, 0x00, 0x00, 0x1e, 0xcb, 0x0c, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x94, 0xff, 0xff, 0xca, 0xeb, 0xff, 0xec, 0x0d, 0x00, 0x56, 0xff, 0xff, 0xff, 0xff, 0xaf, 0x00, 0x00, 0x00, 0x5e, 0xa1, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x06, 0xd6, 0xff, 0xff, 0xff, 0xff, 0xff, 0xaa, 0x00, 0x00, 0xa8, 0xff, 0xfe, 0xea, 0xe1, 0x9a, 0x29, 0x04, 0x00, 0xa7, 0x59, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x33, 0xfe, 0xff, 0xff, 0xff, 0xff, 0xff, 0x5e, 0x00, 0x1f, 0xb7, 0x7b, 0x33, 0x0f, 0x31, 0xfd, 0xff, 0xcd, 0x6b, 0xca, 0x19, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x7b, 0xff, 0xff, 0xff, 0xff, 0xff, 0xef, 0x2a, 0x77, 0xd0, 0x17, 0x00, 0x00, 0x00, 0x6f, 0xff, 0xff, 0xff, 0xff, 0xb3, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xc7, 0xff, 0xff, 0xff, 0xff, 0xff, 0xec, 0xd5, 0xff, 0xb2, 0x00, 0x00, 0x00, 0x00, 0xbc, 0xff, 0xff, 0xff, 0xff, 0x67, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x26, 0xf7, 0xff, 0xff, 0xff, 0xed, 0x8c, 0xd6, 0xff, 0xff, 0x65, 0x00, 0x00, 0x00, 0x1a, 0xf1, 0xff, 0xff, 0xff, 0xf7, 0x26, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x66, 0xff, 0xff, 0x97, 0x57, 0x15, 0x18, 0xf1, 0xff, 0xf2, 0x17, 0x00, 0x06, 0x12, 0x72, 0xff, 0xff, 0xff, 0xff, 0xc6, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xb0, 0xff, 0xe0, 0x05, 0x00, 0x00, 0x64, 0xff, 0xff, 0xcd, 0x55, 0xa0, 0xc3, 0xc2, 0xc9, 0xe0, 0xfc, 0xff, 0xff, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x16, 0xeb, 0xff, 0x9f, 0x00, 0x00, 0x00, 0xb0, 0xff, 0xff, 0xe8, 0xa3, 0x58, 0x20, 0x09, 0x03, 0x0b, 0x2f, 0x94, 0xfb, 0x3b, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x54, 0xff, 0xff, 0x50, 0x00, 0x00, 0x10, 0xf4, 0xf8, 0x86, 0x1c, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x33, 0x0a, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x9e, 0xff, 0xe9, 0x0c, 0x00, 0x37, 0xb9, 0xb9, 0x29, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0x0b, 0xdf, 0xff, 0xe4, 0x82, 0xb9, 0xc0, 0x5a, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0x40, 0xff, 0xff, 0xad, 0x6f, 0x3c, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0x8a, 0xff, 0xf7, 0x20, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x03, 0xd1, 0xff, 0xc8, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x30, 0xfc, 0xff, 0x7f, 0x00, 0x00, 0x03, 0x71, 0x75, 0x05, 0x00, 0x00, 0x00, 0x00, 0x00, 0x22, 0x90, 0xc0, 0xac, 0x4e, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x77, 0xff, 0xff, 0x36, 0x00, 0x00, 0x08, 0xe8, 0xec, 0x0a, 0x00, 0x00, 0x00, 0x00, 0x29, 0xe9, 0xff, 0xeb, 0xff, 0xff, 0x65, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0xbf, 0xff, 0xd9, 0x08, 0x00, 0x00, 0x08, 0xdf, 0xe2, 0x09, 0x00, 0x00, 0x00, 0x00, 0xaf, 0xff, 0x66, 0x05, 0x46, 0xff, 0xdb, 0x07, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x20, 0xf3, 0xff, 0x92, 0x00, 0x00, 0x00, 0x08, 0xdf, 0xe2, 0x09, 0x00, 0x00, 0x00, 0x17, 0xf5, 0xce, 0x00, 0x00, 0x00, 0x4b, 0x35, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x61, 0xff, 0xff, 0x4c, 0x00, 0x00, 0x00, 0x08, 0xdf, 0xe2, 0x09, 0x00, 0x00, 0x00, 0x32, 0xff, 0xa1, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x75, 0xff, 0xe7, 0x11, 0x00, 0x00, 0x00, 0x08, 0xdf, 0xe2, 0x09, 0x00, 0x00, 0x00, 0x38, 0xff, 0xa0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x0f, 0x6e, 0x3c, 0x00, 0x00, 0x00, 0x00, 0x08, 0xdf, 0xe2, 0x06, 0x00, 0x00, 0x00, 0x1e, 0xfa, 0xca, 0x00, 0x00, 0x00, 0x88, 0x75, 0x07, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x08, 0xdf, 0xe5, 0x1b, 0x11, 0x13, 0x10, 0x00, 0xbc, 0xff, 0x5c, 0x00, 0x4a, 0xff, 0xe2, 0x08, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x08, 0xe3, 0xff, 0xf3, 0xf2, 0xf5, 0xde, 0x0d, 0x32, 0xf4, 0xff, 0xe0, 0xff, 0xff, 0x57, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x04, 0x90, 0xaf, 0xaf, 0xaf, 0xb1, 0xa1, 0x0e, 0x00, 0x2f, 0xa4, 0xcd, 0xba, 0x4e, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
};

const lv_image_dsc_t launch = {
  .header.cf = LV_COLOR_FORMAT_RGB565A8,
  .header.magic = LV_IMAGE_HEADER_MAGIC,
  .header.w = 32,
  .header.h = 32,
  .data_size = 1024 * 3,
  .data = launch_map,
};