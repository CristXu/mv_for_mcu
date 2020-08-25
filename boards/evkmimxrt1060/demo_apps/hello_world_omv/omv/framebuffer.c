/*
 * This file is part of the OpenMV project.
 * Copyright (c) 2013/2014 Ibrahim Abdelkader <i.abdalkader@gmail.com>
 * This work is licensed under the MIT license, see the file LICENSE for details.
 *
 * Framebuffer stuff.
 *
 */
#include "imlib.h"
#include "framebuffer.h"
#if defined(__clang__)  || defined(__CC_ARM)
extern unsigned int Image$$OMV_MAIN_FB$$Base[];
framebuffer_t *fb_framebuffer = (framebuffer_t*) Image$$OMV_MAIN_FB$$Base;
#else
extern char _fb_base;
framebuffer_t *fb_framebuffer = (framebuffer_t *) &_fb_base;
#endif

uint32_t fb_buffer_size()
{
    switch (MAIN_FB()->bpp) {
        case IMAGE_BPP_BINARY: {
            return ((MAIN_FB()->w + UINT32_T_MASK) >> UINT32_T_SHIFT) * MAIN_FB()->h;
        }
        case IMAGE_BPP_GRAYSCALE: {
            return (MAIN_FB()->w * MAIN_FB()->h) * sizeof(uint8_t);
        }
        case IMAGE_BPP_RGB565: {
            return (MAIN_FB()->w * MAIN_FB()->h) * sizeof(uint16_t);
        }
        case IMAGE_BPP_BAYER: {
            return MAIN_FB()->w * MAIN_FB()->h;
        }
        default: { // JPEG
            return MAIN_FB()->bpp;
        }
    }
}


