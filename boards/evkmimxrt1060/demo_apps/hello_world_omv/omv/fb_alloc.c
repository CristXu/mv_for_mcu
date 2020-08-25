/*
 * This file is part of the OpenMV project.
 * Copyright (c) 2013-2016 Kwabena W. Agyeman <kwagyeman@openmv.io>
 * This work is licensed under the MIT license, see the file LICENSE for details.
 *
 * Interface for using extra frame buffer RAM as a stack.
 *
 */
#include "imlib.h"
#include "stdlib.h"
#include "string.h"
#include "fb_alloc.h"
#include "framebuffer.h"
#include "omv_boardconfig.h"

#if defined(__CC_ARM) || defined (__clang__)
    extern char Image$$OMV_FB_END$$Base;
    #define _fballoc Image$$OMV_FB_END$$Base
    #if defined(OMV_FB_OVERLAY_MEMORY)
        #define FB_OVERLAY_MEMORY_FLAG 0x1
        extern char Image$$OMV_FB_ALLOC_BIG$$Base;        
        extern char Image$$OMV_FB_ALLOC_BIG_END$$Base;
        #define _fballoc_overlay Image$$OMV_FB_ALLOC_BIG_END$$Base
        static char *pointer_overlay = &_fballoc_overlay;
        uint32_t OMV_FB_OVERLAY_MEMORY_ORIGIN = (uint32_t)&Image$$OMV_FB_ALLOC_BIG$$Base;
    #endif
#else
    extern char _fballoc;
    #if defined(OMV_FB_OVERLAY_MEMORY)
        #define FB_OVERLAY_MEMORY_FLAG 0x1
        extern char _fballoc_overlay;
        static char *pointer_overlay = &_fballoc_overlay;
    #endif
#endif

char *pointer = &_fballoc;
static int marks = 0;
#if defined(__clang__) || defined(__GNUC__)
#define __weak __attribute__((weak))
#define NORETURN __attribute__((__noreturn__))
#endif
__weak NORETURN void fb_alloc_fail()
{
    while(1);
}

void fb_alloc_init0()
{
    pointer = &_fballoc;
    marks = 0;
    #if defined(OMV_FB_OVERLAY_MEMORY)
    pointer_overlay = &_fballoc_overlay;
    #endif
}

uint32_t fb_avail()
{
    uint32_t temp = pointer - ((char *) MAIN_FB_PIXELS()) - sizeof(uint32_t);

    return (temp < sizeof(uint32_t)) ? 0 : temp;
}

void fb_alloc_mark()
{
    char *new_pointer = pointer - sizeof(uint32_t);

    // Check if allocation overwrites the framebuffer pixels
    if (new_pointer < (char *) MAIN_FB_PIXELS()) {
        printf(
            "Out of fast Frame Buffer Stack Memory!"
            " Please reduce the resolution of the image you are running this algorithm on to bypass this issue!");
		fb_alloc_fail();
    }

    // fb_alloc does not allow regions which are a size of 0 to be alloced,
    // meaning that the value below is always 8 or more but never 4. So,
    // we will use a size value of 4 as a marker in the alloc stack.
    *((uint32_t *) new_pointer) = sizeof(uint32_t); // Save size.
    pointer = new_pointer;
    marks += 1;
    #if defined(FB_ALLOC_STATS)
    alloc_bytes = 0;
    alloc_bytes_peak = 0;
    #endif
}

void fb_alloc_free_till_mark()
{
    if (!marks) return;
    while (pointer < &_fballoc) {
        uint32_t size = *((uint32_t *) pointer);
        #if defined(OMV_FB_OVERLAY_MEMORY)
        if (size & FB_OVERLAY_MEMORY_FLAG) { // Check for fast flag.
            size &= ~FB_OVERLAY_MEMORY_FLAG; // Remove it.
            pointer_overlay += size - sizeof(uint32_t);
        }
        #endif
        pointer += size; // Get size and pop.
        if (size == sizeof(uint32_t)) break; // Break on first marker.
    }
    marks -= 1;
    #if defined(FB_ALLOC_STATS)
    printf("fb_alloc peak memory: %lu\n", alloc_bytes_peak);
    #endif
}

// returns null pointer without error if size==0
void *fb_alloc(uint32_t size, int hints)
{
    if (!size) {
        return NULL;
    }

    size = ((size+sizeof(uint32_t)-1)/sizeof(uint32_t))*sizeof(uint32_t);// Round Up
    char *result = pointer - size;
    char *new_pointer = result - sizeof(uint32_t);

    // Check if allocation overwrites the framebuffer pixels
    // rocky: this means extra buffer must be immediately after pixels.
    if (new_pointer < (char *) MAIN_FB_PIXELS()) {
        fb_alloc_fail();
    }

    // size is always 4/8/12/etc. so the value below must be 8 or more.
    *((uint32_t *) new_pointer) = size + sizeof(uint32_t); // Save size.
    pointer = new_pointer;
    #if defined(FB_ALLOC_STATS)
    alloc_bytes += size;
    if (alloc_bytes > alloc_bytes_peak) {
        alloc_bytes_peak = alloc_bytes;
    }
    printf("fb_alloc %lu bytes\n", size);
    #endif
    #if defined(OMV_FB_OVERLAY_MEMORY)
    if ((!(hints & FB_ALLOC_PREFER_SIZE))
    && (((uint32_t) (pointer_overlay - OMV_FB_OVERLAY_MEMORY_ORIGIN)) >= size)) {
        // Return overlay memory instead.
        pointer_overlay -= size;
        result = pointer_overlay;
        *new_pointer |= FB_OVERLAY_MEMORY_FLAG; // Add flag.
    }
    #endif
    return result;
}

// returns null pointer without error if passed size==0
void *fb_alloc0(uint32_t size, int hints)
{
    void *mem = fb_alloc(size, hints);
    memset(mem, 0, size); // does nothing if size is zero.
    return mem;
}

void *fb_alloc_all(uint32_t *size, int hints)
{
    uint32_t temp = pointer - ((char *) MAIN_FB_PIXELS()) - sizeof(uint32_t);

    if (temp < sizeof(uint32_t)) {
        *size = 0;
        return NULL;
    }

    #if defined(OMV_FB_OVERLAY_MEMORY)
    if (!(hints & FB_ALLOC_PREFER_SIZE)) {
        *size = (uint32_t) (pointer_overlay - OMV_FB_OVERLAY_MEMORY_ORIGIN);
        temp = IM_MIN(temp, *size);
    }
    #endif
    *size = (temp / sizeof(uint32_t)) * sizeof(uint32_t); // Round Down
    char *result = pointer - *size;
    char *new_pointer = result - sizeof(uint32_t);

    // size is always 4/8/12/etc. so the value below must be 8 or more.
    *((uint32_t *) new_pointer) = *size + sizeof(uint32_t); // Save size.
    pointer = new_pointer;
    #if defined(FB_ALLOC_STATS)
    alloc_bytes += *size;
    if (alloc_bytes > alloc_bytes_peak) {
        alloc_bytes_peak = alloc_bytes;
    }
    printf("fb_alloc_all %lu bytes\n", *size);
    #endif
    #if defined(OMV_FB_OVERLAY_MEMORY)
    if (!(hints & FB_ALLOC_PREFER_SIZE)) {
        // Return overlay memory instead.
        pointer_overlay -= *size;
        result = pointer_overlay;
        *new_pointer |= FB_OVERLAY_MEMORY_FLAG; // Add flag.
    }
    #endif
    return result;
}

// returns null pointer without error if returned size==0
void *fb_alloc0_all(uint32_t *size, int hints)
{
    void *mem = fb_alloc_all(size, hints);
    memset(mem, 0, *size); // does nothing if size is zero.
    return mem;
}

void fb_free()
{
    if (pointer < &_fballoc) {
        uint32_t size = *((uint32_t *) pointer);
        #if defined(OMV_FB_OVERLAY_MEMORY)
        if (size & FB_OVERLAY_MEMORY_FLAG) { // Check for fast flag.
            size &= ~FB_OVERLAY_MEMORY_FLAG; // Remove it.
            pointer_overlay += size - sizeof(uint32_t);
        }
        #endif
        #if defined(FB_ALLOC_STATS)
        alloc_bytes -= size;
        #endif
        pointer += size; // Get size and pop.
    }
}

void fb_free_all()
{
    while (pointer < &_fballoc) {
        uint32_t size = *((uint32_t *) pointer);
        #if defined(OMV_FB_OVERLAY_MEMORY)
        if (size & FB_OVERLAY_MEMORY_FLAG) { // Check for fast flag.
            size &= ~FB_OVERLAY_MEMORY_FLAG; // Remove it.
            pointer_overlay += size - sizeof(uint32_t);
        }
        #endif
        #if defined(FB_ALLOC_STATS)
        alloc_bytes -= size;
        #endif
        pointer += size; // Get size and pop.
    }
    marks = 0;
}
