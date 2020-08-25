#ifndef __FRAMEBUFFER_H__
#define __FRAMEBUFFER_H__
#include "stdint.h"
#include "omv_boardconfig.h"
typedef struct framebuffer {
	union {
		struct {
			int x, y, w, h, u, v, bpp, res;
		};
		uint64_t u64AlignForIMxRT;	// in i.MX RT, this address must be aligned to 8 bytes!
	};
    uint8_t pixels[];	
} framebuffer_t;

extern framebuffer_t *fb_framebuffer; 

// in order to keep the MAIN_FB_PIXELS macro, set this to the head of the fballoc, avoid overlap
#define MAIN_FB()           (fb_framebuffer)
#define MAIN_FB_PIXELS()    (MAIN_FB()->pixels + fb_buffer_size())

uint32_t fb_buffer_size();
#endif
