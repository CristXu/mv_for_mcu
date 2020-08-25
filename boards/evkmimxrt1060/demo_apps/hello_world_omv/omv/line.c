#include "imlib.h"
#ifdef WIN32
#define fast_roundf(x) (int)roundf(x)
#define fast_sqrtf(x) sqrtf(x)
#define fast_atan2f(x, y) atan2f(x, y)
#endif
/* This file is part of the OpenMV project.
 * Copyright (c) 2013-2017 Ibrahim Abdelkader <iabdalkader@openmv.io> & Kwabena W. Agyeman <kwagyeman@openmv.io>
 * This work is licensed under the MIT license, see the file LICENSE for details.
 *
 * Line functions.
 */
static void pixel_magnitude(image_t* ptr, int x, int y, int* theta, uint32_t* mag)
{
	switch (ptr->bpp) {
	case IMAGE_BPP_BINARY: {
		uint32_t* row_ptr = IMAGE_COMPUTE_BINARY_PIXEL_ROW_PTR(ptr, y);
		int pixel; // Sobel Algorithm Below
		int x_acc = 0;
		int y_acc = 0;

		if (y != 0) row_ptr -= ((ptr->w + UINT32_T_MASK) >> UINT32_T_SHIFT);

		pixel = COLOR_BINARY_TO_GRAYSCALE(IMAGE_GET_BINARY_PIXEL_FAST(row_ptr, IM_MAX(x - 1, 0)));
		x_acc += pixel * +1; // x[0,0] -> pixel * +1
		y_acc += pixel * +1; // y[0,0] -> pixel * +1

		pixel = COLOR_BINARY_TO_GRAYSCALE(IMAGE_GET_BINARY_PIXEL_FAST(row_ptr, x));
		// x[0,1] -> pixel * 0
		y_acc += pixel * +2; // y[0,1] -> pixel * +2

		pixel = COLOR_BINARY_TO_GRAYSCALE(IMAGE_GET_BINARY_PIXEL_FAST(row_ptr, IM_MIN(x + 1, ptr->w - 1)));
		x_acc += pixel * -1; // x[0,2] -> pixel * -1
		y_acc += pixel * +1; // y[0,2] -> pixel * +1

		if (y != 0) row_ptr += ((ptr->w + UINT32_T_MASK) >> UINT32_T_SHIFT);

		pixel = COLOR_BINARY_TO_GRAYSCALE(IMAGE_GET_BINARY_PIXEL_FAST(row_ptr, IM_MAX(x - 1, 0)));
		x_acc += pixel * +2; // x[1,0] -> pixel * +2
							 // y[1,0] -> pixel * 0

		// pixel = COLOR_BINARY_TO_GRAYSCALE(IMAGE_GET_BINARY_PIXEL_FAST(row_ptr, x));
		// x[1,1] -> pixel * 0
		// y[1,1] -> pixel * 0

		pixel = COLOR_BINARY_TO_GRAYSCALE(IMAGE_GET_BINARY_PIXEL_FAST(row_ptr, IM_MIN(x + 1, ptr->w - 1)));
		x_acc += pixel * -2; // x[1,2] -> pixel * -2
							 // y[1,2] -> pixel * 0

		if (y != (ptr->h - 1)) row_ptr += ((ptr->w + UINT32_T_MASK) >> UINT32_T_SHIFT);

		pixel = COLOR_BINARY_TO_GRAYSCALE(IMAGE_GET_BINARY_PIXEL_FAST(row_ptr, IM_MAX(x - 1, 0)));
		x_acc += pixel * +1; // x[2,0] -> pixel * +1
		y_acc += pixel * -1; // y[2,0] -> pixel * -1

		pixel = COLOR_BINARY_TO_GRAYSCALE(IMAGE_GET_BINARY_PIXEL_FAST(row_ptr, x));
		// x[2,1] -> pixel * 0
		y_acc += pixel * -2; // y[2,1] -> pixel * -2

		pixel = COLOR_BINARY_TO_GRAYSCALE(IMAGE_GET_BINARY_PIXEL_FAST(row_ptr, IM_MIN(x + 1, ptr->w - 1)));
		x_acc += pixel * -1; // x[2,2] -> pixel * -1
		y_acc += pixel * -1; // y[2,2] -> pixel * -1

		if (y != (ptr->h - 1)) row_ptr -= ((ptr->w + UINT32_T_MASK) >> UINT32_T_SHIFT);

		*theta = fast_roundf((x_acc ? fast_atan2f(y_acc, x_acc) : 1.570796f) * 57.295780) % 180; // * (180 / PI)
		if (*theta < 0)* theta += 180;
		*mag = fast_roundf(fast_sqrtf((x_acc * x_acc) + (y_acc * y_acc)));
		break;
	}
	case IMAGE_BPP_GRAYSCALE: {
		uint8_t* row_ptr = IMAGE_COMPUTE_GRAYSCALE_PIXEL_ROW_PTR(ptr, y);
		int pixel; // Sobel Algorithm Below... w/ Scharr...
		int x_acc = 0;
		int y_acc = 0;

		if (y != 0) row_ptr -= ptr->w;

		pixel = IMAGE_GET_GRAYSCALE_PIXEL_FAST(row_ptr, IM_MAX(x - 1, 0));
		x_acc += pixel * +1; // x[0,0] -> pixel * +1
		y_acc += pixel * +1; // y[0,0] -> pixel * +1

		pixel = IMAGE_GET_GRAYSCALE_PIXEL_FAST(row_ptr, x);
		// x[0,1] -> pixel * 0
		y_acc += pixel * +2; // y[0,1] -> pixel * +2

		pixel = IMAGE_GET_GRAYSCALE_PIXEL_FAST(row_ptr, IM_MIN(x + 1, ptr->w - 1));
		x_acc += pixel * -1; // x[0,2] -> pixel * -1
		y_acc += pixel * +1; // y[0,2] -> pixel * +1

		if (y != 0) row_ptr += ptr->w;

		pixel = IMAGE_GET_GRAYSCALE_PIXEL_FAST(row_ptr, IM_MAX(x - 1, 0));
		x_acc += pixel * +2; // x[1,0] -> pixel * +2
							 // y[1,0] -> pixel * 0

		// pixel = IMAGE_GET_GRAYSCALE_PIXEL_FAST(row_ptr, x));
		// x[1,1] -> pixel * 0
		// y[1,1] -> pixel * 0

		pixel = IMAGE_GET_GRAYSCALE_PIXEL_FAST(row_ptr, IM_MIN(x + 1, ptr->w - 1));
		x_acc += pixel * -2; // x[1,2] -> pixel * -2
							 // y[1,2] -> pixel * 0

		if (y != (ptr->h - 1)) row_ptr += ptr->w;

		pixel = IMAGE_GET_GRAYSCALE_PIXEL_FAST(row_ptr, IM_MAX(x - 1, 0));
		x_acc += pixel * +1; // x[2,0] -> pixel * +1
		y_acc += pixel * -1; // y[2,0] -> pixel * -1

		pixel = IMAGE_GET_GRAYSCALE_PIXEL_FAST(row_ptr, x);
		// x[2,1] -> pixel * 0
		y_acc += pixel * -2; // y[2,1] -> pixel * -2

		pixel = IMAGE_GET_GRAYSCALE_PIXEL_FAST(row_ptr, IM_MIN(x + 1, ptr->w - 1));
		x_acc += pixel * -1; // x[2,2] -> pixel * -1
		y_acc += pixel * -1; // y[2,2] -> pixel * -1

		if (y != (ptr->h - 1)) row_ptr -= ptr->w;

		*theta = fast_roundf((x_acc ? fast_atan2f(y_acc, x_acc) : 1.570796f) * 57.295780) % 180; // * (180 / PI)
		if (*theta < 0)* theta += 180;
		*mag = fast_roundf(fast_sqrtf((x_acc * x_acc) + (y_acc * y_acc)));
		break;
	}
	case IMAGE_BPP_RGB565: {
		uint16_t* row_ptr = IMAGE_COMPUTE_RGB565_PIXEL_ROW_PTR(ptr, y);
		int pixel; // Sobel Algorithm Below... w/ Scharr...
		int x_acc = 0;
		int y_acc = 0;

		if (y != 0) row_ptr -= ptr->w;

		pixel = COLOR_RGB565_TO_GRAYSCALE(IMAGE_GET_RGB565_PIXEL_FAST(row_ptr, IM_MAX(x - 1, 0)));
		x_acc += pixel * +1; // x[0,0] -> pixel * +1
		y_acc += pixel * +1; // y[0,0] -> pixel * +1

		pixel = COLOR_RGB565_TO_GRAYSCALE(IMAGE_GET_RGB565_PIXEL_FAST(row_ptr, x));
		// x[0,1] -> pixel * 0
		y_acc += pixel * +2; // y[0,1] -> pixel * +2

		pixel = COLOR_RGB565_TO_GRAYSCALE(IMAGE_GET_RGB565_PIXEL_FAST(row_ptr, IM_MIN(x + 1, ptr->w - 1)));
		x_acc += pixel * -1; // x[0,2] -> pixel * -1
		y_acc += pixel * +1; // y[0,2] -> pixel * +1

		if (y != 0) row_ptr += ptr->w;

		pixel = COLOR_RGB565_TO_GRAYSCALE(IMAGE_GET_RGB565_PIXEL_FAST(row_ptr, IM_MAX(x - 1, 0)));
		x_acc += pixel * +2; // x[1,0] -> pixel * +2
							 // y[1,0] -> pixel * 0

		// pixel = COLOR_RGB565_TO_GRAYSCALE(IMAGE_GET_RGB565_PIXEL_FAST(row_ptr, x));
		// x[1,1] -> pixel * 0
		// y[1,1] -> pixel * 0

		pixel = COLOR_RGB565_TO_GRAYSCALE(IMAGE_GET_RGB565_PIXEL_FAST(row_ptr, IM_MIN(x + 1, ptr->w - 1)));
		x_acc += pixel * -2; // x[1,2] -> pixel * -2
							 // y[1,2] -> pixel * 0

		if (y != (ptr->h - 1)) row_ptr += ptr->w;

		pixel = COLOR_RGB565_TO_GRAYSCALE(IMAGE_GET_RGB565_PIXEL_FAST(row_ptr, IM_MAX(x - 1, 0)));
		x_acc += pixel * +1; // x[2,0] -> pixel * +1
		y_acc += pixel * -1; // y[2,0] -> pixel * -1

		pixel = COLOR_RGB565_TO_GRAYSCALE(IMAGE_GET_RGB565_PIXEL_FAST(row_ptr, x));
		// x[2,1] -> pixel * 0
		y_acc += pixel * -2; // y[2,1] -> pixel * -2

		pixel = COLOR_RGB565_TO_GRAYSCALE(IMAGE_GET_RGB565_PIXEL_FAST(row_ptr, IM_MIN(x + 1, ptr->w - 1)));
		x_acc += pixel * -1; // x[2,2] -> pixel * -1
		y_acc += pixel * -1; // y[2,2] -> pixel * -1

		if (y != (ptr->h - 1)) row_ptr -= ptr->w;

		*theta = fast_roundf((x_acc ? fast_atan2f(y_acc, x_acc) : 1.570796f) * 57.295780) % 180; // * (180 / PI)
		if (*theta < 0)* theta += 180;
		*mag = fast_roundf(fast_sqrtf((x_acc * x_acc) + (y_acc * y_acc)));
		break;
	}
	default: {
		break;
	}
	}
}
#ifdef WIN32
#undef fast_roundf(x)
#undef fast_sqrtf(x)
#undef fast_atan2f(x, y)
#endif
// http://www.brackeen.com/vga/source/djgpp20/lines.c.html
// http://www.brackeen.com/vga/source/bc31/lines.c.html
size_t trace_line(image_t* ptr, line_t* l, int* theta_buffer, uint32_t* mag_buffer, point_t* point_buffer)
{
	int dx = l->x2 - l->x1; // the horizontal distance of the line
	int dy = l->y2 - l->y1; // the vertical distance of the line
	int dxabs = abs(dx);
	int dyabs = abs(dy);
	int sdx = (dx < 0) ? -1 : ((dx > 0) ? 1 : 0);
	int sdy = (dy < 0) ? -1 : ((dy > 0) ? 1 : 0);
	int x = dyabs >> 1; // correct
	int y = dxabs >> 1; // correct
	int px = l->x1;
	int py = l->y1;

	size_t index = 0;

	pixel_magnitude(ptr, px, py, theta_buffer + index, mag_buffer + index);
	point_buffer[index++] = (point_t){ .x = px,.y = py };

	if (dxabs >= dyabs) { // the line is more horizontal than vertical
		for (int i = 0; i < dxabs; i++) {
			y += dyabs;

			if (y >= dxabs) {
				y -= dxabs;
				py += sdy;
			}

			px += sdx;

			pixel_magnitude(ptr, px, py, theta_buffer + index, mag_buffer + index);
			point_buffer[index++] = (point_t){ .x = px,.y = py };
		}
	}
	else { // the line is more vertical than horizontal
		for (int i = 0; i < dyabs; i++) {
			x += dxabs;

			if (x >= dyabs) {
				x -= dyabs;
				px += sdx;
			}

			py += sdy;

			pixel_magnitude(ptr, px, py, theta_buffer + index, mag_buffer + index);
			point_buffer[index++] = (point_t){ .x = px,.y = py };

		}
	}

	return index;
}
