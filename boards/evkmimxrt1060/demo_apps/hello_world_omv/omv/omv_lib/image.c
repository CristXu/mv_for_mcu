#include "image.h"
#include "stdlib.h"
#include "array.h"
#include "sensor.h"
#warning "#include ff_wrapper.h to enable file operations here"
#include "xalloc.h"
#include "fb_alloc.h"
#include "framebuffer.h"
#undef M_PI
#define M_PI    3.141592654f
#ifndef M_PI_2
#define M_PI_2    1.5707963267f
#endif

#define exit while
image_module image;
image_type img;

// image module
#warning "#define FR_OK here, but include ff.h in imlib.h is the key"
#define FR_OK 0
cascade_t HaarCascade(char* path, HaarCascade_opt opt_args){
	cascade_t cascade;
	int res = imlib_load_cascade(&cascade, path);
	if (res != FR_OK){
		printf("Load cascade failed!\r\n");
		exit(-1);
	}
	if (opt_args.stages > 0 && opt_args.stages < cascade.n_stages){
		cascade.n_stages = opt_args.stages;
	}
	return cascade;
}

// image type
void draw_rectangle(rectangle_t* obj, draw_rectangle_opt opt_args){
	int arg_rx = obj->x;
	int arg_ry = obj->y;
	int arg_rw = obj->w;
	int arg_rh = obj->h;
	imlib_draw_rectangle(img.img, arg_rx, arg_ry, arg_rw, arg_rh, opt_args.arg_c, opt_args.arg_thickness, opt_args.arg_fill);
}

array_t * find_features(cascade_t* cascade, find_features_opt opt_args){
	bool need_free = false;
	if(!img.img) {
		printf("please init the member img in struct image_type first\r\n");
		exit(-1);
	}
	if(!cascade) {
		printf("Pass a valid cascade pointer\r\n");
		exit(-1);
	}
	cascade->threshold = opt_args.threshold;
	cascade->scale_factor = opt_args.scale_factor;
	if(!opt_args.roi){
		opt_args.roi = (rectangle_t*)malloc(sizeof(rectangle_t));
		opt_args.roi->x = opt_args.roi->y = 0;
		opt_args.roi->w = img.img->w;
		opt_args.roi->h = img.img->h;
		need_free = true;
	}
	else{
		if(opt_args.roi->w < cascade->window.w && opt_args.roi->h < cascade->window.h){
			printf("Region of interest is smaller than detector window!\r\n");
			exit(-1);
		}
	}
	fb_alloc_mark();
	array_t *objects_array = imlib_detect_objects(img.img, cascade, opt_args.roi);
	fb_alloc_free_till_mark();
	if(need_free){
		free(opt_args.roi);
		opt_args.roi = NULL;
	}
	return objects_array;
}
image_module image = {
	.HaarCascade = HaarCascade,
};

image_type img =  {
	.img = NULL,
	.find_features = find_features,
	.draw_rectangle = draw_rectangle,
};



