#ifndef IMAGE_H 
#define IMAGE_H
#include <arm_math.h>
#include "imlib.h"
// init helper 
#define HaarCascade_opt_default  { \
	.stages = 25, \
};

#define draw_rectangle_opt_default  { \
	.arg_c = 255, \
	.arg_thickness = 1, \
	.arg_fill = false, \
};

#define find_features_opt_default { \
	.threshold = 0.5, \
	.scale_factor = 1.5, \
	.roi = NULL, \
};

#define DECLARE_AND_INIT_FUNCTION_PARAMS(func, parms_name) \
	func##_opt parms_name = func##_opt_default; 
	
typedef struct {
	int stages;
}HaarCascade_opt;
typedef struct image_module{
	cascade_t  (*HaarCascade)(char*, HaarCascade_opt opt_args);
}image_module;

typedef struct {
	float threshold;
	float scale_factor;
	rectangle_t* roi;
}find_features_opt;

typedef struct {
	int arg_c;
	int arg_thickness;
	bool arg_fill;
}draw_rectangle_opt;

typedef struct image_type {
	array_t * (*find_features)(cascade_t*, find_features_opt opt_args);
	void (*draw_rectangle)(rectangle_t*, draw_rectangle_opt opt_args);
	image_t* img;
}image_type;

#endif