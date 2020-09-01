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

#define find_qrcodes_opt_default { \
	.roi = NULL, \
};

#define lens_corr_opt_default { \
	.strength = 1.8, \
	.zoom = 1.0, \
};

#define DECLARE_AND_INIT_FUNCTION_PARAMS(func, parms_name) \
	func##_opt parms_name = func##_opt_default; 

//image_module typedef
typedef struct {
	int stages;
}HaarCascade_opt;
typedef struct image_module{
	cascade_t  (*HaarCascade)(char*, HaarCascade_opt opt_args);
}image_module;

// image_type typedef
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

typedef struct _corners {
	point_t p[4];
} corners_t;
typedef struct qrcode_obj {
	corners_t corners;
	uint32_t  x, y, w, h, version, ecc_level, mask, data_type, eci;
	char* payload;
} qrcode_obj_t;
typedef struct _qrcode {
	qrcode_obj_t* qrcode;
	size_t size;
	rectangle_t* (*rect)(qrcode_obj_t*);
	void (*print)(qrcode_obj_t*);
} qrcode_t;

typedef struct {
	rectangle_t* roi;
}find_qrcodes_opt;

void free_qrcode_obj(qrcode_t* qrcodes_obj);

typedef struct {
	float strength;
	float zoom;
}lens_corr_opt;

typedef struct image_type {
	image_t* img;
	array_t* (*find_features)(cascade_t*, find_features_opt opt_args);
	void (*draw_rectangle)(rectangle_t*, draw_rectangle_opt opt_args);
	qrcode_t* (*find_qrcodes)(find_qrcodes_opt opt_args);
	void (*lens_corr)(lens_corr_opt opt_args);
}image_type;

#endif