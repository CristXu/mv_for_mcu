#ifndef IMAGE_H 
#define IMAGE_H
#include <arm_math.h>
#include "imlib.h"
// init helper 
extern char default_color;
#define HaarCascade_opt_default  { \
	.stages = 25, \
};

#define get_histogram_opt_default { \
	.roi = NULL, \
	.thr_list = NULL, \
	.invert = false, \
};

#define HISTOGRAM_BINS_INIT(opt_args_name) { \
	assert(img.img);   \
	image_bpp_t bpp = img.img->bpp; \
	int bins, lbins, abins, bbins; \
	switch(bpp){   \
		case IMAGE_BPP_BINARY: \
			bins = COLOR_BINARY_MAX-COLOR_BINARY_MIN+1; \
			opt_args_name.LBinCount = bins; \
			opt_args_name.ABinCount = 0; \
			opt_args_name.BBinCount = 0; \
			break; \
		case IMAGE_BPP_GRAYSCALE:    \
			bins = COLOR_GRAYSCALE_MAX-COLOR_GRAYSCALE_MIN+1; \
			opt_args_name.LBinCount = bins; \
			opt_args_name.ABinCount = 0; \
			opt_args_name.BBinCount = 0; \
			break; \
		case IMAGE_BPP_RGB565:\
			lbins = COLOR_L_MAX-COLOR_L_MIN+1; \
			abins = COLOR_A_MAX-COLOR_A_MIN+1; \
			bbins = COLOR_B_MAX-COLOR_B_MIN+1; \
			opt_args_name.LBinCount = lbins; \
			opt_args_name.ABinCount = abins; \
			opt_args_name.BBinCount = bbins; \
			break;\
		default: \
			break;\
	}\
}

#define draw_rectangle_opt_default  { \
	.arg_c = {1, &default_color}, \
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

#define find_apriltags_opt_default { \
	.families = TAG36H11, \
	.fx = -1, \
	.fy = -1, \
	.cx = -1, \
	.cy = -1, \
	.roi = NULL, \
};

#define find_blobs_opt_default { \
	.thr_list = NULL, \
	.roi = NULL, \
	.x_stride = 2, \
	.y_stride = 1, \
	.area_threshold = 10, \
	.pixels_threshold = 10, \
	.merge = false, \
	.invert = false, \
	.margin = 0, \
	.threshold_cb = NULL, \
	.merge_cb = NULL, \
	.x_hist_bins_max = 0, \
	.y_hist_bins_max = 0, \
};

#define THRESHOLD_INIT_HELPER(array, name) \
	threshold_list_t thr_list = {   \
		.row = sizeof(array) / sizeof(array[0]), \
		.col = sizeof(array[0]), \
		.items = (char*)array, \
	};  \
	name.thr_list = &thr_list;

#define lens_corr_opt_default { \
	.strength = 1.8, \
	.zoom = 1.0, \
};

#define draw_string_opt_default { \
	.arg_c = {1, &default_color}, \
	.arg_scale = 1.0, \
	.arg_x_spacing = 0, \
	.arg_y_spacing = 0, \
	.arg_mono_space = true, \
	.arg_char_rotation = true, \
	.arg_char_hmirror = false, \
	.arg_char_vflip = false, \
	.arg_string_rotation = 0, \
	.arg_string_hmirror = false, \
	.arg_string_vflip = false, \
};

#define draw_cross_opt_default { \
	.arg_c = {1, &default_color},  \
	.arg_s = 5, \
	.arg_thickness = 1, \
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
	size_t num;
	char* color;
}color_t;

typedef struct {
	int row, col;
	char* items;
}threshold_list_t;

typedef struct {
	rectangle_t* roi;
	threshold_list_t* thr_list;
	bool invert;
	int LBinCount, ABinCount, BBinCount;	
}get_histogram_opt;

typedef struct {
	float threshold;
	float scale_factor;
	rectangle_t* roi;
}find_features_opt;

typedef struct {
	rectangle_t* roi;
}find_qrcodes_opt;

typedef struct {
	apriltag_families_t families;
	float fx, fy, cx, cy;
	rectangle_t* roi;
}find_apriltags_opt;

typedef struct {
	threshold_list_t *thr_list;
	rectangle_t *roi;
	uint32_t x_stride, y_stride, area_threshold, pixels_threshold;
	bool merge, invert;
	int margin;
	void* threshold_cb;
	void* merge_cb;
	uint32_t x_hist_bins_max, y_hist_bins_max;
}find_blobs_opt;

typedef struct {
	float strength;
	float zoom;
}lens_corr_opt;

typedef struct {
	color_t arg_c;
	float arg_scale;
	int arg_x_spacing;
	int arg_y_spacing;
	bool arg_mono_space;
	int arg_char_rotation;
	int arg_char_hmirror;
	int arg_char_vflip;
	int arg_string_rotation;
	int arg_string_hmirror;
	int arg_string_vflip;
}draw_string_opt;

typedef struct {
	color_t arg_c;
	int arg_thickness;
	bool arg_fill;
}draw_rectangle_opt;

typedef struct {
	color_t arg_c;
	int arg_s, arg_thickness;
}draw_cross_opt;

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
void free_qrcode_obj(qrcode_t* qrcodes_obj);

typedef struct apriltag_obj {
	corners_t corners;
	uint32_t  x, y, w, h, id, family, cx, cy, hamming;
	double rotation, decision_margin, goodness;
	double x_translation, y_translation, z_translation;
	double x_rotation, y_rotation, z_rotation;
} apriltag_obj_t;
typedef struct _apriltag {
	apriltag_obj_t* apriltag;
	size_t size;
	rectangle_t* (*rect)(apriltag_obj_t*);
	void (*print)(apriltag_obj_t*);
} apriltag_t;
void free_apriltag_obj(apriltag_t* apriltags_obj);

typedef enum {
	TYPE_FLOAT = 0,
	TYPE_INT,
	TYPE_U16,
} TYPE;
typedef struct {
	TYPE type;
	size_t len;
	union {
		void *items;
		int *i_items;
		float *f_items;
		uint16_t *u16_items;
	};
}hist_list_t;
typedef struct blob_obj {
	corners_t corners;
    corners_t min_corners;
    uint32_t x, y, w, h, pixels, code, count, perimeter;
    float cx, cy, rotation, roundness;
	hist_list_t* x_hist_bins;
    hist_list_t* y_hist_bins;
} blob_obj_t;
typedef struct _blob {
	blob_obj_t* blob;
	size_t size;
	rectangle_t* (*rect)(blob_obj_t*);
	void (*print)(blob_obj_t*);
} blob_t;
void free_blob_obj(blob_t* blob_obj);
void free_hist_list(hist_list_t *list);

typedef struct {
	image_bpp_t bpp;
	union {
		uint8_t value;
		uint8_t LValue;
	};
	int8_t AValue;
	int8_t BValue;
}percentile_obj_t;

typedef struct {
	percentile_obj_t* self;
	percentile_obj_t* percentile;
	void (*print)(percentile_obj_t*);
}percentile_type_t;
void free_percentile_obj(percentile_type_t* self);

typedef struct {
	image_bpp_t bpp;
	hist_list_t* LBins, *ABins, *BBins;
}hist_obj_t;

typedef struct {
	hist_obj_t* self;  // because the hist_t will always has one obj, so we can add an element that point to him
	hist_obj_t* hist;
	percentile_type_t* (*get_percentile)(void *, float);
	void (*print)(hist_obj_t*);
}hist_t;
void free_hist_obj(hist_t* hist);


#define FREE_HIST_LIST(o, free_o) \
	free_hist_list(o->x_hist_bins); \
	free_hist_list(o->y_hist_bins); \
	if(free_o) free(o);

typedef struct image_type {
	image_t* img;
	array_t* (*find_features)(cascade_t*, find_features_opt);
	void (*draw_rectangle)(rectangle_t*, draw_rectangle_opt);
	qrcode_t* (*find_qrcodes)(find_qrcodes_opt);
	apriltag_t* (*find_apriltags)(find_apriltags_opt);
	blob_t* (*find_blobs)(find_blobs_opt);
	hist_t* (*get_histogram)(get_histogram_opt);
	void (*lens_corr)(lens_corr_opt);
	void (*draw_string)(int, int, const char*, draw_string_opt);
	void (*draw_cross)(int, int, draw_cross_opt);
}image_type;

#endif