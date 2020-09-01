#include "image.h"
#include "stdlib.h"
#include "array.h"
#include "sensor.h"
#warning "#include ff_wrapper.h to enable file operations here"
#include "xalloc.h"
#include "fb_alloc.h"
#include "framebuffer.h"
#include "fsl_debug_console.h"
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
// qrcode 
rectangle_t *rect(qrcode_obj_t* self) {
	rectangle_t *rec = (rectangle_t *)malloc(sizeof(rectangle_t));
	rec->x = self->x;
	rec->y = self->y;
	rec->w = self->w;
	rec->h = self->h;
	return rec;
}
void qrcode_print(qrcode_obj_t* self) {
	PRINTF(
		"{\"x\":%d, \"y\":%d, \"w\":%d, \"h\":%d, \"payload\":\"%s\","
		" \"version\":%d, \"ecc_level\":%d, \"mask\":%d, \"data_type\":%d, \"eci\":%d}\r\n",
		(self->x),(self->y),(self->w),(self->h),(self->payload),(self->version),(self->ecc_level),(self->mask),(self->data_type),(self->eci));
	PRINTF("Corners in clock-wise: \r\n");
	for (int i = 0;i < 3;i++) {
		PRINTF("(%d, %d), ", self->corners.p[i].x, self->corners.p[i].y);
	}
	PRINTF("(%d, %d) \n\r", self->corners.p[3].x, self->corners.p[3].y);

}
void update_qrcode_obj(qrcode_obj_t* qrcode, find_qrcodes_list_lnk_data_t lnk_data) {
	// update corners
	// each corners has 4-points, differ from the MP's version in which has 4 corners for each with 1 point
	corners_t corners;
	for (int i = 0;i < 4;i++) {
		corners.p[i].x = lnk_data.corners[i].x;
		corners.p[i].y = lnk_data.corners[i].y;
	}
	qrcode->corners = corners;
	// update rect
	qrcode->x = lnk_data.rect.x;
	qrcode->y = lnk_data.rect.y;
	qrcode->w = lnk_data.rect.w;
	qrcode->h = lnk_data.rect.h;
	// update others.... 
	qrcode->payload = (char*)malloc(sizeof(char) * (lnk_data.payload_len+1));
	memcpy(qrcode->payload, lnk_data.payload, lnk_data.payload_len);
	qrcode->payload[lnk_data.payload_len] = '\0'; // add tail 0, as the legal string

	qrcode->version = lnk_data.version;
	qrcode->ecc_level = lnk_data.ecc_level;
	qrcode->mask = lnk_data.mask;
	qrcode->data_type = lnk_data.data_type;
	qrcode->eci = lnk_data.eci;
	xfree(lnk_data.payload);
}
#define INIT_QRCODE_LOCAL_DICTS(self) \
	self->rect = rect; \
	self->print = qrcode_print; 

qrcode_t* find_qrcodes(find_qrcodes_opt opt_args) {
	image_t* image = img.img;
	list_t out;
	find_qrcodes_list_lnk_data_t lnk_data;
	rectangle_t roi = {.x = 0, .y=0, .w=image->w, .h = image->h};
	if (opt_args.roi) {
		// update the roi with given value, if given_roi is not NULL
		memcpy(&roi, &opt_args.roi, sizeof(rectangle_t));
	}
	fb_alloc_mark();
	imlib_find_qrcodes(&out, image, &roi);
	fb_alloc_free_till_mark();
	size_t size = list_size(&out);
	// malloc the rect_obj for list_size(&out)
	qrcode_obj_t* qrcode = (qrcode_obj_t*)malloc(sizeof(qrcode_obj_t) * list_size(&out));
	for (size_t i = 0; list_size(&out); i++) {
		qrcode_obj_t* qrcode_cur = qrcode + i;
		list_pop_front(&out, &lnk_data);
		update_qrcode_obj(qrcode_cur, lnk_data);
	}
	qrcode_t* qrcode_array = (qrcode_t*)malloc(sizeof(qrcode_t));
	INIT_QRCODE_LOCAL_DICTS(qrcode_array);
	qrcode_array->qrcode = qrcode;
	qrcode_array->size = size;
	return (qrcode_array);
}
void free_qrcode_obj(qrcode_t* qrcodes_obj) {
	assert(qrcodes_obj);
	for (size_t i = 0; i < (qrcodes_obj->size); i++) {
		qrcode_obj_t* wait_for_free = qrcodes_obj->qrcode + i;
		xfree(wait_for_free->payload);
	}
	free(qrcodes_obj->qrcode);
	free(qrcodes_obj);
}

// lens corr
void lens_corr(lens_corr_opt opt_args){
    image_t *arg_img = img.img;
    float arg_strength = opt_args.strength;
    assert(arg_strength > 0.0);
    float arg_zoom = opt_args.zoom;
    assert(arg_zoom > 0.0);

    fb_alloc_mark();
    imlib_lens_corr(arg_img, arg_strength, arg_zoom);
    fb_alloc_free_till_mark();
}

void draw_string(int arg_x_off, int arg_y_off, const char* arg_str, draw_string_opt opt_args){
	image_t *arg_img = img.img;
    int arg_c = opt_args.arg_c;
    float arg_scale = opt_args.arg_scale;
    assert(0 < arg_scale);
    int arg_x_spacing = opt_args.arg_x_spacing;    
    int arg_y_spacing = opt_args.arg_y_spacing;        
    bool arg_mono_space = opt_args.arg_mono_space;
    int arg_char_rotation = opt_args.arg_char_rotation;
    int arg_char_hmirror = opt_args.arg_char_hmirror;
    int arg_char_vflip = opt_args.arg_char_vflip;
    int arg_string_rotation = opt_args.arg_string_rotation;
    int arg_string_hmirror = opt_args.arg_string_hmirror;
    int arg_string_vflip = opt_args.arg_string_vflip;
	imlib_draw_string(arg_img, arg_x_off, arg_y_off, arg_str,
                      arg_c, arg_scale, arg_x_spacing, arg_y_spacing, arg_mono_space,
                      arg_char_rotation, arg_char_hmirror, arg_char_vflip,
                      arg_string_rotation, arg_string_hmirror, arg_string_vflip);
}

image_module image = {
	.HaarCascade = HaarCascade,
};

image_type img =  {
	.img = NULL,
	.find_features = find_features,
	.find_qrcodes = find_qrcodes,
	.draw_rectangle = draw_rectangle,
	.lens_corr = lens_corr,
	.draw_string = draw_string,
};



