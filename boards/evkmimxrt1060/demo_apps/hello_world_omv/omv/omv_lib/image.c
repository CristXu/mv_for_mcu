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

char default_color = -1;

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
// helper 
#define LIST_MIN_ALLOC 4

hist_list_t *new_hist_list(size_t n, void* items, TYPE type ){
	hist_list_t * o = (hist_list_t *)malloc(sizeof(hist_list_t));
	o->type = type;
	o->len = n;
	size_t alloc = n < LIST_MIN_ALLOC ? LIST_MIN_ALLOC : n;
	switch(type){
		case TYPE_FLOAT:
			o->items = malloc(alloc * sizeof(float));	
			break;		
		case TYPE_INT:
			o->items = malloc(alloc * sizeof(int));	
			break;	
		case TYPE_U16:
			o->items = malloc(alloc * sizeof(uint16_t));	
			break;	
	}		
	if(items){
		for(int i=0; i<n; i++){
			switch(type){
				case TYPE_FLOAT:
					o->f_items[i] = ((float*)items)[i];	
					break;		
				case TYPE_INT:
					o->i_items[i] = ((int*)items)[i];
					break;	
				case TYPE_U16:
					o->u16_items[i] = ((uint16_t*)items)[i];
					break;	
			}	
		}
	}
	return o;
}
void free_hist_list(hist_list_t *list){
	free(list->items);
	free(list);
}
void helper_thresholds(threshold_list_t* arg, list_t *thresholds){
	if(!arg) return;
    uint32_t arg_thresholds_len = arg->row;
    char *arg_thresholds = arg->items;
    if (!arg_thresholds_len) return;
    for(uint32_t i = 0; i < arg_thresholds_len; i++) {
        uint32_t arg_threshold_len;
        char *arg_threshold = arg_thresholds + arg->col * i;   
        if (arg_threshold_len) {
            color_thresholds_list_lnk_data_t lnk_data;
            lnk_data.LMin = (arg_threshold_len > 0) ? IM_MAX(IM_MIN(arg_threshold[0],
                        IM_MAX(COLOR_L_MAX, COLOR_GRAYSCALE_MAX)), IM_MIN(COLOR_L_MIN, COLOR_GRAYSCALE_MIN)) :
                        IM_MIN(COLOR_L_MIN, COLOR_GRAYSCALE_MIN);
            lnk_data.LMax = (arg_threshold_len > 1) ? IM_MAX(IM_MIN(arg_threshold[1],
                        IM_MAX(COLOR_L_MAX, COLOR_GRAYSCALE_MAX)), IM_MIN(COLOR_L_MIN, COLOR_GRAYSCALE_MIN)) :
                        IM_MAX(COLOR_L_MAX, COLOR_GRAYSCALE_MAX);
            lnk_data.AMin = (arg_threshold_len > 2) ? IM_MAX(IM_MIN(arg_threshold[2], COLOR_A_MAX), COLOR_A_MIN) : COLOR_A_MIN;
            lnk_data.AMax = (arg_threshold_len > 3) ? IM_MAX(IM_MIN(arg_threshold[3], COLOR_A_MAX), COLOR_A_MIN) : COLOR_A_MAX;
            lnk_data.BMin = (arg_threshold_len > 4) ? IM_MAX(IM_MIN(arg_threshold[4], COLOR_B_MAX), COLOR_B_MIN) : COLOR_B_MIN;
            lnk_data.BMax = (arg_threshold_len > 5) ? IM_MAX(IM_MIN(arg_threshold[5], COLOR_B_MAX), COLOR_B_MIN) : COLOR_B_MAX;
            color_thresholds_list_lnk_data_t lnk_data_tmp;
            memcpy(&lnk_data_tmp, &lnk_data, sizeof(color_thresholds_list_lnk_data_t));
            lnk_data.LMin = IM_MIN(lnk_data_tmp.LMin, lnk_data_tmp.LMax);
            lnk_data.LMax = IM_MAX(lnk_data_tmp.LMin, lnk_data_tmp.LMax);
            lnk_data.AMin = IM_MIN(lnk_data_tmp.AMin, lnk_data_tmp.AMax);
            lnk_data.AMax = IM_MAX(lnk_data_tmp.AMin, lnk_data_tmp.AMax);
            lnk_data.BMin = IM_MIN(lnk_data_tmp.BMin, lnk_data_tmp.BMax);
            lnk_data.BMax = IM_MAX(lnk_data_tmp.BMin, lnk_data_tmp.BMax);
            list_push_back(thresholds, &lnk_data);
        }
    }
}
int helper_color(image_t *img, color_t *color){
	assert(color->num == 1 || color->num == 3);
	int default_val;
	if(color->num == 1){
		default_val = (int)(*(color->color));
	}
	if(color->num == 3){
		char *arg_color = color->color;
		default_val = COLOR_R8_G8_B8_TO_RGB565(IM_MAX(IM_MIN(arg_color[0], COLOR_R8_MAX), COLOR_R8_MIN),
                                                   IM_MAX(IM_MIN(arg_color[1], COLOR_G8_MAX), COLOR_G8_MIN),
                                                   IM_MAX(IM_MIN(arg_color[2], COLOR_B8_MAX), COLOR_B8_MIN));
		switch(img->bpp){
			case IMAGE_BPP_BINARY: {
				default_val = COLOR_RGB565_TO_BINARY(default_val);
				break;
			}
            case IMAGE_BPP_GRAYSCALE: {
                default_val = COLOR_RGB565_TO_GRAYSCALE(default_val);
                    break;
            }
		}
	}
	return default_val;
}

// image type
void draw_rectangle(rectangle_t* obj, draw_rectangle_opt opt_args){
	int arg_rx = obj->x;
	int arg_ry = obj->y;
	int arg_rw = obj->w;
	int arg_rh = obj->h;
	int arg_c = helper_color(img.img, &opt_args.arg_c);
	imlib_draw_rectangle(img.img, arg_rx, arg_ry, arg_rw, arg_rh, arg_c, opt_args.arg_thickness, opt_args.arg_fill);
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
rectangle_t *qrcode_rect(qrcode_obj_t* self) {
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
void update_qrcode_obj(qrcode_obj_t* qrcode, find_qrcodes_list_lnk_data_t* lnk_data) {
	// update corners
	// each corners has 4-points, differ from the MP's version in which has 4 corners for each with 1 point
	corners_t corners;
	for (int i = 0;i < 4;i++) {
		corners.p[i].x = lnk_data->corners[i].x;
		corners.p[i].y = lnk_data->corners[i].y;
	}
	qrcode->corners = corners;
	// update rect
	qrcode->x = lnk_data->rect.x;
	qrcode->y = lnk_data->rect.y;
	qrcode->w = lnk_data->rect.w;
	qrcode->h = lnk_data->rect.h;
	// update others.... 
	qrcode->payload = (char*)malloc(sizeof(char) * (lnk_data->payload_len+1));
	memcpy(qrcode->payload, lnk_data->payload, lnk_data->payload_len);
	qrcode->payload[lnk_data->payload_len] = '\0'; // add tail 0, as the legal string

	qrcode->version = lnk_data->version;
	qrcode->ecc_level = lnk_data->ecc_level;
	qrcode->mask = lnk_data->mask;
	qrcode->data_type = lnk_data->data_type;
	qrcode->eci = lnk_data->eci;
	xfree(lnk_data->payload);
}
#define INIT_QRCODE_LOCAL_DICTS(self) \
	self->rect = qrcode_rect; \
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
		update_qrcode_obj(qrcode_cur, &lnk_data);
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

//apriltags
rectangle_t *apriltag_rect(apriltag_obj_t* self) {
	rectangle_t *rec = (rectangle_t *)malloc(sizeof(rectangle_t));
	rec->x = self->x;
	rec->y = self->y;
	rec->w = self->w;
	rec->h = self->h;
	return rec;
}
void update_apriltag_obj(apriltag_obj_t* apriltag, find_apriltags_list_lnk_data_t* lnk_data) {
	// update corners
	// each corners has 4-points, differ from the MP's version in which has 4 corners for each with 1 point
	corners_t corners;
	for (int i = 0;i < 4;i++) {
		corners.p[i].x = lnk_data->corners[i].x;
		corners.p[i].y = lnk_data->corners[i].y;
	}
	apriltag->corners = corners;
	// update rect
	apriltag->x = lnk_data->rect.x;
	apriltag->y = lnk_data->rect.y;
	apriltag->w = lnk_data->rect.w;
	apriltag->h = lnk_data->rect.h;
	// update others.... 
	apriltag->id = lnk_data->id;
	apriltag->family = lnk_data->family;
	apriltag->cx = lnk_data->centroid.x;
	apriltag->cy = lnk_data->centroid.y;
	apriltag->rotation = lnk_data->z_rotation;
	apriltag->decision_margin = lnk_data->decision_margin;
	apriltag->hamming = lnk_data->hamming;
	apriltag->goodness = lnk_data->goodness;
	apriltag->x_translation = lnk_data->x_translation;
	apriltag->y_translation = lnk_data->y_translation;
	apriltag->z_translation = lnk_data->z_translation;
	apriltag->x_rotation = lnk_data->x_rotation;
	apriltag->y_rotation = lnk_data->y_rotation;
	apriltag->z_rotation = lnk_data->z_rotation;	
}
void apriltag_print(apriltag_obj_t* self) {
	char* familie_names[6] = {"TAG16H5", "TAG25H7", "TAG25H9", "TAG36H10", "TAG36H11", "ARTOOLKIT"};	
	PRINTF(
		  "{\"x\":%d, \"y\":%d, \"w\":%d, \"h\":%d, \"id\":%d,"
		  " \"family\":%s, \"cx\":%d, \"cy\":%d, \"rotation\":%f, \"decision_margin\":%f, \"hamming\":%d, \"goodness\":%f,"
		  " \"x_translation\":%f, \"y_translation\":%f, \"z_translation\":%f,"
		  " \"x_rotation\":%f, \"y_rotation\":%f, \"z_rotation\":%f}",
		  (self->x),(self->y),(self->w),(self->h),(self->id),(familie_names[IM_LOG2_8(self->family)]),(self->cx),(self->cy),(self->rotation),(self->decision_margin),(self->hamming), (self->goodness), 
	      (self->x_translation), (self->y_translation), (self->z_translation), 
	      (self->x_rotation), (self->y_rotation), (self->z_rotation));

}
#define INIT_APRILTAG_LOCAL_DICTS(self) \
	self->rect = apriltag_rect; \
	self->print = apriltag_print; 

apriltag_t* find_apriltags(find_apriltags_opt opt_args) {
	image_t* image = img.img;
	list_t out;
	find_apriltags_list_lnk_data_t lnk_data;
	rectangle_t roi = {.x = 0, .y=0, .w=image->w, .h = image->h};
	if (opt_args.roi) {
		// update the roi with given value, if given_roi is not NULL
		memcpy(&roi, &opt_args.roi, sizeof(rectangle_t));
	}
	apriltag_families_t families = opt_args.families;
	float fx, fy, cx, cy;
	fx = (opt_args.fx == -1) ? ((2.8 / 3.984) * image->w) : opt_args.fx;
	fy = (opt_args.fy == -1) ? ((2.8 / 2.952) * image->h) : opt_args.fy;
	cx = (opt_args.cx == -1) ? (0.5 * image->w) : opt_args.cx;
	cy = (opt_args.cy == -1) ? (0.5 * image->h) : opt_args.cy;
	fb_alloc_mark();
	imlib_find_apriltags(&out, image, &roi, families, fx, fy, cx, cy);
	fb_alloc_free_till_mark();
	size_t size = list_size(&out);
	// malloc the rect_obj for list_size(&out)
	apriltag_obj_t* apriltag = (apriltag_obj_t*)malloc(sizeof(apriltag_obj_t) * list_size(&out));
	for (size_t i = 0; list_size(&out); i++) {
		apriltag_obj_t* apriltag_cur = apriltag + i;
		list_pop_front(&out, &lnk_data);
		update_apriltag_obj(apriltag_cur, &lnk_data);
	}
	apriltag_t* apriltag_array = (apriltag_t*)malloc(sizeof(apriltag_t));
	INIT_APRILTAG_LOCAL_DICTS(apriltag_array);
	apriltag_array->apriltag = apriltag;
	apriltag_array->size = size;
	return (apriltag_array);
}

void free_apriltag_obj(apriltag_t* apriltags_obj) {
	assert(apriltags_obj);
	free(apriltags_obj->apriltag);
	free(apriltags_obj);
}

// blob 
rectangle_t *blob_rect(blob_obj_t* self) {
	rectangle_t *rec = (rectangle_t *)malloc(sizeof(rectangle_t));
	rec->x = self->x;
	rec->y = self->y;
	rec->w = self->w;
	rec->h = self->h;
	return rec;
}

void update_blob_obj(blob_obj_t* blob, find_blobs_list_lnk_data_t *lnk_data) {
	// update corners
	// each corners has 4-points, differ from the MP's version in which has 4 corners for each with 1 point
	corners_t corners;
	for (int i = 0;i < 4;i++) {
		corners.p[i].x = lnk_data->corners[(FIND_BLOBS_CORNERS_RESOLUTION*i)/4].x;
		corners.p[i].y = lnk_data->corners[(FIND_BLOBS_CORNERS_RESOLUTION*i)/4].y;
	}
	point_t min_corners_point[4];
	point_min_area_rectangle(lnk_data->corners, min_corners_point, FIND_BLOBS_CORNERS_RESOLUTION);
	corners_t min_corners;
	blob->corners = corners;
	// update rect
	blob->x = lnk_data->rect.x;
	blob->y = lnk_data->rect.y;
	blob->w = lnk_data->rect.w;
	blob->h = lnk_data->rect.h;
	blob->pixels = lnk_data->pixels;
	blob->cx = lnk_data->centroid_x;
	blob->cy = lnk_data->centroid_y;
	blob->rotation = lnk_data->rotation;
	blob->code = lnk_data->code;
	blob->count = lnk_data->count;
	blob->perimeter = lnk_data->perimeter;
	blob->roundness = lnk_data->roundness;
	blob->x_hist_bins = new_hist_list(lnk_data->x_hist_bins_count, NULL, TYPE_U16);
	blob->y_hist_bins = new_hist_list(lnk_data->y_hist_bins_count, NULL, TYPE_U16);
	for (int i = 0; i < lnk_data->x_hist_bins_count; i++) {
         (blob->x_hist_bins)->u16_items[i] = lnk_data->x_hist_bins[i];
    }

	for (int i = 0; i < lnk_data->y_hist_bins_count; i++) {
		(blob->y_hist_bins)->u16_items[i] = lnk_data->y_hist_bins[i];
	}

}
static void blob_print(blob_obj_t* self)
{
    PRINTF(   
		  "{\"x\":%d, \"y\":%d, \"w\":%d, \"h\":%d,"
		  " \"pixels\":%d, \"cx\":%d, \"cy\":%d, \"rotation\":%f, \"code\":%d, \"count\":%d,"
		  " \"perimeter\":%d, \"roundness\":%f}",
		  self->x,self->y,self->w,self->h,self->pixels,fast_roundf(self->cx),fast_roundf(self->cy),
		  (double)(self->rotation), (self->code),(self->count),(self->perimeter),(double)(self->roundness));
}
bool find_blobs_threshold_cb(void *fun_obj, find_blobs_list_lnk_data_t *blob){
	blob_obj_t* o = (blob_obj_t*)malloc(sizeof(blob_obj_t));
	update_blob_obj(o, blob);
	((void(*)(blob_obj_t*))fun_obj)(o);
	FREE_HIST_LIST(o, true);
	return true;
}
bool find_blobs_merge_cb(void *fun_obj, find_blobs_list_lnk_data_t *blob0, find_blobs_list_lnk_data_t *blob1){
	blob_obj_t* o0 = (blob_obj_t*)malloc(sizeof(blob_obj_t));
	update_blob_obj(o0, blob0);
	blob_obj_t* o1 = (blob_obj_t*)malloc(sizeof(blob_obj_t));
	update_blob_obj(o1, blob1);
	((void(*)(blob_obj_t*, blob_obj_t*))fun_obj)(o0, o1);
	FREE_HIST_LIST(o0, true);
	FREE_HIST_LIST(o0, true);
	return true;
}

#define INIT_BLOB_LOCAL_DICTS(self) \
	self->rect = blob_rect; \
	self->print = blob_print; 

blob_t* find_blobs(find_blobs_opt opt_args){
	image_t* arg_img = img.img;
	list_t out;
	find_blobs_list_lnk_data_t lnk_data;
	assert(opt_args.thr_list);
	list_t thresholds;
    list_init(&thresholds, sizeof(color_thresholds_list_lnk_data_t));
    helper_thresholds(opt_args.thr_list, &thresholds);
	rectangle_t roi = {.x = 0, .y=0, .w=arg_img->w, .h = arg_img->h};
	if (opt_args.roi) {
		// update the roi with given value, if given_roi is not NULL
		memcpy(&roi, &opt_args.roi, sizeof(rectangle_t));
	}
	uint32_t x_stride = opt_args.x_stride;
    assert(x_stride > 0);
    uint32_t y_stride = opt_args.y_stride;
	assert(y_stride > 0);
    uint32_t area_threshold = opt_args.area_threshold;
    uint32_t pixels_threshold = opt_args.pixels_threshold;
    bool merge = opt_args.merge;
	bool invert = opt_args.invert;
    int margin = opt_args.margin;
    void* threshold_cb = opt_args.threshold_cb;
    void* merge_cb = opt_args.merge_cb;
    uint32_t x_hist_bins_max = opt_args.x_hist_bins_max;
    uint32_t y_hist_bins_max = opt_args.y_hist_bins_max;
    fb_alloc_mark();
    imlib_find_blobs(&out, arg_img, &roi, x_stride, y_stride, &thresholds, invert,
            area_threshold, pixels_threshold, merge, margin,
            find_blobs_threshold_cb, threshold_cb, find_blobs_merge_cb, merge_cb, x_hist_bins_max, y_hist_bins_max);
    fb_alloc_free_till_mark();
    list_free(&thresholds);
	
	size_t size = list_size(&out);
	// malloc the rect_obj for list_size(&out)
	blob_obj_t* blob = (blob_obj_t*)malloc(sizeof(blob_obj_t) * list_size(&out));
	for (size_t i = 0; list_size(&out); i++) {
		blob_obj_t* blob_cur = blob + i;
		list_pop_front(&out, &lnk_data);
		update_blob_obj(blob_cur, &lnk_data);
	}
	blob_t* blob_array = (blob_t*)malloc(sizeof(blob_t));
	INIT_BLOB_LOCAL_DICTS(blob_array);
	blob_array->blob = blob;
	blob_array->size = size;
	return (blob_array);
}
void free_blob_obj(blob_t* blobs_obj) {
	assert(blobs_obj);
	free(blobs_obj->blob);
	free(blobs_obj);
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
// get percentile
void percentile_print(percentile_obj_t* self){
    switch(self->bpp) {
        case IMAGE_BPP_BINARY: {
            PRINTF("{\"value\":%d}", (self->LValue));
            break;
        }
        case IMAGE_BPP_GRAYSCALE: {
            PRINTF("{\"value\":%d}", (self->LValue));
            break;
        }
        case IMAGE_BPP_RGB565: {
			PRINTF("{\"l_value:%d\", \"a_value\":%d, \"b_value\":%d}",
                      (self->LValue),
                      (self->AValue),
                      (self->BValue));
            break;
        }
        default: {
            PRINTF("{}");
            break;
        }
    }	
}
percentile_type_t* get_percentile(void*self_in, float percentile){
	histogram_t hist;
    hist.LBinCount = (((hist_obj_t*)self_in)->LBins)->len;
    hist.ABinCount = (((hist_obj_t*)self_in)->ABins)->len;
    hist.BBinCount = (((hist_obj_t*)self_in)->BBins)->len;
    fb_alloc_mark();
    hist.LBins = fb_alloc(hist.LBinCount * sizeof(float), FB_ALLOC_NO_HINT);
    hist.ABins = fb_alloc(hist.ABinCount * sizeof(float), FB_ALLOC_NO_HINT);
    hist.BBins = fb_alloc(hist.BBinCount * sizeof(float), FB_ALLOC_NO_HINT);
	for (int i = 0; i < hist.LBinCount; i++) {
        hist.LBins[i] = (((hist_obj_t*)self_in)->LBins)->f_items[i];
    }

    for (int i = 0; i < hist.ABinCount; i++) {
        hist.ABins[i] = (((hist_obj_t*)self_in)->ABins)->f_items[i];
    }

    for (int i = 0; i < hist.BBinCount; i++) {
        hist.BBins[i] = (((hist_obj_t*)self_in)->BBins)->f_items[i];
    }
	
	percentile_t p;
    imlib_get_percentile(&p, ((hist_obj_t*)self_in)->bpp, &hist, percentile);
    fb_alloc_free_till_mark();

    percentile_obj_t *o = malloc(sizeof(percentile_obj_t));
    o->bpp = ((hist_obj_t*)self_in)->bpp;

    o->LValue = (p.LValue);
    o->AValue = (p.AValue);
    o->BValue = (p.BValue);
	
	percentile_type_t *percentile_type = malloc(sizeof(percentile_type_t));
	percentile_type->percentile = o;
	percentile_type->self = o;
	percentile_type->print = percentile_print;
	return percentile_type;
	
}

void free_percentile_obj(percentile_type_t* self){
	free(self->percentile);
	free(self);
}
// get histogram 
#define INIT_HIST_LOCAL_DICTS(self) \
	self->print = histogram_print; \
	self->get_percentile = get_percentile; 

void histogram_print(hist_obj_t* hist)
{
    hist_obj_t *self = hist;
	PRINTF("{\"bins\":");
    switch(self->bpp) {
        case IMAGE_BPP_BINARY: {			
			for(int i=0;i<self->LBins->len;i++)
				 PRINTF("%d, ", self->LBins->f_items[i]);
            break;
        }
        case IMAGE_BPP_GRAYSCALE: {
			for(int i=0;i<self->LBins->len;i++)
				 PRINTF("%d, ", self->LBins->f_items[i]);
            break;
        }
        case IMAGE_BPP_RGB565: {
            PRINTF("(%d, %d, %d)", self->LBins, self->ABins, self->BBins);
            break;
        }
        default: {
            break;
        }
    }
	PRINTF("}\r\n");
	return;
}

hist_t * get_histogram(get_histogram_opt opt_args){
	image_t* arg_img = img.img;
	list_t thresholds;
    list_init(&thresholds, sizeof(color_thresholds_list_lnk_data_t));
    helper_thresholds(opt_args.thr_list, &thresholds);	
	bool invert = opt_args.invert;
	rectangle_t roi = {.x = 0, .y=0, .w=arg_img->w, .h = arg_img->h};
	if (opt_args.roi) {
		// update the roi with given value, if given_roi is not NULL
		memcpy(&roi, opt_args.roi, sizeof(rectangle_t));
	}
	histogram_t hist;
	hist.LBinCount = opt_args.LBinCount;
	hist.ABinCount = opt_args.ABinCount;
	hist.BBinCount = opt_args.BBinCount;
	switch(arg_img->bpp){
		case IMAGE_BPP_BINARY:
			fb_alloc_mark();
			hist.LBins = fb_alloc(hist.LBinCount * sizeof(float), FB_ALLOC_NO_HINT);
			hist.ABins = NULL;
			hist.BBins = NULL;
			imlib_get_histogram(&hist, arg_img, &roi, &thresholds, invert);
			list_free(&thresholds);
			break;
		case IMAGE_BPP_GRAYSCALE:
            fb_alloc_mark();
            hist.LBins = fb_alloc(hist.LBinCount * sizeof(float), FB_ALLOC_NO_HINT);
            hist.ABins = NULL;
            hist.BBins = NULL;
            imlib_get_histogram(&hist, arg_img, &roi, &thresholds, invert);
            list_free(&thresholds);			
		case IMAGE_BPP_RGB565:	
            fb_alloc_mark();
            hist.LBins = fb_alloc(hist.LBinCount * sizeof(float), FB_ALLOC_NO_HINT);
            hist.ABins = fb_alloc(hist.ABinCount * sizeof(float), FB_ALLOC_NO_HINT);
            hist.BBins = fb_alloc(hist.BBinCount * sizeof(float), FB_ALLOC_NO_HINT);
            imlib_get_histogram(&hist, arg_img, &roi, &thresholds, invert);
            list_free(&thresholds);
		default:
			break;
	}			
	hist_t* hist_type = (hist_t*)malloc(sizeof(hist_t));
	hist_obj_t* hist_obj = (hist_obj_t*)malloc(sizeof(hist_obj_t));
	hist_obj->bpp = arg_img->bpp;
	hist_obj->LBins = new_hist_list(hist.LBinCount, NULL, TYPE_FLOAT);
	hist_obj->ABins = new_hist_list(hist.ABinCount, NULL, TYPE_FLOAT);
	hist_obj->BBins = new_hist_list(hist.BBinCount, NULL, TYPE_FLOAT);
	
	for (int i = 0; i < hist.LBinCount; i++) {
        (hist_obj->LBins)->f_items[i] = (hist.LBins[i]);
    }

    for (int i = 0; i < hist.ABinCount; i++) {
        (hist_obj->ABins)->f_items[i] = (hist.ABins[i]);
    }

    for (int i = 0; i < hist.BBinCount; i++) {
        (hist_obj->BBins)->f_items[i] = (hist.BBins[i]);
    }
	
	fb_alloc_free_till_mark();
	INIT_HIST_LOCAL_DICTS(hist_type);
	hist_type->hist = hist_obj;
	hist_type->self = hist_obj;
	return hist_type;
}

void free_hist_obj(hist_t* hist) {
	assert(hist);
	free_hist_list(hist->hist->LBins);
	free_hist_list(hist->hist->ABins);
	free_hist_list(hist->hist->BBins);
	free(hist);
}
void draw_string(int arg_x_off, int arg_y_off, const char* arg_str, draw_string_opt opt_args){
	image_t *arg_img = img.img;
    int arg_c = helper_color(arg_img, &opt_args.arg_c);
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

void draw_cross(int arg_x, int arg_y, draw_cross_opt opt_args){
	image_t *arg_img = img.img;
	int arg_c = helper_color(arg_img, &opt_args.arg_c);
	int arg_s = opt_args.arg_s;
	int arg_thickness = opt_args.arg_thickness;
	imlib_draw_line(arg_img, arg_x - arg_s, arg_y,         arg_x + arg_s, arg_y,         arg_c, arg_thickness);
	imlib_draw_line(arg_img, arg_x,         arg_y - arg_s, arg_x,         arg_y + arg_s, arg_c, arg_thickness);
}

image_module image = {
	.HaarCascade = HaarCascade,
};

#define OMV_BLOB_DETECT
image_type img =  {
#ifdef OMV_FACE_DETECT
	.find_features = find_features,
#endif
#ifdef OMV_QRCODE_DETECT
	.find_qrcodes = find_qrcodes,
#endif
#ifdef OMV_APRITAG_DETECT	
	.find_apriltags = find_apriltags,
#endif
#ifdef OMV_BLOB_DETECT
	.find_blobs = find_blobs,
	.get_histogram = get_histogram,
#endif
	.draw_rectangle = draw_rectangle,
	.lens_corr = lens_corr,
	.draw_string = draw_string,
	.draw_cross = draw_cross,
};



