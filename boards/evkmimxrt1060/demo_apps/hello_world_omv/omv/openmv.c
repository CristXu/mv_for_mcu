#include "openmv.h"
/* class fb */
char* _fballoc_start, * _fballoc_end;
void* init_fb_mem(size_t size) {
	_fballoc_start = (char*)malloc(sizeof(char) * size);
	_fballoc_end = _fballoc_start + size;
	fb_alloc_init0();
	return _fballoc_start;
}
/* class rect */
void rect_print(rect_obj_t *self) {
	printf("{\"x\":%d, \"y\":%d, \"w\":%d, \"h\":%d, \"magnitude\":%d}\n\r",
		self->x, self->y, self->w, self->h, self->magnitude);
	printf("Corners in clock-wise: ");
	for (int i = 0;i < 3;i++) {
		printf("(%d, %d), ", self->corners.p[i].x, self->corners.p[i].y);
	}
	printf("(%d, %d) \n\r", self->corners.p[3].x, self->corners.p[3].y);
}
void update_rect_obj(rect_obj_t* rect, find_rects_list_lnk_data_t lnk_data) {
	// update corners
	// each corners has 4-points, differ from the MP's version in which has 4 corners for each with 1 point
	corners_t corners;
	for (int i = 0;i < 4;i++) {
		corners.p[i].x = lnk_data.corners[i].x;
		corners.p[i].y = lnk_data.corners[i].y;
	}
	rect->corners = corners;
	// update rect
	rect->x = lnk_data.rect.x;
	rect->y = lnk_data.rect.y;
	rect->w = lnk_data.rect.w;
	rect->h = lnk_data.rect.h;
	// update magnitude 
	rect->magnitude = lnk_data.magnitude;
}
rect_t *find_rects(image_t* image, rectangle_t* given_roi, uint32_t thr) {
	list_t out;
	find_rects_list_lnk_data_t lnk_data;
	rectangle_t roi = { .x = 0,.y = 0,.w = image->w,.h = image->h };
	if (given_roi) {
		// update the roi with given value, if given_roi is not NULL
		memcpy(&roi, given_roi, sizeof(rectangle_t));
	}
	fb_alloc_mark();
	imlib_find_rects(&out, image, &roi, thr);
	fb_alloc_free_till_mark();
	size_t size = list_size(&out);
	// malloc the rect_obj for list_size(&out)
	rect_obj_t* rect = (rect_obj_t*)malloc(sizeof(rect_obj_t) * list_size(&out));
	for (size_t i = 0; list_size(&out); i++) {
		rect_obj_t* rect_cur = rect + i;
		list_pop_front(&out, &lnk_data);
		update_rect_obj(rect_cur, lnk_data);
	}
	rect_t *rect_array = (rect_t*)malloc(sizeof(rect_t));
	rect_array->rect = rect;
	rect_array->size = size;
	return (rect_array);
}
/* class apriltags */
/* class qrcodes */
void qrcode_print(qrcode_obj_t* self) {
	printf(
		"{\"x\":%d, \"y\":%d, \"w\":%d, \"h\":%d, \"payload\":\"%s\","
		" \"version\":%d, \"ecc_level\":%d, \"mask\":%d, \"data_type\":%d, \"eci\":%d}",
		(self->x),(self->y),(self->w),(self->h),(self->payload),(self->version),(self->ecc_level),(self->mask),(self->data_type),(self->eci));
	printf("Corners in clock-wise: ");
	for (int i = 0;i < 3;i++) {
		printf("(%d, %d), ", self->corners.p[i].x, self->corners.p[i].y);
	}
	printf("(%d, %d) \n\r", self->corners.p[3].x, self->corners.p[3].y);

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
qrcode_t* find_qrcodes(image_t* image, rectangle_t* given_roi) {
	list_t out;
	find_qrcodes_list_lnk_data_t lnk_data;
	rectangle_t roi = {.x = 0, .y=0, .w=image->w, .h = image->h};
	if (given_roi) {
		// update the roi with given value, if given_roi is not NULL
		memcpy(&roi, given_roi, sizeof(rectangle_t));
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
	qrcode_array->qrcode = qrcode;
	qrcode_array->size = size;
	return (qrcode_array);
}
void free_qrcode_obj(qrcode_t* qrcodes_obj) {
	assert(qrcodes_obj);
	for (size_t i = 0; i < (qrcodes_obj->size); i++) {
		qrcode_obj_t* wait_for_free = qrcodes_obj->qrcode + i;
		free(wait_for_free->payload);
	}
	free(qrcodes_obj->qrcode);
	free(qrcodes_obj);
}

