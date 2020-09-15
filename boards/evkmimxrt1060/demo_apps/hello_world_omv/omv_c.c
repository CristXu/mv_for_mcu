/*
 * Copyright (c) 2013 - 2015, Freescale Semiconductor, Inc.
 * Copyright 2016-2017 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "fsl_device_registers.h"
#include "fsl_debug_console.h"
#include "board.h"

#include "pin_mux.h"
#include "clock_config.h"

#include "systick.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/


/*******************************************************************************
 * Prototypes
 ******************************************************************************/

/*******************************************************************************
 * Code
 ******************************************************************************/
/*!
 * @brief Main function
 */
#include "utils.h"
//import img 
//import image

void HardFault_Handler(){
	__asm volatile (
		"bx lr \r\n":::
	);
}
void SystemInitHook(void){
	extern char Image$$VECTOR_ROM$$Base[];
	extern char Image$$VECTOR_ROM$$Limit[];
	extern char Image$$VECTOR_RAM$$Base[];
	#define _vector_rom Image$$VECTOR_ROM$$Base
	#define _vector_size (Image$$VECTOR_ROM$$Limit - Image$$VECTOR_ROM$$Base)
	#define _vector_ram Image$$VECTOR_RAM$$Base
	memcpy(_vector_ram, _vector_rom, _vector_size);
	SCB->VTOR = (volatile uint32_t)_vector_ram;
	__DSB();
}
#ifdef OMV_BLOB_DETECT
void threshold_cb(blob_obj_t *obj){
	PRINTF("In threshold, code %d \r\n", obj->code);
}
void merge_cb(blob_obj_t *o0, blob_obj_t *o1){
	PRINTF("In merge, code0 %d, code1 %d \r\n", o0->code, o1->code);	
}
#endif

int main(void)
{	
#ifdef SELF_RENAME_LIB	
	image_module image_lib = *module.image;
	sensor_api sensor_lib = *module.sensor;
	#define sensor sensor_lib 
	#define image image_lib 
#endif	
    /* Init board hardware. */
    BOARD_ConfigMPU();
    BOARD_InitBootPins();
    BOARD_InitBootClocks();
    BOARD_InitDebugConsole();
	extern void find_blobs();

	init_timer();

	sensor.reset();
	sensor.set_pixformat(PIXFORMAT_RGB565);
	sensor.set_framesize(FRAMESIZE_QQVGA);
	//sensor.set_windowing(256, 240, 0, 0);
	sensor.skip_frames(1000);
	#ifdef OMV_FACE_DETECT
	DECLARE_AND_INIT_FUNCTION_PARAMS(HaarCascade, haar_kw);
	cascade_t face_cascade = image.HaarCascade("frontalface", haar_kw);
	DECLARE_AND_INIT_FUNCTION_PARAMS(draw_rectangle, draw_rectangle_kw);
	DECLARE_AND_INIT_FUNCTION_PARAMS(find_features, find_features_kw);
	while (1)
	{
		#ifdef USING_SENSOR_API
		image_type img = *sensor.snapshot();
		#else
		extern image_type img;
		extern sensor_t s_sensor;
		image_t data;
		sensor_snapshot(&s_sensor, &data, NULL);
		img.img = data;
		#endif
		uint32_t start = HAL_GetTick();
		array_t * objects = img.find_features(&face_cascade, find_features_kw);
		extern void free_qrcode_obj(qrcode_t* qrcodes_obj);
		uint32_t end = HAL_GetTick();
		if(array_length(objects) > 0) 
			PRINTF("%d ms\r\n", (end - start));
		for(int i=0; i<array_length(objects); i++){
			rectangle_t *r = array_at(objects, i);
			img.draw_rectangle(r, draw_rectangle_kw);	
		}	
		array_free(objects);
	}
	#endif
	#ifdef OMV_QRCODE_DETECT
	DECLARE_AND_INIT_FUNCTION_PARAMS(draw_rectangle, draw_rectangle_kw);
	DECLARE_AND_INIT_FUNCTION_PARAMS(find_qrcodes, find_qrcodes_kw);
	DECLARE_AND_INIT_FUNCTION_PARAMS(lens_corr, lens_corr_kw);
	DECLARE_AND_INIT_FUNCTION_PARAMS(draw_string, draw_string_kw);
	while(1){
		image_type img = *sensor.snapshot();
		lens_corr_kw.strength = 2.8;
		//img.lens_corr(lens_corr_kw);
		uint32_t start = HAL_GetTick();
		qrcode_t* qrcode = img.find_qrcodes(find_qrcodes_kw);
		uint32_t end = HAL_GetTick();
		if(qrcode->size) 
			PRINTF("Total %d, %d ms\r\n", qrcode->size, (end - start));
		for(int i=0;i<qrcode->size;i++){
			qrcode_obj_t *cur = qrcode->qrcode + i;
			rectangle_t* rect = qrcode->rect(cur);
			img.draw_rectangle(rect, draw_rectangle_kw);
			img.draw_string(0, 0, cur->payload, draw_string_kw);
			qrcode->print(cur);
			free(rect);
		}
		free_qrcode_obj(qrcode);
	}
	#endif
	#ifdef OMV_APRITAG_DETECT
	DECLARE_AND_INIT_FUNCTION_PARAMS(draw_rectangle, draw_rectangle_kw);
	DECLARE_AND_INIT_FUNCTION_PARAMS(draw_cross, draw_cross_kw);
	DECLARE_AND_INIT_FUNCTION_PARAMS(find_apriltags, find_apriltags_kw);
	apriltag_families_t tag_families = 0;
	tag_families |= TAG16H5;
	tag_families |= TAG25H7;
	tag_families |= TAG25H9;
	tag_families |= TAG36H10;
	tag_families |= TAG36H11;
	tag_families |= ARTOOLKIT;
	find_apriltags_kw.families = tag_families;
	while(1){
		image_type img = *sensor.snapshot();
		uint32_t start = HAL_GetTick();
		apriltag_t* apriltag = img.find_apriltags(find_apriltags_kw);
		uint32_t end = HAL_GetTick();
		if(apriltag->size) 
			PRINTF("Total %d, %d ms\r\n", apriltag->size, (end - start));
		for(int i=0;i<apriltag->size;i++){
			apriltag_obj_t *cur = apriltag->apriltag + i;
			rectangle_t* rect = apriltag->rect(cur);
			img.draw_rectangle(rect, draw_rectangle_kw);
			img.draw_cross(cur->cx, cur->cy, draw_cross_kw);
			apriltag->print(cur);
			free(rect);
		}
		free_apriltag_obj(apriltag);
	}
	#endif
#ifdef OMV_BLOB_DETECT
	DECLARE_AND_INIT_FUNCTION_PARAMS(set_auto_gain, set_auto_gain_kw);
	DECLARE_AND_INIT_FUNCTION_PARAMS(set_auto_whitebal, set_auto_whitebal_kw);
	sensor.set_auto_gain(false, set_auto_gain_kw);
	sensor.set_auto_whitebal(false, set_auto_whitebal_kw);
	
	DECLARE_AND_INIT_FUNCTION_PARAMS(draw_rectangle, draw_rectangle_kw);
	DECLARE_AND_INIT_FUNCTION_PARAMS(draw_cross, draw_cross_kw);
	DECLARE_AND_INIT_FUNCTION_PARAMS(draw_string, draw_string_kw);
	char color[] ={255, 0, 0};
	draw_rectangle_kw.arg_c = (color_t){.num=3, .color=color};
	draw_cross_kw.arg_c = (color_t){.num=3, .color=color};
	draw_string_kw.arg_c = (color_t){.num=3, .color=color};
	
#ifdef AUTO_COLOR_TRACK
	rectangle_t roi = (rectangle_t){(160 - 50) >> 1, (120 - 50) >> 1, 50, 50};
	
	PRINTF("Auto algorithms done. Hold the object you want to track in front of the camera in the box.\r\n");
	PRINTF("MAKE SURE THE COLOR OF THE OBJECT YOU WANT TO TRACK IS FULLY ENCLOSED BY THE BOX!\r\n");
	for(int i=0;i<60;i++){
		image_type img = *sensor.snapshot();
		img.draw_rectangle(&roi, draw_rectangle_kw);
	}
	DECLARE_AND_INIT_FUNCTION_PARAMS(get_histogram, get_histogram_kw);
	HISTOGRAM_BINS_INIT(get_histogram_kw);
	get_histogram_kw.roi = &roi;
	
	DECLARE_AND_INIT_FUNCTION_PARAMS(find_blobs, find_blobs_kw);
	char thresholds[1][6] = {{50, 50, 0, 0, 0, 0}};
	THRESHOLD_INIT_HELPER(thresholds, find_blobs_kw);
	find_blobs_kw.area_threshold = 100;
	find_blobs_kw.pixels_threshold = 100;
	find_blobs_kw.merge = true;
	find_blobs_kw.margin = 10;
	char *threshold = thresholds[0];
	for(int i=0;i<60;i++){
		image_type img = *sensor.snapshot();
		hist_t* hist = img.get_histogram(get_histogram_kw);
		percentile_type_t* lo = hist->get_percentile(hist->self, 0.01);
		percentile_type_t* hi = hist->get_percentile(hist->self, 0.99);
		threshold[0] = (threshold[0] + lo->percentile->LValue) / 2;
		threshold[1] = (threshold[1] + hi->percentile->LValue) / 2;
		threshold[2] = (threshold[2] + lo->percentile->AValue) / 2;
		threshold[3] = (threshold[3] + hi->percentile->AValue) / 2;
		threshold[4] = (threshold[4] + lo->percentile->BValue) / 2;
		threshold[5] = (threshold[5] + hi->percentile->BValue) / 2;
		blob_t *blob = img.find_blobs(find_blobs_kw);
		for(int i=0;i<blob->size;i++){
			blob_obj_t *cur = blob->blob + i;
			rectangle_t* rect = blob->rect(cur);
			blob->print(cur);
			img.draw_rectangle(rect, draw_rectangle_kw);
			img.draw_cross(cur->cx, cur->cy, draw_cross_kw);
			img.draw_string(cur->x + 2, cur->y + 2, "got", draw_string_kw);
			free(rect);
			FREE_HIST_LIST(cur, false);
		}
		free_blob_obj(blob);
		free_percentile_obj(lo);
		free_percentile_obj(hi);		
		free_hist_obj(hist);
	}
#else
	char threshold[3][6] = {{30, 100, 15, 127, 15, 127},
							{30, 100, -64, -8, -32, 32},
							{0,  30,  0,   64, -128, 0},
							};
	DECLARE_AND_INIT_FUNCTION_PARAMS(find_blobs, find_blobs_kw);
	THRESHOLD_INIT_HELPER(threshold, find_blobs_kw);
	find_blobs_kw.merge = true;
	find_blobs_kw.pixels_threshold = 100;
	find_blobs_kw.area_threshold = 100;
	find_blobs_kw.threshold_cb = (void*)threshold_cb;
	find_blobs_kw.merge_cb = (void*)merge_cb;	
#endif
												
	while(1){
		image_type img = *sensor.snapshot();
		uint32_t start = HAL_GetTick();
		blob_t* blob = img.find_blobs(find_blobs_kw);;
		uint32_t end = HAL_GetTick();
		if(blob->size) 
			PRINTF("Total %d, %d ms\r\n", blob->size, (end - start));
		for(int i=0;i<blob->size;i++){
			blob_obj_t *cur = blob->blob + i;
			rectangle_t* rect = blob->rect(cur);
			blob->print(cur);
#ifdef MULTI_COLOR_TRACK
			if (cur->code == 3) { // r/g code
				img.draw_rectangle(rect, draw_rectangle_kw);
				img.draw_cross(cur->cx, cur->cy, draw_cross_kw);
				img.draw_string(cur->x + 2, cur->y + 2, "r/g", draw_string_kw);
			}
			if (cur->code == 5) { // r/b code
				img.draw_rectangle(rect, draw_rectangle_kw);
				img.draw_cross(cur->cx, cur->cy, draw_cross_kw);
				img.draw_string(cur->x + 2, cur->y + 2, "r/b", draw_string_kw);
			}
			if (cur->code == 6) { // g/b code
				img.draw_rectangle(rect, draw_rectangle_kw);
				img.draw_cross(cur->cx, cur->cy, draw_cross_kw);
				img.draw_string(cur->x + 2, cur->y + 2, "g/b", draw_string_kw);
			}
			if (cur->code == 7) { // r/g/b code
				img.draw_rectangle(rect, draw_rectangle_kw);
				img.draw_cross(cur->cx, cur->cy, draw_cross_kw);
				img.draw_string(cur->x + 2, cur->y + 2, "r/g/b", draw_string_kw);
			}
#endif
#if defined(SINGLE_COLOR_TRACK) ||  defined(AUTO_COLOR_TRACK)
			img.draw_rectangle(rect, draw_rectangle_kw);
			img.draw_cross(cur->cx, cur->cy, draw_cross_kw);
			img.draw_string(cur->x + 2, cur->y + 2, "got", draw_string_kw);
#endif
			free(rect);
			FREE_HIST_LIST(cur, false);
		}
		free_blob_obj(blob);
	}
	
#endif
	
	return -1;
}
