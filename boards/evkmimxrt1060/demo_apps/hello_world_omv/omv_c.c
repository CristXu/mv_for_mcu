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
#include "sensor.h"

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
	
	init_timer();

	sensor.reset();
	sensor.set_pixformat(PIXFORMAT_GRAYSCALE);
	sensor.set_framesize(FRAMESIZE_HQVGA);
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
			draw_rectangle_kw.arg_c = 0;
			img.draw_rectangle(rect, draw_rectangle_kw);
			img.draw_string(0, 0, cur->payload, draw_string_kw);
			qrcode->print(cur);
			free(rect);
		}
		free_qrcode_obj(qrcode);
	}
	#endif
}
