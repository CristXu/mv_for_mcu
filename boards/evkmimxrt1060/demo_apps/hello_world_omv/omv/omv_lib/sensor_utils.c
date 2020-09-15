#include "sensor_utils.h"
#include "systick.h"

extern image_type img;
extern sensor_t g_sensors[];
extern sensor_t *g_pcur_sensor;
static image_t data;
image_type *snapshot(){
	sensor_snapshot(g_pcur_sensor, &data, NULL);
	img.img = &data;
	return &img;
}
int select_sensor(int idx){
	assert(idx < 5);
	extern int g_cur_sensor_idx;
	int old_idx = g_cur_sensor_idx;
	g_cur_sensor_idx = idx;
	g_pcur_sensor = &g_sensors[idx];
	return old_idx;
}
uint32_t systick_current_millis()
{
    return HAL_GetTick();
}
void sensor_skip_frames(uint32_t ms){
	uint32_t millis = systick_current_millis();
	while ((systick_current_millis() - millis) < ms) { // 32-bit math handles wrap arrounds...
			if (sensor_snapshot(g_pcur_sensor, NULL, NULL) == -1) {
				while(1);
			}
		}
}

int sensor_set_windowing_bundle(int x, int y, int w, int h){
	// test if the (w==h) ==0, means drop x, y, only has w,h 
	// array_len() == 2
	assert(x || y || w || h);
	int res_w = resolution[g_pcur_sensor->framesize][0];
	int res_h = resolution[g_pcur_sensor->framesize][1];
	if (!(w || h)) {
		w = x;
		h = y;
		x = (res_w / 2) - (w / 2);
		y = (res_h / 2) - (h / 2);
	}
	assert(w >= 8 && h >= 8);
	assert(x >= 0 && ((x + w) <= res_w) && (y >= 0) && ((y + h) <= res_h));
	return sensor_set_windowing(x, y, w, h);
}
int sensor_set_auto_gain_bundle(int enable, set_auto_gain_opt opt_args){
	float gain_db = opt_args.gain_db;
	float gain_db_ceiling = opt_args.gain_db_ceiling;
	return sensor_set_auto_gain(enable, gain_db, gain_db_ceiling);
}
int sensor_set_auto_whitebal_bundle(int enable, set_auto_whitebal_opt opt_args){
	float r_gain_db = opt_args.r_gain_db;
	float g_gain_db = opt_args.g_gain_db;
	float b_gain_db = opt_args.b_gain_db;	
	return sensor_set_auto_whitebal(enable, r_gain_db, g_gain_db, b_gain_db);
}
sensor_api sensor = {
	    // Sensor function pointers
   .reset = sensor_reset,
   .sleep = sensor_sleep,
   .read_reg = sensor_read_reg,
   .write_reg = sensor_write_reg,
   .set_pixformat = sensor_set_pixformat,
   .set_framesize = sensor_set_framesize,
   .set_framerate = sensor_set_framerate,
   .set_contrast = sensor_set_contrast,
   .set_brightness = sensor_set_brightness,
   .set_saturation = sensor_set_saturation,
   .set_gainceiling = sensor_set_gainceiling,
   .set_quality =  sensor_set_quality,
   .set_colorbar = sensor_set_colorbar,
   .set_auto_gain = sensor_set_auto_gain_bundle,
   .get_gain_db = sensor_get_gain_db,
   .set_auto_exposure = sensor_set_auto_exposure,
   .get_exposure_us = sensor_get_exposure_us,
   .set_auto_whitebal = sensor_set_auto_whitebal_bundle,
   .get_rgb_gain_db = sensor_get_rgb_gain_db,
   .set_hmirror = sensor_set_hmirror,
   .set_vflip = sensor_set_vflip,
   .set_special_effect = sensor_set_special_effect,
   .set_lens_correction = sensor_set_lens_correction,
   .ioctl =  sensor_ioctl,
   .skip_frames = sensor_skip_frames,
   
   .set_windowing = sensor_set_windowing_bundle,
   .snapshot = snapshot,
};