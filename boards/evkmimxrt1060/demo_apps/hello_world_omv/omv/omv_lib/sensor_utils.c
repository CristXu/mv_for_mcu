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
   .set_auto_gain = sensor_set_auto_gain,
   .get_gain_db = sensor_get_gain_db,
   .set_auto_exposure = sensor_set_auto_exposure,
   .get_exposure_us = sensor_get_exposure_us,
   .set_auto_whitebal = sensor_set_auto_whitebal,
   .get_rgb_gain_db = sensor_get_rgb_gain_db,
   .set_hmirror = sensor_set_hmirror,
   .set_vflip = sensor_set_vflip,
   .set_special_effect = sensor_set_special_effect,
   .set_lens_correction = sensor_set_lens_correction,
   .ioctl =  sensor_ioctl,
   .skip_frames = sensor_skip_frames,
   .snapshot = snapshot,
};