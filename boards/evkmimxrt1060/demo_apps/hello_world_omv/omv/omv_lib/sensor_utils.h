#ifndef SENSOR_UTILS_H
#define SENSOR_UTILS_H
#include "sensor.h"
#include "image.h"
typedef struct sensor_api {
	// Sensor function pointers
    int  (*reset)               ();
    int  (*sleep)               (int enable);
    int  (*read_reg)            (uint16_t reg_addr);
    int  (*write_reg)           (uint16_t reg_addr, uint16_t reg_data);
    int  (*set_pixformat)       (pixformat_t pixformat);
    int  (*set_framesize)       (framesize_t framesize);
    int  (*set_framerate)       (framerate_t framerate);
    int  (*set_contrast)        (int level);
    int  (*set_brightness)      (int level);
    int  (*set_saturation)      (int level);
    int  (*set_gainceiling)     (gainceiling_t gainceiling);
    int  (*set_quality)         (int quality);
    int  (*set_colorbar)        (int enable);
    int  (*set_auto_gain)       (int enable, float gain_db, float gain_db_ceiling);
    int  (*get_gain_db)         (float *gain_db);
    int  (*set_auto_exposure)   (int enable, int exposure_us);
    int  (*get_exposure_us)     (int *exposure_us);
    int  (*set_auto_whitebal)   (int enable, float r_gain_db, float g_gain_db, float b_gain_db);
    int  (*get_rgb_gain_db)     (float *r_gain_db, float *g_gain_db, float *b_gain_db);
    int  (*set_hmirror)         (int enable);
    int  (*set_vflip)           (int enable);
    int  (*set_special_effect)  (sde_t sde);
    int  (*set_lens_correction) (int enable, int radi, int coef);
    int  (*ioctl)               (int request, ...);
	void (*skip_frames)         (uint32_t ms);
	
	int (*set_windowing)        (int x, int y, int w, int h);
    image_type *(*snapshot)     ();
}sensor_api;
#endif