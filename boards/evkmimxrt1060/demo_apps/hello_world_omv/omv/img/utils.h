#include "image.h"
#include "sensor_utils.h"
//#define img extern image_type img;
//#define image extern image_module image;
//#define import 
#define USING_SENSOR_API
// improt library
extern image_type img;
extern image_module image;
extern sensor_api sensor;

typedef struct omv_module {
	image_module* image;
	image_type* img;
	sensor_api* sensor;
}omv_module_t;

omv_module_t module = {
	.image = &image,
	.img = &img,
	.sensor = &sensor
};

