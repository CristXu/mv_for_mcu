#define MINIMUM_IMAGE   //software YUV_BRG & LAB_BGR
#define OMV_MAIN_FB_SIZE	(0x13000) // 76 * 1024
#define OMV_FB_ALLOC_SIZE	(0x11000) // 68 * 1024 

//#define OMV_FACE_DETECT 
//#define OMV_EYE_DETECT
//#define OMV_QRCODE_DETECT 
//#define OMV_APRITAG_DETECT
#define OMV_BLOB_DETECT

#ifdef OMV_APRITAG_DETECT
#undef OMV_MAIN_FB_SIZE
#undef OMV_FB_ALLOC_SIZE
#define OMV_MAIN_FB_SIZE	(0x9600) // 37 * 1024
#define OMV_FB_ALLOC_SIZE	(0x30000) // 40 * 1024 
#endif

#ifdef OMV_BLOB_DETECT
//#define SINGLE_COLOR_TRACK
//#define MULTI_COLOR_TRACK
#define AUTO_COLOR_TRACK
#undef OMV_MAIN_FB_SIZE
#undef OMV_FB_ALLOC_SIZE
#define __stack_size__      (0x2000)  // the blob need more stacks
#define OMV_MAIN_FB_SIZE	(0x9600) // 37 * 1024
#ifdef AUTO_COLOR_TRACK
#define OMV_FB_ALLOC_SIZE	(0x30000) // 40 * 1024 
#else
#define OMV_FB_ALLOC_SIZE	(0x30000) // 40 * 1024 
#endif
#endif

//#define REMOVE_QUIRC_RESIZE
#ifdef OMV_QRCODE_DETECT
#undef OMV_MAIN_FB_SIZE
#undef OMV_FB_ALLOC_SIZE
#ifdef REMOVE_QUIRC_RESIZE 
#define OMV_MAIN_FB_SIZE	(0x9600) // 37 * 1024
#define OMV_FB_ALLOC_SIZE	(0xA000) // 40 * 1024 
#else
#define OMV_MAIN_FB_SIZE	(0x9600) // 37 * 1024
#define OMV_FB_ALLOC_SIZE	(0x14000) // 80 * 1024 
#endif  //REMOVE_QUIRC_RESIZE
#endif  //OMV_QRCODE_DETECT


#ifdef OMV_FACE_DETECT
#undef OMV_MAIN_FB_SIZE
#undef OMV_FB_ALLOC_SIZE
#define OMV_MAIN_FB_SIZE 77 * 1024
#define OMV_FB_ALLOC_SIZE	66*1024 //83*1024
#endif

//if you do not need lcd to display, save much more mem : 480 * 272 * 2 *2B
#define NO_LCD_MONITOR