/*
 * This file is part of the OpenMV project.
 * Copyright (c) 2013/2014 Ibrahim Abdelkader <i.abdalkader@gmail.com>
 * This work is licensed under the MIT license, see the file LICENSE for details.
 *
 * Sensor abstraction layer.
 *
 */
// vivid omv, by vividsun, test 3, test 4
#include <stdlib.h>
#include <string.h>
#include "cambus.h"
#include "ov9650.h"
#include "ov2640.h"
#include "ov7725.h"
#include "ov7725_regs.h"
#include "mt9v034.h"
#include "sensor.h"
#include "framebuffer.h"
#include "fsl_clock.h"
#include "fsl_csi.h"
#include "fsl_debug_console.h"
#include "fsl_elcdif.h"
#include "fsl_edma.h"
#include "fsl_dmamux.h"
#include "fsl_cache.h"
#include "systick.h"

#define OV_CHIP_ID      (0x0A)
#define ON_CHIP_ID      (0x00)
#define MAX_XFER_SIZE (0xFFFC)

#ifdef BOARD_OMVRT1
	#define OV7725_I2C LPI2C4
	#ifndef NO_LCD_MONITOR
		#define NO_LCD_MONITOR
	#endif
#else
	#define OV7725_I2C LPI2C1
#endif

/* LCD definition. */
#define APP_ELCDIF LCDIF

#define APP_LCD_HEIGHT 272
#define APP_LCD_WIDTH 480
#define APP_HSW 41
#define APP_HFP 4
#define APP_HBP 8
#define APP_VSW 10
#define APP_VFP 4
#define APP_VBP 2
#define APP_LCD_POL_FLAGS \
    (kELCDIF_DataEnableActiveHigh | kELCDIF_VsyncActiveLow | kELCDIF_HsyncActiveLow | kELCDIF_DriveDataOnRisingClkEdge)

#define APP_LCDIF_DATA_BUS kELCDIF_DataBus16Bit

/* Display. */
#define LCD_DISP_GPIO GPIO1
#define LCD_DISP_GPIO_PIN 2
/* Back light. */
#define LCD_BL_GPIO GPIO2
#define LCD_BL_GPIO_PIN 31

#define APP_BPP 2
/* Camera definition. */
#define APP_CAMERA_HEIGHT 240
#define APP_CAMERA_WIDTH 320
#define APP_CAMERA_CONTROL_FLAGS (kCAMERA_HrefActiveHigh | kCAMERA_DataLatchOnRisingEdge)
#define APP_FRAME_BUFFER_COUNT 4
#define FRAME_BUFFER_ALIGN 64
uint32_t g_cur_sensor_idx = 0;
sensor_t g_sensors[1];
sensor_t* g_pcur_sensor = &g_sensors[0];

/*static*/ volatile uint8_t s_isOmvSensorSnapshotReady;


//This moment ,for the easy use, we do not take the DMA function to our project.
                                   //we will add it later.
#define DMA DMA0
#define DMA0_DMA16_DriverIRQHandler DMA_CH_0_16_DriverIRQHandler
#define BUFF_LENGTH 4U    
edma_handle_t g_EDMA_Handle;
volatile bool g_Transfer_Done = false;
/*******************************************************************************
 * Variables
 ******************************************************************************/
#ifndef NO_LCD_MONITOR
#define LCD_FB __attribute__((section(".lcd_fb")))
/*static*/ LCD_FB uint16_t s_frameBuffer[2][272][480] ;
#endif


const int resolution[][2] = {
    {0,    0   },
    // C/SIF Resolutions
    {88,   72  },    /* QQCIF     */
    {176,  144 },    /* QCIF      */
    {352,  288 },    /* CIF       */
    {88,   60  },    /* QQSIF     */
    {176,  120 },    /* QSIF      */
    {352,  240 },    /* SIF       */
    // VGA Resolutions
    {40,   30  },    /* QQQQVGA   */
    {80,   60  },    /* QQQVGA    */
    {160,  120 },    /* QQVGA     */
    {320,  240 },    /* QVGA      */
    {640,  480 },    /* VGA       */
    {60,   40  },    /* HQQQVGA   */
    {120,  80  },    /* HQQVGA    */
    {240,  160 },    /* HQVGA     */
    // FFT Resolutions
    {64,   32  },    /* 64x32     */
    {64,   64  },    /* 64x64     */
    {128,  64  },    /* 128x64    */
    {128,  128 },    /* 128x64    */
    // Other
    {128,  160 },    /* LCD       */
    {128,  160 },    /* QQVGA2    */
    {720,  480 },    /* WVGA      */
    {752,  480 },    /* WVGA2     */
    {800,  600 },    /* SVGA      */
    {1280, 1024},    /* SXGA      */
    {1600, 1200},    /* UXGA      */
};

void BOARD_InitLcd(void)
{
    volatile uint32_t i = 0x100U;

    gpio_pin_config_t config = {
        kGPIO_DigitalOutput, 0,
    };

    /* Reset the LCD. */
    GPIO_PinInit(LCD_DISP_GPIO, LCD_DISP_GPIO_PIN, &config);

    GPIO_PinWrite(LCD_DISP_GPIO, LCD_DISP_GPIO_PIN, 0);

    while (i--)
    {
    }

    GPIO_PinWrite(LCD_DISP_GPIO, LCD_DISP_GPIO_PIN, 1);

    /* Backlight. */
    config.outputLogic = 1;
    GPIO_PinInit(LCD_BL_GPIO, LCD_BL_GPIO_PIN, &config);
}

void BOARD_InitLcdifPixClock(void)
{
    /*
     * The desired output frame rate is 60Hz. So the pixel clock frequency is:
     * (480 + 41 + 4 + 18) * (272 + 10 + 4 + 2) * 60 = 9.2M.
     * Here set the LCDIF pixel clock to 9.3M.
     */

    /*
     * Initialize the Video PLL.
     * Video PLL output clock is OSC24M * (loopDivider + (denominator / numerator)) / postDivider = 93MHz.
     */
    clock_video_pll_config_t config = {
        .loopDivider = 31, .postDivider = 8, .numerator = 0, .denominator = 0,
    };

    CLOCK_InitVideoPll(&config);

    /*
     * 000 derive clock from PLL2
     * 001 derive clock from PLL3 PFD3
     * 010 derive clock from PLL5
     * 011 derive clock from PLL2 PFD0
     * 100 derive clock from PLL2 PFD1
     * 101 derive clock from PLL3 PFD1
     */
    CLOCK_SetMux(kCLOCK_LcdifPreMux, 2);

    CLOCK_SetDiv(kCLOCK_LcdifPreDiv, 4);

    CLOCK_SetDiv(kCLOCK_LcdifDiv, 1);
}

#ifdef BOARD_OMVRT1

#define SENSOR_RSTB_GPIO		GPIO1
#define SENSOR_RSTB_GPIO_PIN 	18

#define SENSOR_PWDN_GPIO		GPIO1
#define SENSOR_PWDN_GPIO_PIN 	19

#else
#define SENSOR_PWDN_GPIO		GPIO1
#define SENSOR_PWDN_GPIO_PIN 	4

#endif

void sensor_gpio_init(void)
{
    gpio_pin_config_t config = {
        kGPIO_DigitalOutput, 0,
    };

    /* Reset the LCD. */
	#ifdef BOARD_OMVRT1
    GPIO_PinInit(SENSOR_RSTB_GPIO, SENSOR_RSTB_GPIO_PIN, &config);
	#endif
	
	GPIO_PinInit(SENSOR_PWDN_GPIO, SENSOR_PWDN_GPIO_PIN, &config);	
}

void sensor_init0()      //make a note that we do not have the function of the jpeg,so no need to init the jpeg buffer
{
    // Clear framebuffers
    memset(MAIN_FB(), 0, sizeof(*MAIN_FB()));

}
uint32_t activeFrameAddr;
uint32_t inactiveFrameAddr;

#ifndef NO_LCD_MONITOR
void LCDMonitor_InitFB(void)
{
	int i, x,y;
	for (i=0; i<2; i++) {
		for (x=0;x<480;x++) {
			for (y=0;y<272;y++) {
				if (x % 10 < 8 && y % 10 < 8)
					s_frameBuffer[i][y][x] = 0;
				else
					s_frameBuffer[i][y][x] = (4 | 8<<6 | 4<<11);
			}
		}
	}
}

void LCDMonitor_Init(void)
{
    // Initialize the camera bus.
    BOARD_InitLcdifPixClock();
   // BOARD_InitDebugConsole();
    BOARD_InitLcd();	
    elcdif_rgb_mode_config_t lcdConfig = {
        .panelWidth = APP_LCD_WIDTH,
        .panelHeight = APP_LCD_HEIGHT,
        .hsw = APP_HSW,
        .hfp = APP_HFP,
        .hbp = APP_HBP,
        .vsw = APP_VSW,
        .vfp = APP_VFP,
        .vbp = APP_VBP,
        .polarityFlags = APP_LCD_POL_FLAGS,
        .pixelFormat = kELCDIF_PixelFormatRGB565,
        .dataBus = APP_LCDIF_DATA_BUS,
    };	
	LCDMonitor_InitFB();

    lcdConfig.bufferAddr = (uint32_t)activeFrameAddr;

    ELCDIF_RgbModeInit(APP_ELCDIF, &lcdConfig);

    ELCDIF_SetNextBufferAddr(APP_ELCDIF, (uint32_t)s_frameBuffer);
    ELCDIF_RgbModeStart(APP_ELCDIF);  	

}
void UnHook_OnUsbDbgScriptExec(void) {
    LCDMonitor_Init();
}
#endif

#define CSI_FRAG_MODE
#ifdef CSI_FRAG_MODE
CSI_Type *s_pCSI = CSI;

//				 			8bit | PixRisEdge | gatedClk  | SyncClrFifo| HSyncActHigh|SofOnVsyncRis|ExtVSync
#define CSICR1_INIT_VAL 	0<<0 | 1<<1	      | 1<<4	  | 1<<8	   | 1<<11		 | 1<<17	   |1<<30   

uint64_t s_dmaFragBufs[2][640 * 2 / 8];	// max supported line length

typedef struct _CSIIrq_t
{
	uint8_t isStarted;
	uint8_t isGray;
	uint32_t base0;
	uint32_t linePerFrag;
	uint32_t cnt;
	uint32_t dmaBytePerLine;
	uint32_t dmaBytePerFrag;
	uint32_t dmaFragNdx;

	uint32_t datBytePerLine;
	uint32_t datBytePerFrag;
	uint32_t datFragNdx;

	uint32_t datCurBase;

	uint32_t fragCnt;
	// in color mode, dmaFragNdx should == datLineNdx
	// in gray mode, to save memory, move backword nextDmaBulk every 4 lines
	
}CSIIrq_t;
volatile CSIIrq_t s_irq;

typedef union {
	uint8_t u8Ary[4][2];
	struct {
		uint8_t y0, u0, y1, v0, y2, u2, y3, v2;
	};
	
}YUV64bit_t;


#if defined(__CC_ARM) || defined(__clang__)
#define RAM_CODE __attribute__((section(".ram_code")))
#else
#define RAM_CODE
#endif

#ifdef __CC_ARM
#define ARMCC_ASM_FUNC	__asm
ARMCC_ASM_FUNC RAM_CODE uint32_t ExtractYFromYuv(uint32_t dmaBase, uint32_t datBase, uint32_t _128bitUnitCnt) {
	push	{r4-r7, lr}
10
	LDMIA	R0!, {r3-r6}
	// schedule code carefully to allow dual-issue on Cortex-M7
	bfi		r7, r3, #0, #8	// Y0
	bfi		ip, r5, #0, #8	// Y4
	lsr		r3,	r3,	#16
	lsr		r5,	r5,	#16
	bfi		r7, r3, #8, #8	// Y1
	bfi		ip, r5, #8, #8  // Y5
	bfi		r7, r4, #16, #8 // Y2
	bfi		ip, r6, #16, #8 // Y6
	lsr		r4,	r4,	#16
	lsr		r6,	r6,	#16
	bfi		r7, r4, #24, #8 // Y3
	bfi		ip, r6, #24, #8	// Y7
	STMIA	r1!, {r7, ip}
	
	subs	r2,	#1
	bne		%b10
	mov		r0,	r1
	pop		{r4-r7, pc}
}
#else
__attribute__((naked))
RAM_CODE uint32_t ExtractYFromYuv(uint32_t dmaBase, uint32_t datBase, uint32_t _128bitUnitCnt) {
	__asm volatile (
		"	push	{r1-r7, ip, lr}  \n "
		"10:  \n "
		"	ldmia	r0!, {r3-r6}  \n "
			// schedule code carefully to allow dual-issue on Cortex-M7
		"	bfi		r7, r3, #0, #8  \n "	// Y0
		"	bfi		ip, r5, #0, #8  \n "	// Y4
		"	lsr		r3,	r3,	#16  \n "
		"	lsr		r5,	r5,	#16  \n "
		"	bfi		r7, r3, #8, #8  \n "	// Y1
		"	bfi		ip, r5, #8, #8  \n "  // Y5
		"	bfi		r7, r4, #16, #8  \n " // Y2
		"	bfi		ip, r6, #16, #8  \n " // Y6
		"	lsr		r4,	r4,	#16  \n "
		"	lsr		r6,	r6,	#16  \n "
		"	bfi		r7, r4, #24, #8  \n " // Y3
		"	bfi		ip, r6, #24, #8  \n "	// Y7
		"	stmia	r1!, {r7, ip}  \n "	
		"	subs	r2,	#1  \n "
		"	bne		10b  \n "
		"	mov		r0,	r1  \n "
		"	pop		{r1-r7, ip, pc}  \n "		
	);
}

#endif

#ifdef CSI_FRAG_MODE
RAM_CODE void CSI_IRQHandler(void) {
    uint32_t csisr = s_pCSI->CSISR;
    /* Clear the error flags. */
    s_pCSI->CSISR = csisr;

	if (csisr & (1<<16)) {
		// VSync
		//               SOF    | FB1    | FB2    irqEn
		s_pCSI->CSICR1 = 1U<<16 | 1U<<19 | 1U<<20 | CSICR1_INIT_VAL;
		//				 64 doubleWords| RxFifoDmaReqEn| ReflashRFF|ResetFrmCnt
		s_pCSI->CSICR3 = 6<<4          | 1<<12         | 1<<14     |1<<15;
	} else if (csisr & (3<<19))
	{
		uint32_t dmaBase, lineNdx = s_irq.dmaFragNdx * s_irq.linePerFrag;
			if (s_irq.dmaFragNdx & 1)
				dmaBase = s_pCSI->CSIDMASA_FB2;
			else
				dmaBase = s_pCSI->CSIDMASA_FB1;
		if (dmaBase >= 0x20200000)
			DCACHE_CleanInvalidateByRange(dmaBase, s_irq.dmaBytePerFrag);
		if (s_irq.isGray || 
			(g_pcur_sensor->isWindowing &&  lineNdx >= g_pcur_sensor->wndY && lineNdx - g_pcur_sensor->wndY <= g_pcur_sensor->wndH) )
		{

			dmaBase += g_pcur_sensor->wndX * 2 * s_irq.linePerFrag;	// apply line window offset
			if (s_irq.isGray) {
				
				s_irq.datCurBase = ExtractYFromYuv(dmaBase, s_irq.datCurBase, (g_pcur_sensor->wndW * s_irq.linePerFrag) >> 3);
			} else {
				uint32_t byteToCopy = (g_pcur_sensor->wndW * s_irq.linePerFrag) << 1;
				memcpy((void*)s_irq.datCurBase, (void*)dmaBase, byteToCopy);
				s_irq.datCurBase += byteToCopy;
			}
		}
		
		if (++s_irq.dmaFragNdx == s_irq.fragCnt || (csisr & (3<<19)) == 3<<19 )
		{
			CSI_Stop(CSI);
			//				 64 doubleWords| ReflashRFF
			s_pCSI->CSICR3 = 6<<4		   | 1<<14;
			NVIC_DisableIRQ(CSI_IRQn);
			s_isOmvSensorSnapshotReady = 1;	
			goto Cleanup;
		}
		
		if (csisr & (1<<19) ) {
			if (!s_irq.isGray && !g_pcur_sensor->isWindowing)
				s_pCSI->CSIDMASA_FB1 += 2 * s_irq.dmaBytePerFrag;
		} else {
			if (!s_irq.isGray && !g_pcur_sensor->isWindowing)
				s_pCSI->CSIDMASA_FB2 += 2 * s_irq.dmaBytePerFrag;
			s_pCSI->CSICR3 |= CSI_CSICR3_DMA_REFLASH_RFF_MASK;	// reflash DMA
		}
	}
Cleanup:
	return;
}
#else
extern void CSI_DriverIRQHandler(void);    //warning:if no define this,will appera the situation that goto the default IRQ function!
RAM_CODE void CSI_IRQHandler(void)
{
	CSI_DriverIRQHandler();
}
#endif

void CsiFragModeInit(void) {
	CSI_Reset(CSI);
	
	s_pCSI->CSICR1 = CSICR1_INIT_VAL;
	s_pCSI->CSICR2 = 3U << 30;	// INCR16 for RxFIFO DMA
	s_pCSI->CSICR3 = 2U << 4;	// 16 double words to trigger DMA request
	s_pCSI->CSIFBUF_PARA = 0;	// no stride


	s_pCSI->CSICR18 = 13<<12 | 1<<18;	// HProt AHB bus protocol, write to memory when CSI_ENABLE is 1

	NVIC_SetPriority(CSI_IRQn, 0);
}

void CsiFragModeCalc(void) {
	s_irq.datBytePerLine = s_irq.dmaBytePerLine = g_pcur_sensor->fb_w * 2;
	if (g_pcur_sensor->pixformat == PIXFORMAT_GRAYSCALE) {
		s_irq.datBytePerLine /= 2;	// only contain Y
		s_irq.isGray = 1;
		g_pcur_sensor->gs_bpp = 1;
	} else {
		s_irq.isGray = 0;
		g_pcur_sensor->gs_bpp = 2;
	}
	if (g_pcur_sensor->fb_w == 0 || g_pcur_sensor->fb_h == 0)
		return;

	// calculate max bytes per DMA frag
	uint32_t dmaBytePerFrag, byteStep, dmaByteTotal;
	uint32_t maxBytePerLine = sizeof(s_dmaFragBufs) / ARRAY_SIZE(s_dmaFragBufs);
	dmaByteTotal = g_pcur_sensor->fb_w * g_pcur_sensor->fb_h * 2;	
	if (g_pcur_sensor->wndX == 0 && g_pcur_sensor->wndY == 0) // (s_irq.isGray)
	{
		dmaBytePerFrag = s_irq.dmaBytePerLine;  // set a minial default value
		for (byteStep = s_irq.dmaBytePerLine; byteStep < maxBytePerLine; byteStep += s_irq.dmaBytePerLine) {
			if (0 == byteStep % 32 )
			{
				// find maximum allowed bytes per frag
				dmaBytePerFrag = (maxBytePerLine / byteStep) * byteStep;
				for (; dmaBytePerFrag >= byteStep; dmaBytePerFrag -= byteStep) {
					if (dmaByteTotal % dmaBytePerFrag == 0)
						break;
				}
				if (dmaBytePerFrag < byteStep) {
					dmaBytePerFrag = byteStep;
					while (1) {}
				}
				break;
			}
		}
	} 
	else {
		// for window mode, we only accept 1 line per frag
		dmaBytePerFrag = s_irq.dmaBytePerLine;
	}
	s_irq.linePerFrag = dmaBytePerFrag / s_irq.dmaBytePerLine;
	s_irq.dmaBytePerFrag = dmaBytePerFrag;
	s_irq.datBytePerLine = s_irq.isGray ? dmaBytePerFrag / 2 : dmaBytePerFrag;

	// >>> calculate how many lines per fragment (DMA xfer unit)
	uint32_t burstBytes;
	if (!(s_irq.dmaBytePerLine % (8 * 16)))
	{
		burstBytes = 128;
		s_pCSI->CSICR2 = CSI_CSICR2_DMA_BURST_TYPE_RFF(3U);
		s_pCSI->CSICR3 = (CSI->CSICR3 & ~CSI_CSICR3_RxFF_LEVEL_MASK) | ((2U << CSI_CSICR3_RxFF_LEVEL_SHIFT));
	}
	else if (!(s_irq.dmaBytePerLine % (8 * 8)))
	{
		burstBytes = 64;
		s_pCSI->CSICR2 = CSI_CSICR2_DMA_BURST_TYPE_RFF(2U);
		s_pCSI->CSICR3 = (CSI->CSICR3 & ~CSI_CSICR3_RxFF_LEVEL_MASK) | ((1U << CSI_CSICR3_RxFF_LEVEL_SHIFT));
	}
	else
	{
		burstBytes = 32;
		s_pCSI->CSICR2 = CSI_CSICR2_DMA_BURST_TYPE_RFF(1U);
		s_pCSI->CSICR3 = (CSI->CSICR3 & ~CSI_CSICR3_RxFF_LEVEL_MASK) | ((0U << CSI_CSICR3_RxFF_LEVEL_SHIFT));
	}
	s_irq.fragCnt = g_pcur_sensor->fb_h / s_irq.linePerFrag;
	// <<<
}

void CsiFragModeStartNewFrame(void) {
	CsiFragModeCalc();
	s_irq.dmaFragNdx = 0;
	s_irq.cnt++;
	// DMA also writes to this cache line, to avoid being invalidated, clean MAIN_FB header.
	DCACHE_CleanByRange((uint32_t)MAIN_FB(), 32);
	if (s_irq.isGray || g_pcur_sensor->isWindowing) {
		s_pCSI->CSIDMASA_FB1 = (uint32_t) s_dmaFragBufs[0];
		s_pCSI->CSIDMASA_FB2 = (uint32_t) s_dmaFragBufs[1];
	} else {
		s_pCSI->CSIDMASA_FB1 = s_irq.base0;
		s_pCSI->CSIDMASA_FB2 = s_irq.base0 + s_irq.dmaBytePerFrag;
	}
	s_irq.datCurBase = s_irq.base0; // + g_pcur_sensor->wndY * s_irq.datBytePerLine + g_pcur_sensor->wndX * g_pcur_sensor->gs_bpp;
	s_pCSI->CSICR1 = CSICR1_INIT_VAL | 1<<16;	// enable SOF iRQ
	if (s_irq.dmaBytePerFrag & 0xFFFF0000) {
		
		uint32_t l16 = s_irq.linePerFrag , h16 = s_irq.dmaBytePerLine << 16;
		s_pCSI->CSIIMAG_PARA = l16 | h16;
	} else {
		s_pCSI->CSIIMAG_PARA = 1U | s_irq.dmaBytePerFrag << 16;	// set xfer cnt
	}
	__set_PRIMASK(1);
	s_pCSI->CSISR = s_pCSI->CSISR;
	s_pCSI->CSICR18 |= 1U<<31;	// start CSI
	NVIC_EnableIRQ(CSI_IRQn);
	__set_PRIMASK(0);	
}
#define CAMERA_TAKE_SNAPSHOT() CsiFragModeStartNewFrame()
#else
#define CAMERA_TAKE_SNAPSHOT() do { \
CAMERA_RECEIVER_SubmitEmptyBuffer(&cameraReceiver, (uint32_t)fb_framebuffer->pixels); \
/* fool the driver to make it think we have 2 FBs, otherwise it refuses to work */ \
CAMERA_RECEIVER_SubmitEmptyBuffer(&cameraReceiver, (uint32_t)fb_framebuffer->pixels); \
CAMERA_RECEIVER_Start(&cameraReceiver);  \
} while(0)

#endif

#define CAMERA_WAIT_FOR_SNAPSHOT() do { \
	while (0 == s_isOmvSensorSnapshotReady) {} \
	s_isOmvSensorSnapshotReady = 0; \
	}while(0)
volatile uint8_t s_isEnUsbIrqForSnapshot;

int sensor_init()
{   
	#ifndef XIP_EXTERNAL_FLASH
	s_isEnUsbIrqForSnapshot = 1;
	#endif
	sensor_gpio_init();
    cambus_init();
	memset(g_pcur_sensor, 0, sizeof(sensor_t));
	s_irq.base0 = (uint32_t)(MAIN_FB()->pixels);
    // Clear sensor chip ID.
    g_pcur_sensor->chip_id = 0;
    g_pcur_sensor->slv_addr = 0x21U; //?
    // Read ON semi sensor ID.
    cambus_readb(g_pcur_sensor->slv_addr, ON_CHIP_ID, &g_pcur_sensor->chip_id);
    if (g_pcur_sensor->chip_id == MT9V034_ID) {
        mt9v034_init(g_pcur_sensor);
    } else { // Read OV sensor ID.
        cambus_readb(g_pcur_sensor->slv_addr, OV_CHIP_ID, &g_pcur_sensor->chip_id);
        // Initialize sensor struct.
        switch (g_pcur_sensor->chip_id) {
            case OV9650_ID:
                ov9650_init(g_pcur_sensor);
                break;
            case OV2640_ID:
                ov2640_init(g_pcur_sensor);
                break;
            case OV7725_ID:
                ov7725_init(g_pcur_sensor);
                break;
            default:
                // Sensor is not supported.
                return -3;
        }
    }

	// sensor_set_pixformat(PIXFORMAT_RGB565);
	// sensor_set_framesize(FRAMESIZE_QVGA);	
	CsiFragModeInit();

	#ifndef NO_LCD_MONITOR // #ifdef __CC_ARM
	LCDMonitor_Init();
	#endif
#ifdef CSI_LINE_MODE

#endif
	
    return 0;
}
#define CSI_CLOCK_SHIFT    (9)
#define CSI_DIV_SHIFT      (11)
#define MAGIC_NUM          (0x80000000)
typedef enum {
	XTAL_24MHZ = 0,
	PLL2_396MHZ,
	USBPLL_120MHZ,
	USBPLL_664_63MHZ
}csi_clock;
typedef enum {
	DIV_1 = 0,
	DIV_2,
	DIV_3,
	DIV_4,
	DIV_5,
	DIV_6,
	DIV_7,	
	DIV_8	
}csi_div;
uint8_t s_isSensorInited;
int sensor_reset()
{
	if (!s_isSensorInited) {
		sensor_init0();
		sensor_init();	
	
	}	
	#ifndef NO_LCD_MONITOR
	LCDMonitor_InitFB();
	#endif
	g_pcur_sensor->isWindowing = 0;
	g_pcur_sensor->wndH = g_pcur_sensor->fb_h;
	g_pcur_sensor->wndW = g_pcur_sensor->fb_w;
	g_pcur_sensor->wndX = g_pcur_sensor->wndY = 0;	

	#ifdef BOARD_RTEVK
	// CSI clk src: 24MHz XTAL,
	sensor_set_framerate(MAGIC_NUM | (XTAL_24MHZ<<CSI_CLOCK_SHIFT|(DIV_2)<<CSI_DIV_SHIFT));	
	#else
	// CSI clk src: 480MHz USBPLL, CSI MCLK = 480 / 4 / 3 = 40MHz
	sensor_set_framerate(MAGIC_NUM | (USBPLL_120MHZ<<CSI_CLOCK_SHIFT|DIV_3<<CSI_DIV_SHIFT));
	#endif
    // Reset the sesnor state
    g_pcur_sensor->sde          = 0xFF;
    g_pcur_sensor->pixformat    = 0xFF;
    g_pcur_sensor->framesize    = 0xFF;
    g_pcur_sensor->framerate    = 0xFF;
    g_pcur_sensor->gainceiling  = 0xFF;


    // Call sensor-specific reset function; in the moment,we use our init function and defaults regs
    g_pcur_sensor->reset(g_pcur_sensor);
	/*
      // Reset all registers
    cambus_writeb(g_pcur_sensor->slv_addr, COM7, COM7_RESET);   
    OV7725_DelayMs(2);
    cambus_writes(g_pcur_sensor->slv_addr,ov7725InitRegs,ARRAY_SIZE(ov7725InitRegs));  
	*/
    return 0;
}

int sensor_get_id()
{
    return g_pcur_sensor->chip_id;
}

int sensor_set_vsync_output(GPIO_Type *gpio, uint32_t pin)
{
    // g_pcur_sensor->vsync_pin  = pin;
    // g_pcur_sensor->vsync_gpio = gpio;
    return 0;
}

int sensor_sleep(int enable)
{
    if (g_pcur_sensor->sleep == NULL
        || g_pcur_sensor->sleep(g_pcur_sensor, enable) != 0) {
        // Operation not supported
        return -1;
    }
    return 0;
}

int sensor_read_reg(uint16_t reg_addr)
{
    if (g_pcur_sensor->read_reg == NULL) {
        // Operation not supported
        return -1;
    }
    return g_pcur_sensor->read_reg(g_pcur_sensor, reg_addr);
}

int sensor_write_reg(uint16_t reg_addr, uint16_t reg_data)
{
    if (g_pcur_sensor->write_reg == NULL) {
        // Operation not supported
        return -1;
    }
    return g_pcur_sensor->write_reg(g_pcur_sensor, reg_addr, reg_data);
}

int sensor_set_pixformat(pixformat_t pixformat)
{

    if (g_pcur_sensor->pixformat == pixformat) {
        // No change
        return 0;
    }

    if (g_pcur_sensor->set_pixformat == NULL
        || g_pcur_sensor->set_pixformat(g_pcur_sensor, pixformat) != 0) {
        // Operation not supported
        return -1;
    }

    // Set pixel format
    g_pcur_sensor->pixformat = pixformat;

    // Set JPEG mode + no support function
    if (pixformat == PIXFORMAT_JPEG) {
        return -1;
    }

    // Skip the first frame.
    MAIN_FB()->bpp = 0;
	// CsiFragModeCalc();
    return 0;
}

int sensor_set_framesize(framesize_t framesize)
{

    // Call the sensor specific function
    if (g_pcur_sensor->set_framesize == NULL
        || g_pcur_sensor->set_framesize(g_pcur_sensor, framesize) != 0) {
        // Operation not supported
        return -1;
    }

    // Set framebuffer size
    g_pcur_sensor->framesize = framesize;

    // Skip the first frame.
    MAIN_FB()->bpp = -1;
    // Set MAIN FB width and height.
    g_pcur_sensor->fb_w = MAIN_FB()->w = resolution[framesize][0];
    g_pcur_sensor->fb_h = MAIN_FB()->h = resolution[framesize][1];

    // Set MAIN FB backup width and height.
    MAIN_FB()->u = resolution[framesize][0];
    MAIN_FB()->v = resolution[framesize][1];
	g_pcur_sensor->wndX = 0; g_pcur_sensor->wndY = 0 ; g_pcur_sensor->wndW = g_pcur_sensor->fb_w ; g_pcur_sensor->wndH = g_pcur_sensor->fb_h;
	// CsiFragModeCalc();
    return 0;
}

int sensor_set_framerate(framerate_t framerate)
{
    if (g_pcur_sensor->framerate == framerate) {
       /* no change */
        return 0;
    }
	if (framerate & 0x80000000)
		CCM->CSCDR3 = framerate & (0x1F<<9);

    /* call the sensor specific function */
    if (g_pcur_sensor->set_framerate == NULL
        || g_pcur_sensor->set_framerate(g_pcur_sensor, framerate) != 0) {
        /* operation not supported */
        return -1;
    }

    /* set the frame rate */
    g_pcur_sensor->framerate = framerate;

    return 0;
}

int sensor_set_windowing(int x, int y, int w, int h)      //may no this function in our RT csi,be used to set the output window,draw a rect in the picture
{
	w = (w + 7) & ~7 , x = (x + 7) & ~7;
	if (x >= g_pcur_sensor->fb_w - 8)
		x = g_pcur_sensor->fb_w - 8;
	if (y >= g_pcur_sensor->fb_h - 1)
		y = g_pcur_sensor->fb_h - 1;
	if (x + w > g_pcur_sensor->fb_w)
		w = g_pcur_sensor->fb_w - x;
	if (y + h > g_pcur_sensor->fb_h)
		h = g_pcur_sensor->fb_h - y;

	g_pcur_sensor->isWindowing = (w < g_pcur_sensor->fb_w && h < g_pcur_sensor->fb_h) ? 1 : 0;
	g_pcur_sensor->wndX = x ; g_pcur_sensor->wndY = y ; g_pcur_sensor->wndW = w ; g_pcur_sensor->wndH = h;
    MAIN_FB()->w = w;
    MAIN_FB()->h = h;
    return 0;
}

int sensor_set_contrast(int level)
{
    if (g_pcur_sensor->set_contrast != NULL) {
        return g_pcur_sensor->set_contrast(g_pcur_sensor, level);
    }
    return -1;
}

int sensor_set_brightness(int level)
{
    if (g_pcur_sensor->set_brightness != NULL) {
        return g_pcur_sensor->set_brightness(g_pcur_sensor, level);
    }
    return -1;
}

int sensor_set_saturation(int level)
{
    if (g_pcur_sensor->set_saturation != NULL) {
        return g_pcur_sensor->set_saturation(g_pcur_sensor, level);
    }
    return -1;
}

int sensor_set_gainceiling(gainceiling_t gainceiling)
{
    if (g_pcur_sensor->gainceiling == gainceiling) {
        /* no change */
        return 0;
    }

    /* call the sensor specific function */
    if (g_pcur_sensor->set_gainceiling == NULL
        || g_pcur_sensor->set_gainceiling(g_pcur_sensor, gainceiling) != 0) {
        /* operation not supported */
        return -1;
    }

    g_pcur_sensor->gainceiling = gainceiling;
    return 0;
}

int sensor_set_quality(int qs)
{
    /* call the sensor specific function */
    if (g_pcur_sensor->set_quality == NULL
        || g_pcur_sensor->set_quality(g_pcur_sensor, qs) != 0) {
        /* operation not supported */
        return -1;
    }
    return 0;
}

int sensor_set_colorbar(int enable)
{
    /* call the sensor specific function */
    if (g_pcur_sensor->set_colorbar == NULL
        || g_pcur_sensor->set_colorbar(g_pcur_sensor, enable) != 0) {
        /* operation not supported */
        return -1;
    }
    return 0;
}

int sensor_set_auto_gain(int enable, float gain_db, float gain_db_ceiling)
{
    /* call the sensor specific function */
    if (g_pcur_sensor->set_auto_gain == NULL
        || g_pcur_sensor->set_auto_gain(g_pcur_sensor, enable, gain_db, gain_db_ceiling) != 0) {
        /* operation not supported */
        return -1;
    }
    return 0;
}

int sensor_get_gain_db(float *gain_db)
{
    /* call the sensor specific function */
    if (g_pcur_sensor->get_gain_db == NULL
        || g_pcur_sensor->get_gain_db(g_pcur_sensor, gain_db) != 0) {
        /* operation not supported */
        return -1;
    }
    return 0;
}

int sensor_set_auto_exposure(int enable, int exposure_us)
{
    /* call the sensor specific function */
    if (g_pcur_sensor->set_auto_exposure == NULL
        || g_pcur_sensor->set_auto_exposure(g_pcur_sensor, enable, exposure_us) != 0) {
        /* operation not supported */
        return -1;
    }
    return 0;
}

int sensor_get_exposure_us(int *exposure_us)
{
    /* call the sensor specific function */
    if (g_pcur_sensor->get_exposure_us == NULL
        || g_pcur_sensor->get_exposure_us(g_pcur_sensor, exposure_us) != 0) {
        /* operation not supported */
        return -1;
    }
    return 0;
}

int sensor_set_auto_whitebal(int enable, float r_gain_db, float g_gain_db, float b_gain_db)
{
    /* call the sensor specific function */
    if (g_pcur_sensor->set_auto_whitebal == NULL
        || g_pcur_sensor->set_auto_whitebal(g_pcur_sensor, enable, r_gain_db, g_gain_db, b_gain_db) != 0) {
        /* operation not supported */
        return -1;
    }
    return 0;
}

int sensor_get_rgb_gain_db(float *r_gain_db, float *g_gain_db, float *b_gain_db)
{
    /* call the sensor specific function */
    if (g_pcur_sensor->get_rgb_gain_db == NULL
        || g_pcur_sensor->get_rgb_gain_db(g_pcur_sensor, r_gain_db, g_gain_db, b_gain_db) != 0) {
        /* operation not supported */
        return -1;
    }
    return 0;
}

int sensor_set_hmirror(int enable)
{
    /* call the sensor specific function */
    if (g_pcur_sensor->set_hmirror == NULL
        || g_pcur_sensor->set_hmirror(g_pcur_sensor, enable) != 0) {
        /* operation not supported */
        return -1;
    }
    return 0;
}

int sensor_set_vflip(int enable)
{
    /* call the sensor specific function */
    if (g_pcur_sensor->set_vflip == NULL
        || g_pcur_sensor->set_vflip(g_pcur_sensor, enable) != 0) {
        /* operation not supported */
        return -1;
    }
    return 0;
}

int sensor_set_special_effect(sde_t sde)
{
    if (g_pcur_sensor->sde == sde) {
        /* no change */
        return 0;
    }

    /* call the sensor specific function */
    if (g_pcur_sensor->set_special_effect == NULL
        || g_pcur_sensor->set_special_effect(g_pcur_sensor, sde) != 0) {
        /* operation not supported */
        return -1;
    }

    g_pcur_sensor->sde = sde;
    return 0;
}

int sensor_set_lens_correction(int enable, int radi, int coef)
{
    /* call the sensor specific function */
    if (g_pcur_sensor->set_lens_correction == NULL
        || g_pcur_sensor->set_lens_correction(g_pcur_sensor, enable, radi, coef) != 0) {
        /* operation not supported */
        return -1;
    }

    return 0;
}

int sensor_ioctl(int request, ... /* arg */)
{
    int ret = -1;
    if (g_pcur_sensor->ioctl != NULL) {
        va_list ap;
        va_start(ap, request);
        /* call the sensor specific function */
        ret = g_pcur_sensor->ioctl(g_pcur_sensor, request, ap);
        va_end(ap);
    }
    return ret;
}
int sensor_set_color_palette(const uint16_t *color_palette)
{
    g_pcur_sensor->color_palette = color_palette;
    return 0;
}

const uint16_t *sensor_get_color_palette()
{
    return g_pcur_sensor->color_palette;
}

static void sensor_check_bufsize()
{
    int bpp=0;
    switch (g_pcur_sensor->pixformat) {
        case PIXFORMAT_BAYER:
        case PIXFORMAT_GRAYSCALE:
            bpp = 1;
            break;
        case PIXFORMAT_YUV422:
        case PIXFORMAT_RGB565:
            bpp = 2;
            break;
        default:
            break;
    }

    if ((MAIN_FB()->w * MAIN_FB()->h * bpp) > OMV_RAW_BUF_SIZE) {
        if (g_pcur_sensor->pixformat == PIXFORMAT_GRAYSCALE) {
            // Crop higher GS resolutions to QVGA
            sensor_set_windowing(190, 120, 320, 240);
        } else if (g_pcur_sensor->pixformat == PIXFORMAT_RGB565) {
            // Switch to BAYER if the frame is too big to fit in RAM.
            sensor_set_pixformat(PIXFORMAT_BAYER);
        }
    }

}
#ifdef __CC_ARM
__asm uint16_t* LCDMonitor_UpdateLineGray(uint16_t *pLcdFB, uint16_t *pCamFB, uint32_t quadPixCnt) {
	push	{r4-r6, lr}
	mov		r5,	#0
	mov		r6,	#0
	add		r0,	r2,	lsl #3
10
	subs	r2,	r2,	#1
	
	ldr		r3, [r1], #4
	ldr		ip,	=0xFCFCFCFC
	and		r3,	r3,	ip
	lsr		r3,	r3,	#2
	lsr		r4,	r3,	#16
	bfi		r5,	r3,	#5,	#6
	bfi		r6,	r4, #5, #6
	rev16	r3,	r3
	rev16	r4,	r4
	bfi		r5,	r3,	#21, #6
	bfi		r6,	r4,	#21, #6
	
	rev16	r3,	r3
	rev16	r4,	r4
	ldr		ip,	=0xFEFE
	and		r3,	r3,	ip
	and		r4,	r4,	ip
	lsr		r3,	r3,	#1
	lsr		r4,	r4,	#1
	
	bfi		r5,	r3,	#0,	#5
	bfi		r6,	r4,	#0,	#5
	rev16	r3,	r3
	rev16	r4,	r4	
	bfi		r5,	r3,	#16,	#5
	bfi		r6,	r4,	#16,	#5
	rev16	r3,	r3
	rev16	r4,	r4
	bfi		r5,	r3,	#11,	#5
	bfi		r6,	r4,	#11,	#5
	rev16	r3,	r3
	rev16	r4,	r4	
	bfi		r5,	r3,	#27,	#5
	bfi		r6,	r4,	#27,	#5	
	
	ror		r5,	r5,	#16
	ror		r6,	r6,	#16	
	strd	r6,	r5,	[r0, #-8]!
	bne		%b10
	pop		{r4-r6, pc}
}

__asm uint16_t* LCDMonitor_UpdateLineRGB565(uint16_t *pLcdFB, uint16_t *pCamFB, uint32_t u64Cnt) {
	add		r0,	r2,	lsl #3
10
	subs	r2,	r2,	#1
	ldrd	r3, ip, [r1], #8
	rev16	r3,	r3
	rev16	ip,	ip
	ror		r3,	r3,	#16
	ror		ip,	ip,	#16
	strd	ip,	r3,	[r0, #-8]!
	bne		%b10
	bx		lr
}
#else
__attribute__((naked))
uint16_t* LCDMonitor_UpdateLineGray(uint16_t *pLcdFB, uint16_t *pCamFB, uint32_t quadPixCnt) {
	__asm volatile(
		"   push   {r0-r6, ip, lr} \n"  // we found GCC caller does not save these for naked callee!
		"	mov 	r5, #0	  \n "
		"	mov 	r6, #0	  \n "
		"10:	\n "
		"	subs	r2, r2, #1	  \n "
		"	ldr 	r3, [r1], #4	\n "
		"	ldr 	ip, =0xFCFCFCFC    \n "
		"	and 	r3, r3, ip	  \n "
		"	lsr 	r3, r3, #2	  \n "
		"	lsr 	r4, r3, #16    \n "
		"	bfi 	r5, r3, #5, #6	  \n "
		"	bfi 	r6, r4, #5, #6	  \n "
		"	rev16	r3, r3	  \n "
		"	rev16	r4, r4	  \n "
		"	bfi 	r5, r3, #21, #6    \n "
		"	bfi 	r6, r4, #21, #6    \n "
		"	rev16	r3, r3	  \n "
		"	rev16	r4, r4	  \n "
		"	ldr 	ip, =0xFEFE    \n "
		"	and 	r3, r3, ip	  \n "
		"	and 	r4, r4, ip	  \n "
		"	lsr 	r3, r3, #1	  \n "
		"	lsr 	r4, r4, #1	  \n "	
		"	bfi 	r5, r3, #0, #5	  \n "
		"	bfi 	r6, r4, #0, #5	  \n "
		"	rev16	r3, r3	  \n "
		"	rev16	r4, r4		\n "
		"	bfi 	r5, r3, #16,	#5	  \n "
		"	bfi 	r6, r4, #16,	#5	  \n "
		"	rev16	r3, r3	  \n "
		"	rev16	r4, r4	  \n "
		"	bfi 	r5, r3, #11,	#5	  \n "
		"	bfi 	r6, r4, #11,	#5	  \n "
		"	rev16	r3, r3	  \n "
		"	rev16	r4, r4	  \n "	
		"	bfi 	r5, r3, #27,	#5	  \n "
		"	bfi 	r6, r4, #27,	#5	  \n "	
		"	strd	r5, r6, [r0], #8	\n "
		"	bne 	10b    \n "
		"	pop 	{r0-r6, ip, pc}    \n "

	);
}

__attribute__((naked))
uint16_t* LCDMonitor_UpdateLineRGB565(uint16_t *pLcdFB, uint16_t *pCamFB, uint32_t u64Cnt) {
	__asm volatile(
	"   push   {r0-r3, ip} \n"  // we found GCC caller does not save these for naked callee!
	"10:  \n"
	"	subs	r2, r2, #1 \n "
	"	ldrd	r3, ip, [r1], #8  \n"
	"	rev16	r3, r3  \n"
	"	rev16	ip, ip  \n"	
	"	strd    r3, ip, [r0], #8  \n"
	"	bne 	10b  \n"
	"   pop     {r0-r3, ip} \n"
	"	bx		lr  \n"
	);
}

#endif

#ifndef NO_LCD_MONITOR
void LCDMonitor_Update(uint32_t fbNdx)
{
	uint32_t y, t1;
	uint16_t *pFB = (uint16_t*) MAIN_FB()->pixels;
	uint8_t *pFBGray = (uint8_t*) MAIN_FB()->pixels;
	uint16_t *pLcd = (uint16_t*) (s_frameBuffer[fbNdx & 1]);
	uint16_t *pLcdBkup;
	uint32_t h = g_pcur_sensor->wndH > 272 ? 272 : g_pcur_sensor->wndH;
	pLcdBkup = pLcd;
	
	pLcd += (480 - g_pcur_sensor->wndW) >> 1;
	pLcd += ((272 - h) >> 1) * 480;
	
	t1 = g_pcur_sensor->wndW * 2 / 8;
	if (s_irq.isGray) {
		pFBGray += (h - 1) * g_pcur_sensor->wndW;
		for (y=0; y< h; y++, pFBGray -= g_pcur_sensor->wndW) {
			LCDMonitor_UpdateLineGray(pLcd, (uint16_t*)pFBGray, t1);
			pLcd += 480;
		}
		ELCDIF_SetNextBufferAddr(LCDIF, (uint32_t) pLcdBkup);		
	}
	else {
		pFB += (h - 1) * g_pcur_sensor->wndW;
		for (y=0; y< h; y++, pFB -= g_pcur_sensor->wndW) {
			LCDMonitor_UpdateLineRGB565(pLcd, pFB, t1);
			pLcd += 480;
		}		
	}

	ELCDIF_SetNextBufferAddr(LCDIF, (uint32_t) pLcdBkup);
}
#endif

volatile uint32_t s_minProcessTicks = 10, s_jpegEncTicks;
int sensor_snapshot(sensor_t *sensor, image_t *pImg, streaming_cb_t streaming_cb)
{
    static uint32_t ls_prevTick;
  	sensor = sensor , streaming_cb = streaming_cb;	// keep compatible with original openMV
    sensor_check_bufsize();

    switch (g_pcur_sensor->pixformat) {
        case PIXFORMAT_GRAYSCALE:
            MAIN_FB()->bpp = 1;
            break;
        case PIXFORMAT_YUV422:
        case PIXFORMAT_RGB565:
            MAIN_FB()->bpp = 2;
            break;
        case PIXFORMAT_BAYER:
            MAIN_FB()->bpp = 3;
            break;
        case PIXFORMAT_JPEG:
            // Read the number of data items transferred
            // MAIN_FB()->bpp = (MAX_XFER_SIZE - __HAL_DMA_GET_COUNTER(&DMAHandle))*4;
            break;
		default:
			break;
    }    
	static uint8_t n;
    {
		uint32_t t1;
		#ifndef NO_LCD_MONITOR // #ifdef __CC_ARM
		LCDMonitor_Update(n);
		#endif  
        CAMERA_TAKE_SNAPSHOT();
		if (!s_isEnUsbIrqForSnapshot)
			NVIC_DisableIRQ(USB_OTG1_IRQn);
		CAMERA_WAIT_FOR_SNAPSHOT();
		if (!s_isEnUsbIrqForSnapshot)
			NVIC_EnableIRQ(USB_OTG1_IRQn);
        ls_prevTick = HAL_GetTick();		
		n++;
    }
	if (pImg) {
		pImg->w = MAIN_FB()->w , pImg->h = MAIN_FB()->h , pImg->bpp = MAIN_FB()->bpp;
		pImg->pixels = (uint8_t*) MAIN_FB()->pixels;		
	}
    return 0;
}

