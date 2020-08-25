/*
 * This file is part of the OpenMV project.
 * Copyright (c) 2013/2014 Ibrahim Abdelkader <i.abdalkader@gmail.com>
 * This work is licensed under the MIT license, see the file LICENSE for details.
 *
 * Board configuration and pin definitions.
 *
 */
#ifndef __OMV_BOARDCONFIG_H__
#define __OMV_BOARDCONFIG_H__
// Architecture info
#define OMV_ARCH_STR            "OpenMV i.MX RT1050/60 port" // 33 chars max
#define OMV_BOARD_TYPE          "M7"
#define OMV_UNIQUE_ID_ADDR      0x1FF0F420

#define OMV_XCLK_MCO            (0U)
#define OMV_XCLK_TIM            (1U)

// Sensor external clock source.
#define OMV_XCLK_SOURCE         (OMV_XCLK_TIM)

// Sensor external clock timer frequency.
#define OMV_XCLK_FREQUENCY      (9000000)

// Sensor PLL register value.
// 0-3: reserved 4-5: full wiwn, 1/2 win, 1/4 win, low 2/3 win
// 6-7: bypass, 4x, 6x, 8x
#define OMV_OV7725_PLL_CONFIG   (0x01)  // x6

// Have built-in RGB->LAB table.
#define OMV_HAVE_LAB_TABLE

// Enable remove_shadows()
#define OMV_ENABLE_REMOVE_SHADOWS

// Enable linpolar()
#define OMV_ENABLE_LINPOLAR

// Enable logpolar()
#define OMV_ENABLE_LOGPOLAR

// Enable chrominvar()
#define OMV_ENABLE_CHROMINVAR

// Enable illuminvar()
#define OMV_ENABLE_ILLUMINVAR

// Enable rotation_corr()
#define OMV_ENABLE_ROTATION_CORR

// Enable get_similarity()
#define OMV_ENABLE_GET_SIMILARITY

// Enable find_lines()
#define OMV_ENABLE_FIND_LINES

// Enable find_line_segments()
#define OMV_ENABLE_FIND_LINE_SEGMENTS

// Enable find_circles()
#define OMV_ENABLE_FIND_CIRCLES

// Enable find_rects()
#define OMV_ENABLE_FIND_RECTS

// Enable find_qrcodes() (14 KB)
#define OMV_ENABLE_QRCODES

// Enable find_apriltags() (64 KB)
#define OMV_ENABLE_APRILTAGS

// Enable find_datamatrices() (26 KB)
#define OMV_ENABLE_DATAMATRICES

// Enable find_barcodes() (42 KB)
#define OMV_ENABLE_BARCODES

// Enable find_displacement()
#ifdef OMV_ENABLE_ROTATION_CORR
#define OMV_ENABLE_FIND_DISPLACEMENT
#endif

#ifndef MINIMUM_IMAGE
// enable SDRAM fb alloc region
#define OMV_FB_OVERLAY_MEMORY 1
#endif

// Enable LENET (200+ KB).
#define OMV_ENABLE_LENET

// Bootloader LED GPIO port/pin
#define OMV_BOOTLDR_LED_PIN     (GPIO_PIN_1)
#define OMV_BOOTLDR_LED_PORT    (GPIOC)

#define OMV_UMM_BLOCK_SIZE      256
// Linker script constants (see the linker script template stm32fxxx.ld.S).
// Note: fb_alloc is a stack-based, dynamically allocated memory on FB.
// The maximum available fb_alloc memory = FB_ALLOC_SIZE + FB_SIZE - (w*h*bpp).
#define OMV_FB_MEMORY       SRAM1   // Framebuffer, fb_alloc
#define OMV_MAIN_MEMORY     CCM     // data, bss, stack and heap
#define OMV_DMA_MEMORY      CCM     // Misc DMA buffers
#define OMV_STACK_SIZE      (8 * 1024)

#if defined(EVK1050_60_QSPI)
#define OMV_FB_SIZE         (601 * 1024)  // FB memory: header + VGA/GS image
#define OMV_FB_ALLOC_SIZE   (323 * 1024)   // minimum fb alloc size
#define OMV_JPEG_BUF_SIZE   (48 * 1024) // IDE JPEG buffer (header + data).
#else
#define OMV_FB_SIZE         (301 * 1024)  // FB memory: header + VGA/GS image
#define OMV_FB_ALLOC_SIZE   (83 * 1024)   // minimum fb alloc size
#endif
// RAW buffer size
#define OMV_RAW_BUF_SIZE        (OMV_FB_SIZE)

#endif //__OMV_BOARDCONFIG_H__
