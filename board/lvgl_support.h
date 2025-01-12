/*
 * Copyright 2019-2021 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef LVGL_SUPPORT_H
#define LVGL_SUPPORT_H

#include <stdint.h>
#include "display_support.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/

#define LCD_WIDTH             DEMO_BUFFER_WIDTH
#define LCD_HEIGHT            DEMO_BUFFER_HEIGHT
#define LCD_FB_BYTE_PER_PIXEL DEMO_BUFFER_BYTE_PER_PIXEL

/*******************************************************************************
 * API
 ******************************************************************************/

#ifdef __cplusplus
extern "C" {
#endif

//void lv_port_pre_init(void);
void lv_port_disp_init(void);
//void lv_port_indev_init(void);

#if (LV_USE_DRAW_VGLITE || LV_USE_DRAW_PXP || LV_USE_DRAW_VG_LITE)
void DEMO_CleanInvalidateCache(void);
void DEMO_CleanInvalidateCacheByAddr(void *addr, int32_t dsize);
#endif

#if defined(__cplusplus)
}
#endif

#endif /*LVGL_SUPPORT_H */
