/*
 * Copyright 2020 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "FreeRTOS.h"
#include "task.h"

#include "fsl_debug_console.h"
#include "lvgl_support.h"
#include "pin_mux.h"
#include "board.h"
#include "lvgl.h"
#include "demos/lv_demos.h"

#include "fsl_soc_src.h"

#include "lvgl_demo_utils.h"

#ifdef SEGGER_SYSTEMVIEW
#include "SEGGER_SYSVIEW_FreeRTOS.h"
#include "SEGGER_RTT.h"
#endif
/*******************************************************************************
 * Definitions
 ******************************************************************************/
static volatile bool s_lvgl_initialized = false;

#ifdef SEGGER_SYSTEMVIEW
extern SEGGER_RTT_CB _SEGGER_RTT;
#endif

/*******************************************************************************
 * Prototypes
 ******************************************************************************/

void print_cb(lv_log_level_t level, const char * buf);

#if LV_USE_LOG
void print_cb(lv_log_level_t level, const char * buf)
{
    LV_UNUSED(level);

    PRINTF("%s\r\n", buf);
}
#endif

static void AppTask(void *param)
{
    lv_init();
    lv_port_disp_init();

#if LV_USE_LOG
    lv_log_register_print_cb(print_cb);
#endif

#ifdef SEGGER_SYSTEMVIEW
    LV_LOG("RTT block address is: 0x%x \r\n", &_SEGGER_RTT);
#endif

    s_lvgl_initialized = true;

    LV_LOG("lvgl demo started\r\n");
    lv_demo_widgets();

    for (;;)
    {
        lv_task_handler();
        vTaskDelay(5);
    }
}

/*******************************************************************************
 * Code
 ******************************************************************************/
/*!
 * @brief Main function
 */
int main(void)
{
    BaseType_t stat;

    /* Init board hardware. */
    BOARD_ConfigMPU();
    BOARD_BootClockRUN();

    /*
     * Reset the displaymix, otherwise during debugging, the
     * debugger may not reset the display, then the behavior
     * is not right.
     */

    BOARD_InitLpuartPins();
    BOARD_InitMipiPanelPins();
    BOARD_MIPIPanelTouch_I2C_Init();
    BOARD_InitDebugConsole();

    DEMO_InitUsTimer();

#ifdef SEGGER_SYSTEMVIEW
    traceINIT();
#endif

    stat = xTaskCreate(AppTask, "lvgl", 2 * 1024, NULL, tskIDLE_PRIORITY + 2, NULL);

    if (pdPASS != stat)
    {
        PRINTF("Failed to create lvgl task");
        while (1)
            ;
    }

    vTaskStartScheduler();

    for (;;)
    {
    } /* should never get here */
}

void SystemInitHook(void)
{
	SRC_AssertSliceSoftwareReset(SRC, kSRC_DisplaySlice);
#if (DEMO_DISPLAY_CONTROLLER == DEMO_DISPLAY_CONTROLLER_LCDIFV2)
	NVIC_ClearPendingIRQ(LCDIFv2_IRQn);
#endif
}

/*!
 * @brief Malloc failed hook.
 */
void vApplicationMallocFailedHook(void)
{
    PRINTF("Malloc failed. Increase the heap size.");

    for (;;)
        ;
}

/*!
 * @brief FreeRTOS tick hook.
 */
void vApplicationTickHook(void)
{
    if (s_lvgl_initialized)
    {
        lv_tick_inc(1);
    }
}

/*!
 * @brief Stack overflow hook.
 */
void vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName)
{
    (void)pcTaskName;
    (void)xTask;

    for (;;)
        ;
}
