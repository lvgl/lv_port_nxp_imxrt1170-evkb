/****************************************************************************
*
*    Copyright 2012 - 2020 Vivante Corporation, Santa Clara, California.
*    All Rights Reserved.
*
*    Permission is hereby granted, free of charge, to any person obtaining
*    a copy of this software and associated documentation files (the
*    'Software'), to deal in the Software without restriction, including
*    without limitation the rights to use, copy, modify, merge, publish,
*    distribute, sub license, and/or sell copies of the Software, and to
*    permit persons to whom the Software is furnished to do so, subject
*    to the following conditions:
*
*    The above copyright notice and this permission notice (including the
*    next paragraph) shall be included in all copies or substantial
*    portions of the Software.
*
*    THE SOFTWARE IS PROVIDED 'AS IS', WITHOUT WARRANTY OF ANY KIND,
*    EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
*    MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT.
*    IN NO EVENT SHALL VIVANTE AND/OR ITS SUPPLIERS BE LIABLE FOR ANY
*    CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT,
*    TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
*    SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*
*****************************************************************************/

#ifndef _VG_LITE_OS_H
#define _VG_LITE_OS_H

#include <stdint.h>

#if !defined(VG_DRIVER_SINGLE_THREAD)

#define vg_lite_os_set_event_state(event, state)      (event)->signal = state

#define vg_lite_os_event_state(event)                 (event)->signal

#define vg_lite_os_config_event(event, sem_id, state) \
    { \
        (event)->semaphore_id = sem_id; \
        (event)->signal = state; \
    }

typedef struct vg_lite_os_async_event
{
    uint32_t    semaphore_id; /*! The Id of the semaphore assigned to this event */
    int32_t     signal;       /*! The command buffer status */
}
vg_lite_os_async_event_t;

/*!
@brief  Set the value in a task’s thread local storage array.
*/
int32_t vg_lite_os_set_tls(void* tls);

/*!
@brief  Get the current task’s thread local storage array.
*/
void * vg_lite_os_get_tls( );

#endif /* not defined(VG_DRIVER_SINGLE_THREAD) */

/*!
@brief  Memory allocate.
*/
void * vg_lite_os_malloc(uint32_t size);

/*!
@brief  Memory free.
*/
void vg_lite_os_free(void * memory);

#if !defined(VG_DRIVER_SINGLE_THREAD)
/*!
@brief  Reset the value in a task’s thread local storage array.
*/
void vg_lite_os_reset_tls();
#endif /* not defined(VG_DRIVER_SINGLE_THREAD) */

/*!
@brief  sleep a number of milliseconds.
*/
void vg_lite_os_sleep(uint32_t msec);

/*!
@brief  initialize the os parameters.
*/
int32_t vg_lite_os_initialize();

/*!
@brief  deinitialize the os parameters.
*/
void vg_lite_os_deinitialize();

#if !defined(VG_DRIVER_SINGLE_THREAD)
/*!
@brief  Mutex semaphore take.
*/
int32_t vg_lite_os_lock();

/*!
@brief  Mutex semaphore give.
*/
int32_t vg_lite_os_unlock();

/*!
@brief  Submit the current command buffer to the command queue.
*/
int32_t vg_lite_os_submit(uint32_t context, uint32_t physical, uint32_t offset, uint32_t size, vg_lite_os_async_event_t *event);

/*!
@brief  Wait for the current command buffer to be executed.
*/
int32_t vg_lite_os_wait(uint32_t timeout, vg_lite_os_async_event_t *event);
#endif /* not defined(VG_DRIVER_SINGLE_THREAD) */

/*!
@brief  IRQ Handler.
*/
void vg_lite_os_IRQHandler(void);

/*!
@brief  Wait until an interrupt from the VGLite graphics hardware has been received.
*/
int32_t vg_lite_os_wait_interrupt(uint32_t timeout, uint32_t mask, uint32_t * value);

#if !defined(VG_DRIVER_SINGLE_THREAD)
/*!
@brief
*/
int32_t vg_lite_os_init_event(vg_lite_os_async_event_t *event,
                                      uint32_t semaphore_id,
                                      int32_t state);

/*!
@brief
*/
int32_t vg_lite_os_delete_event(vg_lite_os_async_event_t *event);

/*!
@brief
*/
int32_t vg_lite_os_wait_event(vg_lite_os_async_event_t *event);

/*!
@brief
*/
int32_t vg_lite_os_signal_event(vg_lite_os_async_event_t *event);

/*!
@brief
*/
int8_t vg_lite_os_query_context_switch(uint32_t context);

#endif /* not defined(VG_DRIVER_SINGLE_THREAD) */
#endif
