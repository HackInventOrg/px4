/****************************************************************************
 *
 *   Copyright (C) 2012-2016 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/*
 * @file drv_input_capture.c
 *
 * Servo driver supporting input capture connected to STM32 timer blocks.
 *
 * Works with any of the 'generic' or 'advanced' STM32 timers that
 * have input pins.
 *
 * Require an interrupt.
 *
 * The use of thie interface is mutually exclusive with the pwm
 * because the same timers are used and there is a resource contention
 * with the ARR as it sets the pwm rate and in this driver needs to match
 * that of the hrt to back calculate the actual point in time the edge
 * was detected.
 *
 * This  is accomplished by taking the difference between the current
 * count rCNT snapped at the time interrupt and the rCCRx captured on the
 * edge transition. This delta is applied to hrt time and the resulting
 * value is the absolute system time the edge occured.
 *
 *
 */

#include <px4_config.h>
#include <nuttx/arch.h>
#include <nuttx/irq.h>

#include <sys/types.h>
#include <stdbool.h>

#include <assert.h>
#include <debug.h>
#include <time.h>
#include <queue.h>
#include <errno.h>
#include <string.h>
#include <stdio.h>

#include <arch/board/board.h>
#include <drivers/drv_input_capture.h>

extern int io_timer_validate_channel_index(unsigned channel);

int up_input_capture_set(unsigned channel, input_capture_edge edge, capture_filter_t filter,
			 capture_callback_t callback, void *context)
{
	int rv = io_timer_validate_channel_index(channel);
	return rv;
}

int up_input_capture_get_filter(unsigned channel, capture_filter_t *filter)
{
	int rv = io_timer_validate_channel_index(channel);
	return rv;
}

int up_input_capture_get_trigger(unsigned channel,  input_capture_edge *edge)
{
	int rv = io_timer_validate_channel_index(channel);
	return rv;
}

int up_input_capture_set_trigger(unsigned channel,  input_capture_edge edge)
{
	int rv = io_timer_validate_channel_index(channel);
	return rv;
}

int up_input_capture_get_callback(unsigned channel, capture_callback_t *callback, void **context)
{
	int rv = io_timer_validate_channel_index(channel);

	return rv;

}

int up_input_capture_set_callback(unsigned channel, capture_callback_t callback, void *context)
{
	int rv = io_timer_validate_channel_index(channel);
	return rv;
}

int up_input_capture_get_stats(unsigned channel, input_capture_stats_t *stats, bool clear)
{
	int rv = io_timer_validate_channel_index(channel);

	return rv;
}
