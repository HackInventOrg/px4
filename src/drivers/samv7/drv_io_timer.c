/****************************************************************************
 *
 *   Copyright (C) 2012 PX4 Development Team. All rights reserved.
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
 * @file drv_pwm_servo.c
 *
 * Servo driver supporting PWM servos connected to STM32 timer blocks.
 *
 * Works with any of the 'generic' or 'advanced' STM32 timers that
 * have output pins, does not require an interrupt.
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
#include <drivers/drv_pwm_output.h>
#include <drivers/drv_hrt.h>

typedef enum io_timer_channel_mode_t {
	IOTimerChanMode_NotUsed = 0,
	IOTimerChanMode_PWMOut  = 1,
	IOTimerChanMode_PWMIn   = 2,
	IOTimerChanMode_Capture = 3,
	IOTimerChanModeSize
} io_timer_channel_mode_t;

/* array of timers dedicated to PWM in and out and capture use */
typedef struct io_timers_t {
	uint32_t	base;
	uint32_t	clock_register;
	uint32_t	clock_bit;
	uint32_t	clock_freq;
	uint32_t	vectorno;
	uint32_t    first_channel_index;
	uint32_t    last_channel_index;
	xcpt_t      handler;
} io_timers_t;

/* array of channels in logical order */
typedef struct timer_io_channels_t {
	uint32_t	gpio_out;
	uint32_t	gpio_in;
	uint8_t		timer_index;
	uint8_t		timer_channel;
	uint16_t	masks;
	uint8_t		ccr_offset;
} timer_io_channels_t;



typedef uint8_t io_timer_channel_allocation_t; /* big enough to hold MAX_TIMER_IO_CHANNELS */
typedef void (*channel_handler_t)(void *context, const io_timers_t *timer, uint32_t chan_index,
				  const timer_io_channels_t *chan,
				  hrt_abstime isrs_time , uint16_t isrs_rcnt);

#if 0
//												 				  NotUsed   PWMOut  PWMIn Capture
io_timer_channel_allocation_t channel_allocations[IOTimerChanModeSize] = { UINT8_MAX,   0  ,  0   ,  0 };
typedef uint8_t io_timer_allocation_t; /* big enough to hold MAX_IO_TIMERS */
#endif
int io_timer_handler0(int irq, void *context);
int io_timer_handler1(int irq, void *context);
int io_timer_handler2(int irq, void *context);
int io_timer_handler3(int irq, void *context);
int io_timer_is_channel_free(unsigned channel);
int io_timer_validate_channel_index(unsigned channel);
int io_timer_get_mode_channels(io_timer_channel_mode_t mode);
int io_timer_get_channel_mode(unsigned channel);
int io_timer_free_channel(unsigned channel);
int io_timer_init_timer(unsigned timer);
int io_timer_set_rate(unsigned timer, unsigned rate);
int io_timer_set_enable(bool state, io_timer_channel_mode_t mode, io_timer_channel_allocation_t masks);
int io_timer_set_ccr(unsigned channel, uint16_t value);
uint16_t io_channel_get_ccr(unsigned channel);
uint32_t io_timer_get_group(unsigned timer);
int io_timer_channel_init(unsigned channel, io_timer_channel_mode_t mode,
			  channel_handler_t channel_handler, void *context);


int io_timer_handler0(int irq, void *context)
{
	return 0;
}

int io_timer_handler1(int irq, void *context)
{
	return 1;

}

int io_timer_handler2(int irq, void *context)
{
	return 2;

}

int io_timer_handler3(int irq, void *context)
{
	return 3;

}
int io_timer_is_channel_free(unsigned channel)
{
	int rv = io_timer_validate_channel_index(channel);
	return rv;
}

int io_timer_validate_channel_index(unsigned channel)
{
	int rv = -EINVAL;

	return rv;
}

int io_timer_get_mode_channels(io_timer_channel_mode_t mode)
{
	return 0;
}

int io_timer_get_channel_mode(unsigned channel)
{
	return -1;
}

int io_timer_free_channel(unsigned channel)
{
	return 0;
}

int io_timer_init_timer(unsigned timer)
{
	return -1;
}


int io_timer_set_rate(unsigned timer, unsigned rate)
{
	return 0;
}

int io_timer_channel_init(unsigned channel, io_timer_channel_mode_t mode,
			  channel_handler_t channel_handler, void *context)
{
    return -1;
}

int io_timer_set_enable(bool state, io_timer_channel_mode_t mode, io_timer_channel_allocation_t masks)
{

	return 0;
}

int io_timer_set_ccr(unsigned channel, uint16_t value)
{
	return 0;
}

uint16_t io_channel_get_ccr(unsigned channel)
{
	uint16_t value = 0;

	return value;
}

uint32_t io_timer_get_group(unsigned timer)
{
        return 0;
}
