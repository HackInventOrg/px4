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

#include "drv_io_timer.h"

//												 				  NotUsed   PWMOut  PWMIn Capture
io_timer_channel_allocation_t channel_allocations[IOTimerChanModeSize] = { UINT8_MAX,   0  ,  0   ,  0 };

typedef uint8_t io_timer_allocation_t; /* big enough to hold MAX_IO_TIMERS */

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
