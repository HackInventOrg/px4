/****************************************************************************
 *
 *   Copyright (c) 2015, 2016 Airmind Development Team. All rights reserved.
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
 * 3. Neither the name Airmind nor the names of its contributors may be
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

/**
* @file drv_led_pwm.cpp
*
*
*/

#include <nuttx/config.h>


#include <sys/types.h>
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include <stdbool.h>
#include <fcntl.h>
#include <unistd.h>
#include <stdio.h>
#include <ctype.h>


#include <systemlib/perf_counter.h>
#include <systemlib/err.h>
#include <systemlib/systemlib.h>
#include <systemlib/px4_macros.h>

#include <drivers/drv_pwm_output.h>

#include <board_config.h>

int led_pwm_servo_set(unsigned channel, uint8_t  value);
unsigned led_pwm_servo_get(unsigned channel);
int led_pwm_servo_init(void);
void led_pwm_servo_deinit(void);
void led_pwm_servo_arm(bool armed);
unsigned led_pwm_timer_get_period(unsigned timer);

unsigned
led_pwm_timer_get_period(unsigned timer)
{
	return 0;
}


int
led_pwm_servo_set(unsigned channel, uint8_t  cvalue)
{
	return -1;
}
unsigned
led_pwm_servo_get(unsigned channel)
{
	return 0; 
}
int
led_pwm_servo_init(void)
{
	led_pwm_servo_arm(true);
	return OK;
}

void
led_pwm_servo_deinit(void)
{
	/* disable the timers */
	led_pwm_servo_arm(false);
}
void
led_pwm_servo_arm(bool armed)
{
}

#endif // BOARD_HAS_LED_PWM
