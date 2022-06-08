/*
 * Timing.h
 *
 *  Created on: Aug 22, 2020
 *      Author: Daniel
 */

#ifndef SRC_CONTROL_TIMING_H_
#define SRC_CONTROL_TIMING_H_

#include "kernel.h"
#include "kernel_id.h"
#include "ecrobot_interface.h"
#include "stdint.h"

#include "../Globals.h"


#define SYSTICK_TIMER_HIRES (*AT91C_RTTC_RTVR)

static const fix16_t S_PER_MS	= F16(0.001f);
static const fix16_t MS_PER_S	= F16(1000.0f);

void init_timing(void);			//Should be called in device startup hook
void increment_timer_1s(void);	//Should be called every 1ms
void timer_1s(void);			//Executes every 1000ms

uint32_t elapsed_ticks_between(uint32_t earlier_time, uint32_t later_time);	//Ticks elapsed between earlier_time (ms) and later_time (ms).
uint32_t elapsed_time_us_between(uint32_t earlier_time, uint32_t later_time);	//Microseconds elapsed between earlier_time (ticks) and later_time (ticks). Max elapsed time is ~4ms = 4000us.

extern uint32_t systick_seconds;		//number of seconds elapsed since program start
extern uint32_t ticks_per_second;	//number of hires timer ticks per second

#endif /* SRC_CONTROL_TIMING_H_ */
