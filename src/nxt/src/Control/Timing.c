/*
 * Timing.c
 *
 *  Created on: Aug 22, 2020
 *      Author: Daniel
 */

#include "Timing.h"


// PUBLIC VARIABLES

uint32_t systick_seconds = 0;		//number of seconds elapsed since program start
uint32_t ticks_per_second = 0;		//number of hires timer ticks per second (approx 30669 ticks/sec)


// PRIVATE VARIABLES

static uint32_t timer_1s_val = 0;	//number of ms since last time timer_1s() was called
static uint32_t ticks_last_sec = 0;	//number of hires timer ticks last time timer_1s() was called

// FUNCTION DEFINITIONS

void init_timing()
{														// AT91C_RTTC_RTMR:	(RTTC) Real-time (timer) Mode Register:
	*AT91C_RTTC_RTMR &= ~AT91C_SYSC_RTPRES; 			// 		clear Real-time Timer Prescaler Value
	*AT91C_RTTC_RTMR |= 0x0001;							//		set prescaler to 1 (no division)
	*AT91C_RTTC_RTMR |= AT91C_SYSC_RTTRST;				// 		set Real Time Timer Restart bit
}

void increment_timer_1s(void)	//Should be called every 1ms
{
	timer_1s_val++;
	if(timer_1s_val == 1000)
	{
		timer_1s_val = 0;
		timer_1s();
	}
}

void timer_1s(void)				//Executes every 1000ms
{
	uint32_t now = SYSTICK_TIMER_HIRES;
	ticks_per_second = elapsed_ticks_between(ticks_last_sec, now);
	ticks_last_sec = now;
	systick_seconds++;
}

uint32_t elapsed_ticks_between(uint32_t earlier_time, uint32_t later_time)
{
	if(earlier_time <= later_time)
		return later_time - earlier_time;
	else
		return later_time + (4294967295 - earlier_time);	// Account for timer wrapping around MAX_UINT
}

uint32_t elapsed_time_us_between(uint32_t earlier_time_hires, uint32_t later_time_hires)
{
	return (later_time_hires - earlier_time_hires)<<5;	//  (1E6 us/s) / (30669 ticks/s) ~= 32.6 us/tick (2^5)
}
