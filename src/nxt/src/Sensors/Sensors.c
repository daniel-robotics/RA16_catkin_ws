/*
 * Sensors.c
 *
 *  Created on: Feb 6, 2020
 *      Author: Daniel
 */

#include "Sensors.h"

void init_sensor_ports(void)
{
	#if NXT == 1
		init_PCF8574(TMUX_PORT, TMUX_ADDR, 0xFF); 		// switch multiplexer hub

	#elif NXT == 2
		init_rcx(IRLINK_PORT);

	#elif NXT == 3
		init_eopd(EA1_PORT);
		init_eopd(EA2_PORT);
		init_eopd(EA3_PORT);
	#endif
}

void term_sensor_ports(void)
{
	#if NXT == 1
		term_PCF8574(TMUX_PORT); 		// limit switch hub

	#elif NXT == 2
		term_rcx(IRLINK_PORT);

	#elif NXT == 3
		term_eopd(EA1_PORT);
		term_eopd(EA2_PORT);
		term_eopd(EA3_PORT);
	#endif
}

TASK(TASK_SENSORS)
{
	uint32_t task_start_time = SYSTICK_TIMER_HIRES;
	GetResource(RES_SENSORS);

	#if NXT == 1
		tmux = read_PCF8574(TMUX_PORT, TMUX_ADDR, FALSE);

	#elif NXT == 2


	#elif NXT == 3
		ea1 = poll_eopd(EA1_PORT);
		ea2 = poll_eopd(EA2_PORT);
		ea3 = poll_eopd(EA3_PORT);
	#endif

	ReleaseResource(RES_SENSORS);
	task_sensors_duration_us = (uint16_t)elapsed_time_us_between(task_start_time, SYSTICK_TIMER_HIRES);
	TerminateTask();
}






