/*
 * Targeting.c
 *
 *  Created on: Aug 27, 2020
 *      Author: Daniel
 */

#include "Targeting.h"


enum control_source current_source = 0;



// Request to set joint targets.
// Targets are only promoted to the global joint state if the new source is >= current_source.
// Allows higher priority sources to take control without interference from lower sources.
// Sources must release_control() when they are finished issuing targets.
// Returns TRUE if successful, FALSE if access denied.
BOOL set_targets(enum control_source source, uint8_t ji, fix16_t jpt, fix16_t jvt)
{
	if(request_control(source))
	{
		j[ji].pt = jpt;
		j[ji].vt = jvt;
		return TRUE;
	}
	else
		return FALSE;
}

BOOL set_all_velocity_zero(enum control_source source)
{
	if(request_control(source))
	{
		for(int ji=0; ji<6; ji++)
		{
			j[ji].pt = DISABLE_PT;
			j[ji].vt = F16(0.0f);
		}
		return TRUE;
	}
	else
		return FALSE;
}

BOOL set_all_position_rest(enum control_source source)
{
	if(request_control(source))
	{
		for(int ji=0; ji<6; ji++)
		{
			j[ji].pt = jpmtr[ji].prest;
			j[ji].vt = jpmtr[ji].vmax/2;
		}
		return TRUE;
	}
	else
		return FALSE;
}

BOOL request_control(enum control_source source)
{
	if(source >= current_source)
	{
		current_source = source;
		return TRUE;
	}
	else
		return FALSE;
}


// Release control from the current_source. Control falls back to the lowest priority source (0).
void release_control(enum control_source source)
{
	if(source == current_source)
		current_source = 0;
}


// Returns the current source of control (joint targets)
enum control_source get_control_source()
{
	return current_source;
}


// Called rapidly by background task
void update_targets()
{
	uint32_t task_start_time = SYSTICK_TIMER_HIRES;

	switch(current_source)
	{
		case CTRL_RS485:
		break;

		case CTRL_DIRCTL:
		break;

		case CTRL_BT:
		break;

		case CTRL_HOMING_SEQUENCE:
			update_homing_sequence();
		break;

		case CTRL_NONE:
			for(int ji=0; ji<6; ji++)
			{
				j[ji].pt = DISABLE_PT;
				j[ji].vt = 0;
			}
		break;
	}

	task_targeting_duration_us = (uint16_t)elapsed_time_us_between(task_start_time, SYSTICK_TIMER_HIRES);
}



