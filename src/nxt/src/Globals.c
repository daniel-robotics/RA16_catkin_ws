/*
 * Globals.c
 *
 *  Created on: Feb 6, 2020
 *      Author: Daniel
 */


#include "Globals.h"

// JOINT STATE VARIABLES

struct joint_state j[6];

void init_joint_states()
{
	for(int ji=0; ji<6; ji++)
	{
		j[ji].p = jpmtr[ji].prest;
		j[ji].v = F16(0.0f);
		j[ji].pt = jpmtr[ji].prest;
		j[ji].vt = F16(0.0f);
		j[ji].pwm = F16(0.0f);
	}
}

// JOINT HOMING SWITCHES

uint8_t tmux = 0xFF;
uint8_t enable_joint_limits = 0xFF;

// END EFFECTOR

uint8_t rcx =	0;	//State of RCX pneumatic controller
uint8_t ea1	=	0;	//End-effector sensors
uint8_t ea2	=	0;
uint8_t ea3	=	0;

// OPERATING SYSTEM

uint32_t systick_ms = 0;
uint16_t task_motorreg_duration_us	= 0;
uint16_t task_lcd_duration_us		= 0;
uint16_t task_targeting_duration_us	= 0;
uint16_t task_sensors_duration_us	= 0;
uint16_t task_bluetooth_duration_us	= 0;


