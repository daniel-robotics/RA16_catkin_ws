/*
 * Homing.c
 *
 *  Created on: Nov 13, 2020
 *      Author: Daniel
 */


#include "Homing.h"


// TARGETING

static const enum control_source source = CTRL_HOMING_SEQUENCE;


// PRIVATE VARIABLES

static enum homing_sequence_state state = HOMING_INACTIVE;
static uint8_t ji = 0;

// PUBLIC FUNCTION DEFINITIONS


// Begins homing sequence for a joint.
// Needs to figure out which direction to move the joint to find its home switch, therefore requires that either:
//	- The joint is already at the home switch (switch is pressed), OR
//	- The home switch is located at one of the end stops (pmax or pmin) (the joint is always on the same side of the switch)
// Returns TRUE if homing sequence begun, FALSE if failed for one of the above reasons.
BOOL begin_homing_sequence(uint8_t joint_index)
{
	if(state != HOMING_INACTIVE)		// Do not begin again if already begun
		return FALSE;

	if( is_tmux_pressed(joint_index)	// Require that either the switch is already pressed, OR
		|| (jpmtr[joint_index].phome == jpmtr[joint_index].pmin)		// the switch is located at one of the end stops.
		|| (jpmtr[joint_index].phome == jpmtr[joint_index].pmax) )
	{
		if(request_control(source) == FALSE)	// Abort if unable to gain control
			return FALSE;
		ji = joint_index;
		enable_joint_limits &= ~(0x01<<joint_index);	// Alert joint controller it's in homing mode
		state = HOMING_ROLL_OFF_SWITCH_1;
		return TRUE;
	}
	else
		return FALSE;
}

void end_homing_sequence()
{
	if(state == HOMING_INACTIVE)		// Do not end again if already stopped
		return;

	enable_joint_limits = 0xFF;
	set_all_position_rest(source);
	release_control(source);
	state = HOMING_INACTIVE;
}

enum homing_sequence_state get_homing_sequence_state(void)
{
	return state;
}

void update_homing_sequence()
{
	BOOL switch_pressed = is_tmux_pressed(ji);
	switch(state)
	{
		case HOMING_INACTIVE:
			break;

		case HOMING_ROLL_OFF_SWITCH_1:
		{
			if(switch_pressed)
			{
				fix16_t jvt = F16(-50.0f);				// Roll off switch in the negative angle direction
				if(jpmtr[ji].phome == jpmtr[ji].pmin)	// (unless switch is at the lower joint limit, then increase angle)
					jvt = -jvt;
				set_targets(source, ji, DISABLE_PT, jvt);
			}
			else
				state=HOMING_ROLL_ON_SWITCH;
			break;
		}

		case HOMING_ROLL_ON_SWITCH:
		{
			if(!switch_pressed)
			{
				fix16_t jvt = F16(35.0f);				// Roll on switch in the positive angle direction
				if(jpmtr[ji].phome == jpmtr[ji].pmin)	// (unless switch is at the lower joint limit, then decrease angle)
					jvt = -jvt;
				set_targets(source, ji, DISABLE_PT, jvt);
			}
			else
				state=HOMING_ROLL_OFF_SWITCH_2;
			break;
		}

		case HOMING_ROLL_OFF_SWITCH_2:
		{
			if(switch_pressed)
			{
				fix16_t jvt = F16(35.0f);				// Pass over the switch in the positive angle direction
				if(jpmtr[ji].phome == jpmtr[ji].pmax)	// (unless switch is at the upper joint limit, then decrease angle)
					jvt = -jvt;
				set_targets(source, ji, DISABLE_PT, jvt);
			}
			else
				end_homing_sequence();
			break;
		}

	}
}
