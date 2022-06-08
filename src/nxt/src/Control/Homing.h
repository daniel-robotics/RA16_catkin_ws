/*
 * Homing.h
 *
 *	Public interface for Homing.c.
 *
 *     Version: 1.0
 *  Created on: Nov 13, 2020
 *      Author: Daniel
 */

#ifndef SRC_CONTROL_HOMING_H_
#define SRC_CONTROL_HOMING_H_

#include "kernel.h"
#include "kernel_id.h"
#include "ecrobot_interface.h"
#include "stdint.h"

#include "../Globals.h"
#include "Targeting.h"
#include "fix16.h"

enum homing_sequence_state{
	HOMING_INACTIVE,
	HOMING_ROLL_OFF_SWITCH_1,
	HOMING_ROLL_ON_SWITCH,
	HOMING_ROLL_OFF_SWITCH_2
};


// Begins homing sequence for a joint.
// Needs to figure out which direction to move the joint to find its home switch, therefore requires that either:
//	- The joint is already at the home switch (switch is pressed), OR
//	- The home switch is located at one of the end stops (pmax or pmin) (the joint is always on the same side of the switch)
// Returns TRUE if homing sequence begun, FALSE if failed for one of the above reasons.
BOOL begin_homing_sequence(uint8_t joint_index);

// Aborts homing sequence and stops the joint
void end_homing_sequence(void);

enum homing_sequence_state get_homing_sequence_state(void);


// Called rapidly by targeting
void update_homing_sequence(void);


#endif /* SRC_CONTROL_HOMING_H_ */
