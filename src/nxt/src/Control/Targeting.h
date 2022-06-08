/*
 * Targeting.h
 *
 *	Public interface for Targeting.c.
 *
 *     Version: 1.0
 *  Created on: Aug 27, 2020
 *      Author: Daniel
 */

#ifndef SRC_TARGETING_H_
#define SRC_TARGETING_H_

#include "kernel.h"
#include "kernel_id.h"
#include "ecrobot_interface.h"
#include "stdint.h"

#include "../Globals.h"
#include "../Control/Timing.h"
#include "../Control/Homing.h"
#include "fix16.h"



// Defines which source is allowed to set joint targets, in increasing priority.
enum control_source {
	CTRL_RS485,
	CTRL_DIRCTL,
	CTRL_BT,
	CTRL_HOMING_SEQUENCE,
	CTRL_NONE,
};


// Request to set joint targets.
// Targets are only promoted to the global joint state if the new source is >= current_source.
// Allows higher priority sources to take control without interference from lower sources.
// Sources must release_control() when they are finished issuing targets.
// Returns TRUE if successful, FALSE if access denied.
BOOL set_targets(enum control_source source, uint8_t ji, fix16_t jpt, fix16_t jvt);

// Stop motion (vt=0) for all joints.
BOOL set_all_velocity_zero(enum control_source source);

// Drive all joints back to their rest position (prest)
BOOL set_all_position_rest(enum control_source source);

// Request to take control of joint targets, but do not set any targets right now.
BOOL request_control(enum control_source source);

// Release control from the current_source. Control falls back to the lowest priority source (0).
void release_control(enum control_source source);


// Returns the current source of control (joint targets)
enum control_source get_control_source(void);


// Called rapidly by background task
void update_targets(void);





#endif /* SRC_TARGETING_H_ */
