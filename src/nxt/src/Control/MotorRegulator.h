/*
 * MotorRegulator.h
 *
 *	Public interface for MotorRegulator.
 *
 *     Version: 1.0
 *  Created on: Dec 9, 2018
 *      Author: Daniel
 */

#ifndef SRC_CONTROL_MOTORREGULATOR_H_
#define SRC_CONTROL_MOTORREGULATOR_H_

#include "kernel.h"
#include "kernel_id.h"
#include "ecrobot_interface.h"
#include "stdint.h"

#include "../Globals.h"
#include "../Control/Timing.h"
#include "fix16.h"


#define BRAKEMODE 1
#define NUM_SAMPLES 5
#define POS_ERR_ACC_MAX F16(100.0f)
#define VEL_ERR_ACC_MAX F16(100.0f)

DeclareResource(RES_MOTORS);
DeclareAlarm(ALARM_MOTORREG);
DeclareTask(TASK_MOTORREG);
TASK(TASK_MOTORREG);


// NXT_OSEK Hook Routines
void init_motor_regulator(void);
void term_motor_regulator(void);

extern fix16_t lcdval;

#endif /* SRC_CONTROL_MOTORREGULATOR_H_ */
