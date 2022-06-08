/*
 * Sensors.h
 *
 *  Created on: Feb 6, 2020
 *      Author: Daniel
 */

#ifndef SRC_SENSORS_SENSORS_H_
#define SRC_SENSORS_SENSORS_H_

#include "kernel.h"
#include "kernel_id.h"
#include "ecrobot_interface.h"
#include "stdint.h"

#include "../Globals.h"
#include "../Control/Timing.h"

#include "EOPD.h"
#include "PCF8574.h"
#include "../Comms/RCXComm.h"

DeclareResource(RES_SENSORS);
DeclareAlarm(ALARM_SENSORS);
DeclareTask(TASK_SENSORS);
TASK(TASK_SENSORS);


void init_sensor_ports(void);
void term_sensor_ports(void);

#endif /* SRC_SENSORS_SENSORS_H_ */
