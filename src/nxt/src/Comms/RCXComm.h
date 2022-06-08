/*
 * RCXComm.h
 *
 *  Created on: Feb 6, 2020
 *      Author: Daniel
 */

#ifndef SRC_COMMS_RCXCOMM_H_
#define SRC_COMMS_RCXCOMM_H_

#include "kernel.h"
#include "kernel_id.h"
#include "ecrobot_interface.h"
#include "stdint.h"

#include "../Globals.h"

void init_rcx(uint8_t port);
void term_rcx(uint8_t port);

#define STATE_LEFT 0x01
#define STATE_RIGHT 0x02
void set_valve_state(uint8_t port, uint8_t state);

#endif /* SRC_COMMS_RCXCOMM_H_ */
