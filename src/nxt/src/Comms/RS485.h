/*
 * RS485.h
 *
 *  Created on: Feb 6, 2020
 *      Author: Daniel
 */

#ifndef SRC_COMMS_RS485_H_
#define SRC_COMMS_RS485_H_

#include "kernel.h"
#include "kernel_id.h"
#include "ecrobot_interface.h"
#include "stdint.h"

#include <string.h>
#include "../Globals.h"
#include "../HumanInterface/Sound.h"
#include "../Control/Targeting.h"


enum rs485_state {
	RS485_UNINITIALIZED,
	RS485_SENDING_PACKET,
	RS485_WAITING_NXT1,
	RS485_RECEIVING_NXT1,
	RS485_WAITING_NXT2,
	RS485_RECEIVING_NXT2,
	RS485_WAITING_NXT3,
	RS485_RECEIVING_NXT3
};


extern uint32_t rs485_update_cycles;

void init_rs485(void);					//should be called in device startup hook
void term_rs485(void);					//should be called in device shutdown hook
enum rs485_state get_rs485_state(void);
void update_rs485(void);				//should be called in a loop.
void disp_rs485_state(int starty);



//void rs485_startup_procedure(void);		//should be called once at beginning of main task.

#endif /* SRC_COMMS_RS485_H_ */
