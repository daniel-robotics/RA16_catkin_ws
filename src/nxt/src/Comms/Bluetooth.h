/*
 * Bluetooth.h
 *
 *  Created on: Feb 6, 2020
 *      Author: Daniel
 */

#ifndef SRC_COMMS_BLUETOOTH_H_
#define SRC_COMMS_BLUETOOTH_H_

#include "kernel.h"
#include "kernel_id.h"
#include "ecrobot_interface.h"
#include "stdint.h"

#include <string.h>
#include "../Globals.h"
#include "../HumanInterface/LCD.h"
#include "../HumanInterface/Sound.h"
#include "../Control/Targeting.h"
#include "../Control/Timing.h"

static const char BT_PIN[] = "1234";

// PUBLIC VARIABLES

extern uint32_t bt_packets_sent;
extern uint32_t bt_packets_received;
extern uint32_t bt_incomplete_sent;
extern uint16_t nxt_bt_tx_interval;	//PC sets to 0 to initiate disconnect

void init_bt(void);				//should be called in device startup hook
void term_bt(void);				//should be called in device shutdown hook
void update_bt(void);			//should be called in a loop.

void disp_bt_state(int starty);
void disp_bt_rx(int starty);	//echos the bluetooth RX buffer to the screen


#endif /* SRC_COMMS_BLUETOOTH_H_ */
