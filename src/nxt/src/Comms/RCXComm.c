/*
 * RCXComm.c
 *
 *  Created on: Feb 6, 2020
 *      Author: Daniel
 */

#include "RCXComm.h"

void init_rcx(uint8_t port)
{
	ecrobot_init_i2c(port, LOWSPEED);
}

void term_rcx(uint8_t port)
{
	ecrobot_term_i2c(port);
}

void set_valve_state(uint8_t port, uint8_t state)
{
	rcx = state;
}
