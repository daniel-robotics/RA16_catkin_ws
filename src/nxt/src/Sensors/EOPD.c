/*
 * EOPDArray.c
 *
 *  Created on: Apr 24, 2020
 *      Author: Daniel
 */

#include "EOPD.h"

void init_eopd(uint8_t port)
{
	ecrobot_set_light_sensor_active(port);
}

void term_eopd(uint8_t port)
{
	ecrobot_set_light_sensor_inactive(port);
}

uint8_t poll_eopd(uint8_t port)
{
	return (uint8_t)(ecrobot_get_light_sensor(port)/4);
}



