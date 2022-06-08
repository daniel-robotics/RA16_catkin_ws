/*
 * EOPDArray.h
 *
 *  Created on: Apr 24, 2020
 *      Author: Daniel
 */

#ifndef SRC_EOPDARRAY_H_
#define SRC_EOPDARRAY_H_

#include "kernel.h"
#include "kernel_id.h"
#include "ecrobot_interface.h"
#include "stdint.h"

void init_eopd(uint8_t port);
void term_eopd(uint8_t port);

uint8_t poll_eopd(uint8_t port);


#endif /* SRC_EOPDARRAY_H_ */


