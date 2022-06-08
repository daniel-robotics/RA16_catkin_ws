 /*
 * PCF8574.h
 *
 *  Driver for PCF8574 I2C IO Expander
 *  Created on: Oct 13, 2020
 *      Author: Daniel
 */

#ifndef SRC_PCF8574_H_
#define SRC_PCF8574_H_

#include "kernel.h"
#include "kernel_id.h"
#include "ecrobot_interface.h"
#include "stdint.h"

#include "i2c.h"


/* init_PCF8574
 * Initializes I2C subsystem and sets device pin states.
 * 	uint8_t port - NXT_PORT_S1 thru NXT_PORT_S4
 * 	uint8_t addr - I2C address, set using pins A0 A1 A2 on the PCF8574.
 * 				A0=0, A1=0, A2=0 (GND) corresponds to address 0x40 (write) and 0x41 (read) according to the datasheet.
 * 				However, the final bit of the address (write=0, read=1) is set automatically by the I2C subsystem.
 * 				Therefore, NXTOSEK expects to be provided only the top 7 bits of the address - in this case 0x20,
 * 				which is internally shifted one bit to the left and the final bit set to R/W as necessary.
 * 	uint8_t mode - 8 bits corresponding to pins P0-P7.
 * 				1 - digital input OR digital output HIGH. For 8 inputs, mode=0xFF
 * 				0 - digital output LOW
 */
void init_PCF8574(uint8_t port, uint8_t addr, uint8_t mode);


/* init_PCF8574
 * Terminates I2C subsystem
 * 	uint8_t port - NXT_PORT_S1 thru NXT_PORT_S4
 */
void term_PCF8574(uint8_t port);


/* read_PCF8574
 * Reads pins P0-P7. I2C transaction completes asynchronously so there is some delay in acquiring a new value.
 * If pin is being used as an output, returns the last-written state (high/low) of that pin.
 * If pin is being used as an input, returns the current state of that pin.
 * 	uint8_t port - NXT_PORT_S1 thru NXT_PORT_S4
 * 	uint8_t addr - I2C address
 * 	uint8_t wait - If true (1), waits until new data is received before returning. If false (0), returns immediately.
 * 	uint8_t return - last received data from device.
 */
uint8_t read_PCF8574(uint8_t port, uint8_t addr, BOOL wait);


/* write_PCF8574
 * Writes pins P0-P7. Function returns immediately, I2C transaction completes asynchronously so there is some delay.
 * If pin is being used as an output, set bit to 1 (high) or 0 (low).
 * If pin is being used as an input, set bit to 1.
 * 	uint8_t port - NXT_PORT_S1 thru NXT_PORT_S4
 * 	uint8_t addr - I2C address
 * 	uint8_t mode - 8 bits corresponding to pins P0-P7.
 * 				1 - digital input OR digital output HIGH. For 8 inputs, mode=0xFF
 * 				0 - digital output LOW
 *  uint8_t wait - If true (1), waits until data is finished being sent before returning. If false (0), returns immediately.
 */
void write_PCF8574(uint8_t port, uint8_t addr, uint8_t mode, BOOL wait);

#endif /* SRC_PCF8574_H_ */


