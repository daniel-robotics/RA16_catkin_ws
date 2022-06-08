/*
 * LCD.h
 *
 *  Created on: Feb 6, 2020
 *      Author: Daniel
 */

#ifndef SRC_HUMANINTERFACE_LCD_H_
#define SRC_HUMANINTERFACE_LCD_H_

#include "kernel.h"
#include "kernel_id.h"
#include "ecrobot_interface.h"
#include "stdint.h"

#include "../Globals.h"
#include "../Control/Timing.h"
#include "../Control/MotorRegulator.h"
#include "../Control/Targeting.h"
#include "../Control/Homing.h"
#include "../Comms/RS485.h"
#include "../Comms/Bluetooth.h"
#include "../HumanInterface/Sound.h"
#include "fix16.h"

DeclareResource(RES_LCD);
DeclareAlarm(ALARM_LCD);
DeclareTask(TASK_LCD);
TASK(TASK_LCD);

enum page_names {
	PAGE_STARTUP,
	PAGE_STATUS,
	PAGE_DIRCTL,
	PAGE_HOMING,
	PAGE_TIMING
};

enum page_names get_ui_page(void);

void display_labeled_int(const char* label, int32_t value, int linenum);		//linenum from 0 to 7
void display_labeled_unsigned(const char* label, uint32_t value, int linenum);	//linenum from 0 to 7
void display_labeled_fix16(const char* label, fix16_t value, int linenum);	//linenum from 0 to 7
void display_labeled_bin(const char* label, int len, uint32_t value, int digits, int linenum);	//linenum from 0 to 7, digits from 0 to 16
void data_to_hex_str(char* str, uint8_t* data, uint32_t num_bytes);
uint32_t num_characters_signed(int32_t n);
uint32_t num_characters_unsigned(uint32_t n);
#endif /* SRC_HUMANINTERFACE_LCD_H_ */
