/*
 * Sound.h
 *
 *  Created on: Feb 6, 2020
 *      Author: Daniel
 */

#ifndef SRC_HUMANINTERFACE_SOUND_H_
#define SRC_HUMANINTERFACE_SOUND_H_

#include "kernel.h"
#include "kernel_id.h"
#include "ecrobot_interface.h"
#include "stdint.h"

#include "../Globals.h"

void beep(void);
void beepbeep(void);
void beepbeepbeep(void);
void highbeep(void);
void error_buzz(void);
void ui_click(void);

#endif /* SRC_HUMANINTERFACE_SOUND_H_ */
