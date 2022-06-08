/*
 * Sound.c
 *
 *  Created on: Feb 6, 2020
 *      Author: Daniel
 */


#include "Sound.h"

void beep(void)
{
	sound_freq(262, 100);
}

void beepbeep(void)
{
	sound_freq(330, 66);
	systick_wait_ms(750);
	sound_freq(330, 66);
	systick_wait_ms(750);
}

void beepbeepbeep(void)
{
	sound_freq(392, 50);
	systick_wait_ms(600);
	sound_freq(392, 50);
	systick_wait_ms(600);
	sound_freq(392, 50);
	systick_wait_ms(600);
}

void highbeep(void)
{
	sound_freq(523, 150);
}

void error_buzz(void)
{
	sound_freq(100, 100);
}

void ui_click(void)
{
	sound_freq(330, 15);
}
