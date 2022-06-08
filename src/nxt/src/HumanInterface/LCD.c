/*
 * LCD.c
 *
 *  Created on: Feb 6, 2020
 *      Author: Daniel
 */

#include "LCD.h"


// TARGETING

static const enum control_source source = CTRL_DIRCTL;


// PRIVATE VARIABLES

enum button_states {
	RELEASED,
	RISING_EDGE,
	PRESSED,
	FALLING_EDGE
} button[3];

enum button_names {
	LEFT,
	ENTER,
	RIGHT
};

static enum page_names page = PAGE_STARTUP;
#define FIRST_PAGE PAGE_STATUS
#define LAST_PAGE PAGE_TIMING

static int32_t line = 0;
#define LAST_LINE 7


// TASK

TASK(TASK_LCD)
{
	uint32_t task_start_time = SYSTICK_TIMER_HIRES;
	GetResource(RES_LCD);

	#if NXT == 1

		// Process ui button presses
		BOOL button_pressed[3];
		button_pressed[LEFT]  = ((tmux&LEFT_BUTTON_MASK)==0);
		button_pressed[ENTER] = (ecrobot_is_ENTER_button_pressed() || ecrobot_get_touch_sensor(ENTER_BUTTON_PORT));
		button_pressed[RIGHT] = ((tmux&RIGHT_BUTTON_MASK)==0);

		for(int i=0; i<3; i++)
		{
			switch(button[i])
			{
				case RELEASED:
					if(button_pressed[i])
						button[i] = RISING_EDGE;
					break;

				case RISING_EDGE:
					ui_click();
					button[i] = PRESSED;
					break;

				case PRESSED:
					if(!button_pressed[i])
						button[i] = FALLING_EDGE;
					break;

				case FALLING_EDGE:
					button[i] = RELEASED;
					break;
			}
		}

		BOOL page_exiting = (line==0 && (button[LEFT]==RISING_EDGE || button[RIGHT]==RISING_EDGE));


		display_clear(0);

		// Display title bar ("#:      |       ")
		display_goto_xy(0, 0);	display_unsigned(page+1,1);
		display_goto_xy(1, 0);	display_string(":");
		display_goto_xy(8, 0);	display_string("|");
		display_goto_xy(9, 0);	display_unsigned(systick_get_ms(), 7);

		// Display currently selected menu page
		switch(page)
		{
			// Page: Startup (not reachable again after switching page)
			case PAGE_STARTUP:
			{
				if(button[ENTER] == RISING_EDGE)	// left and right buttons are automatically handled
					page = inc_wrap(page, FIRST_PAGE, LAST_PAGE);

				display_goto_xy(2, 0);	display_string("START");
				display_goto_xy(0, 2);	display_string("Start program on");
				display_goto_xy(0, 3);	display_string(" all other NXTs");
				display_goto_xy(0, 4);	display_string(" (right > arrow)");
				display_goto_xy(0, 5);	display_string(" then push any");
				display_goto_xy(0, 6);	display_string(" key on control");
				display_goto_xy(0, 7);	display_string(" box to continue");
				break;
			}

			// Page: Status
			case PAGE_STATUS:
			{
				display_goto_xy(2, 0);	display_string("STATUS");
				disp_bt_state(1);
				display_goto_xy(0, 2);  display_string("tx:");
				display_goto_xy(3, 2);  display_unsigned(bt_packets_sent, 5);
				display_goto_xy(9, 2);  display_string("rx:");
				display_goto_xy(12, 2); display_unsigned(bt_packets_received, 4);
				display_labeled_unsigned("RS485: ",	rs485_update_cycles,	3);
				display_labeled_bin("TMUX:", 6, tmux, 8, 4);
				display_goto_xy(1, 5);	display_int(fix16_to_int(j[0].p),4);
				display_goto_xy(5, 5);	display_string("|");
				display_goto_xy(6, 5);	display_int(fix16_to_int(j[1].p),4);
				display_goto_xy(10,5);	display_string("|");
				display_goto_xy(11,5);	display_int(fix16_to_int(j[2].p),4);
				display_goto_xy(1, 6);	display_int(fix16_to_int(j[3].p),4);
				display_goto_xy(5, 6);	display_string("|");
				display_goto_xy(6, 6);	display_int(fix16_to_int(j[4].p),4);
				display_goto_xy(10,6);	display_string("|");
				display_goto_xy(11,6);	display_int(fix16_to_int(j[5].p),4);

//				display_labeled_unsigned("incomplete:", bt_incomplete_sent, 7);

				static fix16_t angle_increment = F16(3.14159265358979323846264f/20.0f);
				static fix16_t inc = 0;
				fix16_t sintheta = (fix16_mul(angle_increment, inc));
				inc = fix16_add(inc, fix16_one);
				if(inc > F16(40.0f))
					inc = F16(0.0f);
				display_labeled_fix16("Val: ", sintheta, 7);

				break;
			}

			// Page: Direct Joint Control
			case PAGE_DIRCTL:
			{
				if(page_exiting)
				{
					set_all_velocity_zero(source);
					release_control(source);
				}
				else if(button[LEFT] == RISING_EDGE)
				{
					set_targets(source, line-2, DISABLE_PT, F16(-50.0f));
				}
				else if(button[LEFT] == FALLING_EDGE)
				{
					set_targets(source, line-2, DISABLE_PT, F16(0.0f));
				}
				else if(button[RIGHT] == RISING_EDGE)
				{
					set_targets(source, line-2, DISABLE_PT, F16(50.0f));
				}
				else if(button[RIGHT] == FALLING_EDGE)
				{
					set_targets(source, line-2, DISABLE_PT, F16(0.0f));
				}
				else if(button[ENTER] == RISING_EDGE)
				{
					if(line==0) line = 2;
					else		line = inc_wrap(line, 0, LAST_LINE);
				}

				display_goto_xy(2, 0);	display_string("DIRCTL");
				display_goto_xy(0, 1);	display_string("J| PWM| POS| VEL");
				for(int i=0; i<6; i++)
				{
					display_goto_xy(0, i+2);	display_unsigned(i+1,1);
					display_goto_xy(1, i+2);	display_string(":");
					display_goto_xy(2, i+2);	display_int(j[i].pwm,4);
					display_goto_xy(6, i+2);	display_string("|");
					display_goto_xy(7, i+2);	display_int(fix16_to_int(j[i].p),4);
					display_goto_xy(11,i+2);	display_string("|");
					display_goto_xy(12,i+2);	display_int(fix16_to_int(j[i].v),4);
				}
				break;
			}

			// Page:: Homing
			case PAGE_HOMING:
			{
				uint8_t ji = line-2;
				if(page_exiting)
				{
					end_homing_sequence();
					set_all_velocity_zero(source);
					release_control(source);
				}
				else if(button[LEFT] == RISING_EDGE)	// Move toward rest position at half velocity
				{
					set_targets(source, ji, jpmtr[ji].prest, jpmtr[ji].vmax/2);
				}
				else if(button[LEFT] == FALLING_EDGE)	// Stop moving
				{
					set_targets(source, ji, DISABLE_PT, F16(0.0f));
				}
				else if(button[RIGHT] == RISING_EDGE)	// Begin homing sequence for the selected joint
				{
					begin_homing_sequence(ji);
				}
				else if(button[RIGHT] == FALLING_EDGE)
				{

				}
				else if(button[ENTER] == RISING_EDGE)	// Abort homing sequence, advance line selection
				{
					end_homing_sequence();
					if(line==0) line = 2;
					else		line = inc_wrap(line, 0, LAST_LINE);
				}

				display_goto_xy(2, 0);	display_string("HOMING");
				display_goto_xy(0, 1);
				switch(get_homing_sequence_state())
				{
					case HOMING_INACTIVE:
						display_string("<RestPos   Home>");
						break;

					case HOMING_ROLL_OFF_SWITCH_1:
						display_string("ROF1 (ENTR=ABRT)");
						break;

					case HOMING_ROLL_ON_SWITCH:
						display_string("RON  (ENTR=ABRT)");
						break;

					case HOMING_ROLL_OFF_SWITCH_2:
						display_string("ROF2 (ENTR=ABRT)");
						break;
				}
				for(int i=0; i<6; i++)
				{
					display_goto_xy(0, i+2);	display_string("J  Pos:    |Sw: ");
					display_goto_xy(1, i+2);	display_unsigned(i+1,1);
					display_goto_xy(7, i+2);	display_int(fix16_to_int(j[i].p),4);
					if(is_tmux_pressed(i))
					{
						display_goto_xy(15,i+2);	display_string("*");
					}
				}
				break;
			}

			// Page: Timing
			case PAGE_TIMING:
			{
				display_goto_xy(2, 0);	display_string("TIMING");
				display_labeled_unsigned("HiRes T/S:",	ticks_per_second, 1);
				display_labeled_unsigned("MotorReg:",	task_motorreg_duration_us,	3);
				display_labeled_unsigned("LCD:",		task_lcd_duration_us,		4);
				display_labeled_unsigned("Targting:",	task_targeting_duration_us,	5);
				display_labeled_unsigned("Sensors:",	task_sensors_duration_us,	6);
				display_labeled_unsigned("BlueT:",		task_bluetooth_duration_us, 7);
				break;
			}
		}

		// Display line selector
		display_goto_xy(0, line);
		display_string(">");


		// Change page AFTER render to give page a chance to respond to page_exiting
		if(line==0)
		{
			if(button[LEFT] == RISING_EDGE)
				page = dec_wrap(page, FIRST_PAGE, LAST_PAGE);
			else if(button[RIGHT] == RISING_EDGE)
				page = inc_wrap(page, FIRST_PAGE, LAST_PAGE);
		}

		display_update();
	#endif

	ReleaseResource(RES_LCD);
	task_lcd_duration_us = (uint16_t)elapsed_time_us_between(task_start_time, SYSTICK_TIMER_HIRES);
	TerminateTask();
}




// PUBLIC FUNCTION DEFINITIONS

enum page_names get_ui_page()
{
	return page;
}


uint32_t num_characters_signed(int32_t n)
{
	uint32_t neg = 0;
    if (n < 0)
    {
    	neg = 1;	//add 1 character for the negative sign
    	n = (n == -2147483648) ? 2147483647 : -n;	//flip negative n to positive n
    }
    if (n < 10) return 1+neg;
    if (n < 100) return 2+neg;
    if (n < 1000) return 3+neg;
    if (n < 10000) return 4+neg;
    if (n < 100000) return 5+neg;
    if (n < 1000000) return 6+neg;
    if (n < 10000000) return 7+neg;
    if (n < 100000000) return 8+neg;
    if (n < 1000000000) return 9+neg;
    return 10;
}

uint32_t num_characters_unsigned(uint32_t n)
{
    if (n < 10) return 1;
    if (n < 100) return 2;
    if (n < 1000) return 3;
    if (n < 10000) return 4;
    if (n < 100000) return 5;
    if (n < 1000000) return 6;
    if (n < 10000000) return 7;
    if (n < 100000000) return 8;
    if (n < 1000000000) return 9;
    return 10;
}

void display_labeled_int(const char* label, int32_t value, int linenum)	//linenum from 0 to 7
{
	uint32_t places = num_characters_signed(value);
	display_goto_xy(0, linenum);
	display_string(label);
	display_goto_xy(16-places, linenum);
	display_int(value, places);
}

void display_labeled_unsigned(const char* label, uint32_t value, int linenum)	//linenum from 0 to 7
{
	uint32_t places = num_characters_unsigned(value);
	display_goto_xy(0, linenum);
	display_string(label);
	display_goto_xy(16-places, linenum);
	display_unsigned(value, places);
}

void display_labeled_fix16(const char* label, fix16_t value, int linenum)	//linenum from 0 to 7
{
	char str[13];
	uint32_t places = num_characters_signed(fix16_to_int(value))+5;
	fix16_to_str(value, str, 4);
	display_goto_xy(0, linenum);
	display_string(label);
	display_goto_xy(16-places, linenum);
	display_string(str);
}

void display_labeled_bin(const char* label, int len, uint32_t value, int digits, int linenum)	//linenum from 0 to 7, digits from 0 to 16
{
	display_goto_xy(0, linenum);
	display_string(label);
	for(int i=0; i<digits; i++)
	{
		display_goto_xy(len+i, linenum);
		display_unsigned((value>>i)&1, 1);
	}
}


void data_to_hex_str(char* str, uint8_t* data, uint32_t num_bytes)
{
	uint8_t nib;
	uint32_t num_chars = num_bytes<<1;	//multiply by 2; there are 2 hex characters per byte

	for(int i=0; i<num_chars; i++)
	{
		nib = i&1 ? data[i>>1]&0x0F : (data[i>>1]&0xF0)>>4 ;	//if character index is odd, look at lower half of byte. If even, look at upper half of byte.
		if(nib < 0x0A)								//if value is 0-9, simply add the value to the ascii code for '0'
			str[i] = '0' + nib;
		else										//if value is 10-15, add the value to the ascii code for 'A'
			str[i] = 'A' + (nib-10);
	}
}







