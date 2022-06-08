/*
 * Bluetooth.c
 *
 *  Created on: Feb 6, 2020
 *      Author: Daniel
 */

#include "Bluetooth.h"



// TARGETING

static const enum control_source source = CTRL_BT;
static struct unpromoted_joint_targets{
	fix16_t pt;
	fix16_t vt;
} jtgt[6];

static void get_targets_from_global_state()
{
	for(int ji=0; ji<6; ji++)
	{
		jtgt[ji].pt = j[ji].pt;
		jtgt[ji].vt = j[ji].vt;
	}
}

static BOOL promote_targets_to_global_state()
{
	BOOL success = TRUE;
	for(int ji=0; ji<6; ji++)
		if(set_targets(source, ji, jtgt[ji].pt, jtgt[ji].vt) == FALSE)
			success = FALSE;
	return success;
}

// PACKET DEFINITIONS

struct pointer_size_pair { uint8_t* val; size_t size;	};


#define NXT1_BT_VAL0	systick_ms	//4
#define NXT1_BT_VAL1	j[0].p
#define NXT1_BT_VAL2	j[0].v
#define NXT1_BT_VAL3	j[0].pwm	//13
#define NXT1_BT_VAL4	j[1].p
#define NXT1_BT_VAL5	j[1].v
#define NXT1_BT_VAL6	j[1].pwm	//22
#define NXT1_BT_VAL7	j[2].p
#define NXT1_BT_VAL8	j[2].v
#define NXT1_BT_VAL9	j[2].pwm	//31
#define NXT1_BT_VAL10	j[3].p
#define NXT1_BT_VAL11	j[3].v
#define NXT1_BT_VAL12	j[3].pwm	//40
#define NXT1_BT_VAL13	j[4].p
#define NXT1_BT_VAL14	j[4].v
#define NXT1_BT_VAL15	j[4].pwm	//49
#define NXT1_BT_VAL16	j[5].p
#define NXT1_BT_VAL17	j[5].v
#define NXT1_BT_VAL18	j[5].pwm	//58
#define NXT1_BT_VAL19	tmux		//59
#define NXT1_BT_VAL20	ea1
#define NXT1_BT_VAL21	ea2
#define NXT1_BT_VAL22	ea3			//62
#define NXT1_BT_VALS  23
#define NXT1_BT_BYTES (sizeof(NXT1_BT_VAL0)+sizeof(NXT1_BT_VAL1)+sizeof(NXT1_BT_VAL2)+sizeof(NXT1_BT_VAL3)+sizeof(NXT1_BT_VAL4)+sizeof(NXT1_BT_VAL5)+sizeof(NXT1_BT_VAL6)+sizeof(NXT1_BT_VAL7)+sizeof(NXT1_BT_VAL8)+sizeof(NXT1_BT_VAL9)+sizeof(NXT1_BT_VAL10)+sizeof(NXT1_BT_VAL11)+sizeof(NXT1_BT_VAL12)+sizeof(NXT1_BT_VAL13)+sizeof(NXT1_BT_VAL14)+sizeof(NXT1_BT_VAL15)+sizeof(NXT1_BT_VAL16)+sizeof(NXT1_BT_VAL17)+sizeof(NXT1_BT_VAL18)+sizeof(NXT1_BT_VAL19)+sizeof(NXT1_BT_VAL20)+sizeof(NXT1_BT_VAL21)+sizeof(NXT1_BT_VAL22))
static const struct pointer_size_pair packet_definition_nxt1[NXT1_BT_VALS] = {
	{ .val=(uint8_t*)&( NXT1_BT_VAL0  ),	.size=sizeof( NXT1_BT_VAL0	)	},
	{ .val=(uint8_t*)&( NXT1_BT_VAL1  ),	.size=sizeof( NXT1_BT_VAL1	)	},
	{ .val=(uint8_t*)&( NXT1_BT_VAL2  ),	.size=sizeof( NXT1_BT_VAL2	)	},
	{ .val=(uint8_t*)&( NXT1_BT_VAL3  ),	.size=sizeof( NXT1_BT_VAL3	)	},
	{ .val=(uint8_t*)&( NXT1_BT_VAL4  ),	.size=sizeof( NXT1_BT_VAL4	)	},
	{ .val=(uint8_t*)&( NXT1_BT_VAL5  ),	.size=sizeof( NXT1_BT_VAL5	)	},
	{ .val=(uint8_t*)&( NXT1_BT_VAL6  ),	.size=sizeof( NXT1_BT_VAL6	)	},
	{ .val=(uint8_t*)&( NXT1_BT_VAL7  ),	.size=sizeof( NXT1_BT_VAL7	)	},
	{ .val=(uint8_t*)&( NXT1_BT_VAL8  ),	.size=sizeof( NXT1_BT_VAL8	)	},
	{ .val=(uint8_t*)&( NXT1_BT_VAL9  ),	.size=sizeof( NXT1_BT_VAL9	)	},
	{ .val=(uint8_t*)&( NXT1_BT_VAL10 ),	.size=sizeof( NXT1_BT_VAL10	)	},
	{ .val=(uint8_t*)&( NXT1_BT_VAL11 ),	.size=sizeof( NXT1_BT_VAL11	)	},
	{ .val=(uint8_t*)&( NXT1_BT_VAL12 ),	.size=sizeof( NXT1_BT_VAL12	)	},
	{ .val=(uint8_t*)&( NXT1_BT_VAL13 ),	.size=sizeof( NXT1_BT_VAL13	)	},
	{ .val=(uint8_t*)&( NXT1_BT_VAL14 ),	.size=sizeof( NXT1_BT_VAL14	)	},
	{ .val=(uint8_t*)&( NXT1_BT_VAL15 ),	.size=sizeof( NXT1_BT_VAL15	)	},
	{ .val=(uint8_t*)&( NXT1_BT_VAL16 ),	.size=sizeof( NXT1_BT_VAL16	)	},
	{ .val=(uint8_t*)&( NXT1_BT_VAL17 ),	.size=sizeof( NXT1_BT_VAL17	)	},
	{ .val=(uint8_t*)&( NXT1_BT_VAL18 ),	.size=sizeof( NXT1_BT_VAL18	)	},
	{ .val=(uint8_t*)&( NXT1_BT_VAL19 ),	.size=sizeof( NXT1_BT_VAL19	)	},
	{ .val=(uint8_t*)&( NXT1_BT_VAL20 ),	.size=sizeof( NXT1_BT_VAL20	)	},
	{ .val=(uint8_t*)&( NXT1_BT_VAL21 ),	.size=sizeof( NXT1_BT_VAL21	)	},
	{ .val=(uint8_t*)&( NXT1_BT_VAL22 ),	.size=sizeof( NXT1_BT_VAL22	)	}
};


#define PC_BT_VAL0		jtgt[0].pt
#define PC_BT_VAL1		jtgt[0].vt			//8
#define PC_BT_VAL2		jtgt[1].pt
#define PC_BT_VAL3		jtgt[1].vt			//16
#define PC_BT_VAL4		jtgt[2].pt
#define PC_BT_VAL5		jtgt[2].vt			//24
#define PC_BT_VAL6		jtgt[3].pt
#define PC_BT_VAL7		jtgt[3].vt			//32
#define PC_BT_VAL8		jtgt[4].pt
#define PC_BT_VAL9		jtgt[4].vt			//40
#define PC_BT_VAL10		jtgt[5].pt
#define PC_BT_VAL11		jtgt[5].vt			//48
#define PC_BT_VAL12		rcx					//49
#define PC_BT_VAL13		nxt_bt_tx_interval		//51
#define PC_BT_VALS    14
#define PC_BT_BYTES (sizeof(PC_BT_VAL0)+sizeof(PC_BT_VAL1)+sizeof(PC_BT_VAL2)+sizeof(PC_BT_VAL3)+sizeof(PC_BT_VAL4)+sizeof(PC_BT_VAL5)+sizeof(PC_BT_VAL6)+sizeof(PC_BT_VAL7)+sizeof(PC_BT_VAL8)+sizeof(PC_BT_VAL9)+sizeof(PC_BT_VAL10)+sizeof(PC_BT_VAL11)+sizeof(PC_BT_VAL12)+sizeof(PC_BT_VAL13))
static const struct pointer_size_pair packet_definition_pc[PC_BT_VALS] = {
	{ .val=(uint8_t*)&( PC_BT_VAL0	),	.size=sizeof( PC_BT_VAL0	)	},
	{ .val=(uint8_t*)&( PC_BT_VAL1	),	.size=sizeof( PC_BT_VAL1	)	},
	{ .val=(uint8_t*)&( PC_BT_VAL2	),	.size=sizeof( PC_BT_VAL2	)	},
	{ .val=(uint8_t*)&( PC_BT_VAL3	),	.size=sizeof( PC_BT_VAL3	)	},
	{ .val=(uint8_t*)&( PC_BT_VAL4	),	.size=sizeof( PC_BT_VAL4	)	},
	{ .val=(uint8_t*)&( PC_BT_VAL5	),	.size=sizeof( PC_BT_VAL5	)	},
	{ .val=(uint8_t*)&( PC_BT_VAL6	),	.size=sizeof( PC_BT_VAL6	)	},
	{ .val=(uint8_t*)&( PC_BT_VAL7	),	.size=sizeof( PC_BT_VAL7	)	},
	{ .val=(uint8_t*)&( PC_BT_VAL8	),	.size=sizeof( PC_BT_VAL8	)	},
	{ .val=(uint8_t*)&( PC_BT_VAL9	),	.size=sizeof( PC_BT_VAL9	)	},
	{ .val=(uint8_t*)&( PC_BT_VAL10	),	.size=sizeof( PC_BT_VAL10	)	},
	{ .val=(uint8_t*)&( PC_BT_VAL11	),	.size=sizeof( PC_BT_VAL11	)	},
	{ .val=(uint8_t*)&( PC_BT_VAL12	),	.size=sizeof( PC_BT_VAL12	)	},
	{ .val=(uint8_t*)&( PC_BT_VAL13	),	.size=sizeof( PC_BT_VAL13	)	}
};


// PUBLIC VARIABLES
uint32_t bt_packets_sent = 0;
uint32_t bt_packets_received = 0;
uint32_t bt_incomplete_sent = 0;
uint16_t nxt_bt_tx_interval = 0;
static const uint16_t BT_DEFAULT_TX_INTERVAL = 100;
static const uint16_t BT_TX_TIMEOUT_DELAY = 5000;

// PRIVATE FUNCTIONS
static void flush_buffer(void);
static uint32_t send_packet(void);
static uint32_t read_packet(void);

static enum bt_state{
	BT_READY,
	BT_STREAMING
} state;

// PRIVATE VARIABLES
static uint32_t bytes_remaining = 0;
static uint8_t packet_pc[PC_BT_BYTES];
static uint8_t packet_nxt1[NXT1_BT_BYTES];
static uint32_t last_send_time = 0;

// FUNCTION DEFINITIONS

void init_bt(void)
{
	ecrobot_init_bt_slave(BT_PIN);
	state = BT_READY;
}

void term_bt(void)
{
	last_send_time = 0;
	release_control(source);
	flush_buffer();
	ecrobot_term_bt_connection();
	state = BT_READY;
}

void disp_bt_state(int starty)
{
	display_goto_xy(0, starty);
	switch(ecrobot_get_bt_status()){
		case BT_NO_INIT:		display_string("BT: NO INIT");		break;
		case BT_INITIALIZED: 	display_string("BT: INITIALIZED");	break;
		case BT_CONNECTED: 		display_string("BT: CONNECTED");	break;
		case BT_STREAM : 		display_string("BT: STREAMING");	break;
	}
}

void disp_bt_rx(int starty)
{
	char byte_str[3]; byte_str[2] = '\0';
	int pos_x = 0, pos_y = starty;

	for(int i=0; i<PC_BT_BYTES; i++)
	{
		if (pos_x >= 16)	//Max number of characters in a line is 16 (enough space for 8 bytes in hex)
		{
			pos_x = 0;
			pos_y++;
		}
		display_goto_xy(pos_x, pos_y);
		data_to_hex_str(byte_str, &packet_pc[i], 1);
		display_string(byte_str);
		pos_x += 2;	//2 hex characters per byte
	}
}

static void flush_buffer(void)
{
	uint8_t temp;
	while(ecrobot_read_bt_packet(&temp, 1) != 0);
	bytes_remaining = 0;
}

static uint32_t send_packet(void)
{
	get_targets_from_global_state();	// Read global targets into local variables, in case any targets are about to be transmitted.

	uint8_t offset = 0;
	for(int i=0; i<NXT1_BT_VALS; i++)
	{
		uint8_t* val = packet_definition_nxt1[i].val;
		size_t size = packet_definition_nxt1[i].size;
		memcpy(packet_nxt1+offset, val, size);
		offset+=size;
	}

	uint32_t bytes_sent = ecrobot_send_bt_packet(packet_nxt1, NXT1_BT_BYTES);	// Returns 0 if can't send a new packet right now (previous packet still transmitting)

	if(bytes_sent == NXT1_BT_BYTES)		// Check if successful.
		bt_packets_sent++;

	return bytes_sent;
}

static uint32_t read_packet()	//Attempts to read and parse the packet specified by header. Returns 0 when completed.
{
	uint32_t bytes_received = ecrobot_read_bt_packet(packet_pc, PC_BT_BYTES);	// blocks until all bytes are available

	if(bytes_received == PC_BT_BYTES)	//If the packet is finished being read, parse accordingly.
	{
		get_targets_from_global_state();	// Read global targets into local variables. Possibly not all local targets will be set via this transmission.

		uint8_t offset = 0;
		for(int i=0; i<PC_BT_VALS; i++)
		{
			uint8_t* val = packet_definition_pc[i].val;
			size_t size = packet_definition_pc[i].size;
			memcpy(val, packet_pc+offset, size);
			offset+=size;
		}

		promote_targets_to_global_state();	// Write updated local targets to global targets (using the target control priority system)
		bt_packets_received++;
	}

	return bytes_received;
}

void update_bt()
{
	uint32_t task_start_time = SYSTICK_TIMER_HIRES;

	ecrobot_init_bt_slave(BT_PIN);

	uint32_t now = systick_get_ms();
	uint32_t bytes_sent = 0;
	uint32_t bytes_received = 0;

	switch(state)
	{
		case BT_READY:
		{
			if(ecrobot_get_bt_status() == BT_STREAM)		// on connect
			{
				state = BT_STREAMING;
				nxt_bt_tx_interval = BT_DEFAULT_TX_INTERVAL;
				bt_packets_sent = 0;
				bt_packets_received = 0;
				request_control(source);
			}
			break;
		}

		case BT_STREAMING:
		{
			if(ecrobot_get_bt_status() != BT_STREAM)		// on random disconnect
			{
				error_buzz();
				term_bt();
				break;
			}

			bytes_received = read_packet();

			if(nxt_bt_tx_interval == 0) 	// 0 means disconnect
			{
				beep();
				term_bt();
				break;
			}

			if((U16)elapsed_ticks_between(last_send_time, now) >= nxt_bt_tx_interval)	// nxt_bt_tx_interval has elapsed since the last (successful) send, so send a new packet
			{
				bytes_sent = send_packet();
				if(bytes_sent == NXT1_BT_BYTES)
					last_send_time = now;
			}

			if((U16)elapsed_ticks_between(last_send_time, now) >= BT_TX_TIMEOUT_DELAY)	// more than BT_TX_TIMEOUT_DELAY has elapsed since the last send, so an error has occurred
			{
				state = BT_READY;
				error_buzz();
				term_bt();
				break;
			}

			break;
		}
	}

	task_bluetooth_duration_us = (uint16_t)elapsed_time_us_between(task_start_time, SYSTICK_TIMER_HIRES);
}



