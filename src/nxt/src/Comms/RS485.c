/*
 * RS485.c
 *
 *  Created on: Feb 6, 2020
 *      Author: Daniel
 */


#include "RS485.h"


// TARGETING

static const enum control_source source = CTRL_RS485;

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


#define NO_PACKET	0x00


#define PACKET_NXT1_HEADER	0x01
#define PACKET_NXT1_VAL0	jtgt[0].pt	// 4
#define PACKET_NXT1_VAL1	jtgt[0].vt	// 4
#define PACKET_NXT1_VAL2	jtgt[2].pt	// 4
#define PACKET_NXT1_VAL3	jtgt[2].vt	// 4
#define PACKET_NXT1_VAL4	jtgt[3].pt	// 4
#define PACKET_NXT1_VAL5	jtgt[3].vt	// 4
#define PACKET_NXT1_VAL6	jtgt[4].pt	// 4
#define PACKET_NXT1_VAL7	jtgt[4].vt	// 4
#define PACKET_NXT1_VAL8	j[1].p		// 4
#define PACKET_NXT1_VAL9	enable_joint_limits		// 1
#define PACKET_NXT1_VAL10	tmux		// 1
#define PACKET_NXT1_VAL11	rcx			// 1
#define PACKET_NXT1_VALS 12
#define PACKET_NXT1_BYTES (sizeof(PACKET_NXT1_VAL0)+sizeof(PACKET_NXT1_VAL1)+sizeof(PACKET_NXT1_VAL2)+sizeof(PACKET_NXT1_VAL3)+sizeof(PACKET_NXT1_VAL4)+sizeof(PACKET_NXT1_VAL5)+sizeof(PACKET_NXT1_VAL6)+sizeof(PACKET_NXT1_VAL7)+sizeof(PACKET_NXT1_VAL8)+sizeof(PACKET_NXT1_VAL9)+sizeof(PACKET_NXT1_VAL10)+sizeof(PACKET_NXT1_VAL11))
static const struct pointer_size_pair packet_definition_nxt1[PACKET_NXT1_VALS] = {
	{ .val=(uint8_t*)&( PACKET_NXT1_VAL0	),	.size=sizeof( PACKET_NXT1_VAL0	)	},
	{ .val=(uint8_t*)&( PACKET_NXT1_VAL1	),	.size=sizeof( PACKET_NXT1_VAL1	)	},
	{ .val=(uint8_t*)&( PACKET_NXT1_VAL2	),	.size=sizeof( PACKET_NXT1_VAL2	)	},
	{ .val=(uint8_t*)&( PACKET_NXT1_VAL3	),	.size=sizeof( PACKET_NXT1_VAL3	)	},
	{ .val=(uint8_t*)&( PACKET_NXT1_VAL4	),	.size=sizeof( PACKET_NXT1_VAL4	)	},
	{ .val=(uint8_t*)&( PACKET_NXT1_VAL5	),	.size=sizeof( PACKET_NXT1_VAL5	)	},
	{ .val=(uint8_t*)&( PACKET_NXT1_VAL6	),	.size=sizeof( PACKET_NXT1_VAL6	)	},
	{ .val=(uint8_t*)&( PACKET_NXT1_VAL7	),	.size=sizeof( PACKET_NXT1_VAL7	)	},
	{ .val=(uint8_t*)&( PACKET_NXT1_VAL8	),	.size=sizeof( PACKET_NXT1_VAL8	)	},
	{ .val=(uint8_t*)&( PACKET_NXT1_VAL9	),	.size=sizeof( PACKET_NXT1_VAL9	)	},
	{ .val=(uint8_t*)&( PACKET_NXT1_VAL10),	.size=sizeof( PACKET_NXT1_VAL10	)	},
	{ .val=(uint8_t*)&( PACKET_NXT1_VAL11),	.size=sizeof( PACKET_NXT1_VAL11	)	}
};


#define PACKET_NXT2_HEADER	0x02
#define PACKET_NXT2_VAL0	j[0].p		// 4
#define PACKET_NXT2_VAL1	j[0].v		// 4
#define PACKET_NXT2_VAL2	j[4].p		// 4
#define PACKET_NXT2_VAL3	j[4].v		// 4
#define PACKET_NXT2_VAL4	j[0].pwm	// 1
#define PACKET_NXT2_VAL5	j[4].pwm	// 1
#define PACKET_NXT2_VALS 6
#define PACKET_NXT2_BYTES (sizeof(PACKET_NXT2_VAL0)+sizeof(PACKET_NXT2_VAL1)+sizeof(PACKET_NXT2_VAL2)+sizeof(PACKET_NXT2_VAL3)+sizeof(PACKET_NXT2_VAL4)+sizeof(PACKET_NXT2_VAL5))
static const struct pointer_size_pair packet_definition_nxt2[PACKET_NXT2_VALS] = {
	{ .val=(uint8_t*)&( PACKET_NXT2_VAL0	),	.size=sizeof( PACKET_NXT2_VAL0	)	},
	{ .val=(uint8_t*)&( PACKET_NXT2_VAL1	),	.size=sizeof( PACKET_NXT2_VAL1	)	},
	{ .val=(uint8_t*)&( PACKET_NXT2_VAL2	),	.size=sizeof( PACKET_NXT2_VAL2	)	},
	{ .val=(uint8_t*)&( PACKET_NXT2_VAL3	),	.size=sizeof( PACKET_NXT2_VAL3	)	},
	{ .val=(uint8_t*)&( PACKET_NXT2_VAL4	),	.size=sizeof( PACKET_NXT2_VAL4	)	},
	{ .val=(uint8_t*)&( PACKET_NXT2_VAL5	),	.size=sizeof( PACKET_NXT2_VAL5	)	}
};


#define PACKET_NXT3_HEADER	0x03
#define PACKET_NXT3_VAL0	j[2].p		// 4
#define PACKET_NXT3_VAL1	j[2].v		// 4
#define PACKET_NXT3_VAL2	j[3].p		// 4
#define PACKET_NXT3_VAL3	j[3].v		// 4
#define PACKET_NXT3_VAL4	j[2].pwm	// 1
#define PACKET_NXT3_VAL5	j[3].pwm	// 1
#define PACKET_NXT3_VAL6	ea1			// 1
#define PACKET_NXT3_VAL7	ea2			// 1
#define PACKET_NXT3_VAL8	ea3			// 1
#define PACKET_NXT3_VALS 9
#define PACKET_NXT3_BYTES (sizeof(PACKET_NXT3_VAL0)+sizeof(PACKET_NXT3_VAL1)+sizeof(PACKET_NXT3_VAL2)+sizeof(PACKET_NXT3_VAL3)+sizeof(PACKET_NXT3_VAL4)+sizeof(PACKET_NXT3_VAL5)+sizeof(PACKET_NXT3_VAL6)+sizeof(PACKET_NXT3_VAL7)+sizeof(PACKET_NXT3_VAL8))
static const struct pointer_size_pair packet_definition_nxt3[PACKET_NXT3_VALS] = {
	{ .val=(uint8_t*)&( PACKET_NXT3_VAL0	),	.size=sizeof( PACKET_NXT3_VAL0	)	},
	{ .val=(uint8_t*)&( PACKET_NXT3_VAL1	),	.size=sizeof( PACKET_NXT3_VAL1	)	},
	{ .val=(uint8_t*)&( PACKET_NXT3_VAL2	),	.size=sizeof( PACKET_NXT3_VAL2	)	},
	{ .val=(uint8_t*)&( PACKET_NXT3_VAL3	),	.size=sizeof( PACKET_NXT3_VAL3	)	},
	{ .val=(uint8_t*)&( PACKET_NXT3_VAL4	),	.size=sizeof( PACKET_NXT3_VAL4	)	},
	{ .val=(uint8_t*)&( PACKET_NXT3_VAL5	),	.size=sizeof( PACKET_NXT3_VAL5	)	},
	{ .val=(uint8_t*)&( PACKET_NXT3_VAL6	),	.size=sizeof( PACKET_NXT3_VAL6	)	},
	{ .val=(uint8_t*)&( PACKET_NXT3_VAL7	),	.size=sizeof( PACKET_NXT3_VAL7	)	},
	{ .val=(uint8_t*)&( PACKET_NXT3_VAL8	),	.size=sizeof( PACKET_NXT3_VAL8	)	}
};


// PUBLIC VARIABLES

uint32_t rs485_update_cycles;


// PRIVATE VARIABLES

static enum rs485_state state;

static uint8_t header = NO_PACKET;
static uint8_t packet_nxt1[PACKET_NXT1_BYTES];
static uint8_t packet_nxt2[PACKET_NXT2_BYTES];
static uint8_t packet_nxt3[PACKET_NXT3_BYTES];


// FUNCTION DEFINITIONS

void init_rs485(void)
{
	state = RS485_UNINITIALIZED;
	ecrobot_init_rs485(DEFAULT_BAUD_RATE_RS485);
}


void term_rs485(void)
{
	state = RS485_UNINITIALIZED;
	ecrobot_term_rs485();
}


enum rs485_state get_rs485_state()
{
	return state;
}


void disp_rs485_state(int starty)
{
	display_goto_xy(0, starty);
	switch(state){
		case RS485_UNINITIALIZED: 	display_string("RS: NO INIT");			break;
		case RS485_SENDING_PACKET: 	display_string("RS: SENDING_PACKET");	break;
		case RS485_WAITING_NXT1: 	display_string("RS: WAITING_NXT1");		break;
		case RS485_RECEIVING_NXT1: 	display_string("RS: RECEIVING_NXT1");	break;
		case RS485_WAITING_NXT2: 	display_string("RS: WAITING_NXT2");		break;
		case RS485_RECEIVING_NXT2: 	display_string("RS: RECEIVING_NXT2");	break;
		case RS485_WAITING_NXT3: 	display_string("RS: WAITING_NXT3");		break;
		case RS485_RECEIVING_NXT3: 	display_string("RS: RECEIVING_NXT3");	break;
	}
}


static void send_header(void)
{
	uint8_t temp;
	#if NXT == 1
		temp = PACKET_NXT1_HEADER;
	#elif NXT == 2
		temp = PACKET_NXT2_HEADER;
	#elif NXT == 3
		temp = PACKET_NXT3_HEADER;
	#endif
	ecrobot_send_rs485(&temp, 0, 1);
}


static uint8_t read_header(void)
{
	if(ecrobot_read_rs485(&header, 0, 1) == 0)			// If no new data has arrived,
		header = NO_PACKET;								//  set header to indicate accordingly.
	return header;
}


static void flush_buffer(void)
{
	header = NO_PACKET;
	uint8_t temp;
	while(ecrobot_read_rs485(&temp, 0, 1) != 0);
}


static void send_packet(void)
{
	get_targets_from_global_state();	// Read global targets into local variables, in case any targets are about to be transmitted.

	uint32_t offset = 0;
	#if NXT == 1
		for(int i=0; i<PACKET_NXT1_VALS; i++)
		{
			uint8_t* val = packet_definition_nxt1[i].val;
			size_t size = packet_definition_nxt1[i].size;
			memcpy(packet_nxt1+offset, val, size);
			offset+=size;
		}
		ecrobot_send_rs485(packet_nxt1, 0, PACKET_NXT1_BYTES);
	#elif NXT == 2
		for(int i=0; i<PACKET_NXT2_VALS; i++)
		{
			uint8_t* val = packet_definition_nxt2[i].val;
			size_t size = packet_definition_nxt2[i].size;
			memcpy(packet_nxt2+offset, val, size);
			offset+=size;
		}
		ecrobot_send_rs485(packet_nxt2, 0, PACKET_NXT2_BYTES);
	#elif NXT == 3
		for(int i=0; i<PACKET_NXT3_VALS; i++)
		{
			uint8_t* val = packet_definition_nxt3[i].val;
			size_t size = packet_definition_nxt3[i].size;
			memcpy(packet_nxt3+offset, val, size);
			offset+=size;
		}
		ecrobot_send_rs485(packet_nxt3, 0, PACKET_NXT3_BYTES);
	#endif
}


static uint32_t read_packet()	//Attempts to read and parse the packet specified by header. Returns 0 when completed.
{
	uint8_t* packet = packet_nxt1;
	uint32_t size = 0;
	static uint32_t bytes_remaining = 0;
	switch(header)				//Assign packet based on last header that was received
	{
		case PACKET_NXT1_HEADER: packet = packet_nxt1;	size = PACKET_NXT1_BYTES; break;
		case PACKET_NXT2_HEADER: packet = packet_nxt2;	size = PACKET_NXT2_BYTES; break;
		case PACKET_NXT3_HEADER: packet = packet_nxt3;	size = PACKET_NXT3_BYTES; break;
	}

	if(bytes_remaining == 0)	//If function starts out with bytes_remaining == 0, this is the start of a new packet.
		bytes_remaining = size;

	bytes_remaining -= ecrobot_read_rs485(packet, size-bytes_remaining, bytes_remaining);

	if(bytes_remaining == 0)	//If bytes_remaining == 0 at this point, the packet is finished being read, so parse accordingly.
	{
		get_targets_from_global_state();	// Read global targets into local variables. Possibly not all local targets will be set via this transmission.

		uint8_t offset = 0;
		switch(header)
		{
			case PACKET_NXT1_HEADER:
				for(int i=0; i<PACKET_NXT1_VALS; i++)
				{
					uint8_t* val = packet_definition_nxt1[i].val;
					size_t size = packet_definition_nxt1[i].size;
					memcpy(val, packet_nxt1+offset, size);
					offset+=size;
				}
				break;
			case PACKET_NXT2_HEADER:
				for(int i=0; i<PACKET_NXT2_VALS; i++)
				{
					uint8_t* val = packet_definition_nxt2[i].val;
					size_t size = packet_definition_nxt2[i].size;
					memcpy(val, packet_nxt2+offset, size);
					offset+=size;
				}
				break;
			case PACKET_NXT3_HEADER:
				for(int i=0; i<PACKET_NXT3_VALS; i++)
				{
					uint8_t* val = packet_definition_nxt3[i].val;
					size_t size = packet_definition_nxt3[i].size;
					memcpy(val, packet_nxt3+offset, size);
					offset+=size;
				}
				break;
		}

		promote_targets_to_global_state();	// Write updated local targets to global targets (using the target control priority system)
	}

	return bytes_remaining;
}


void update_rs485()
{
	#if NXT == 1

		switch(state)
		{
			case RS485_UNINITIALIZED:			//STATE: RS485_UNINITIALIZED
				flush_buffer();
				beep();
				state = RS485_SENDING_PACKET;			//Start of the update cycle. Choose in what state the state machine begins.
				break;

			case RS485_SENDING_PACKET:			//STATE: RS485_SENDING_PACKET
				send_header();						//Send a 1-byte packet header.
				send_packet();						//Send this NXT's data packet.
				state = RS485_WAITING_NXT2;			//Transition to state: RS485_WAITING_NXT2
				break;

			case RS485_WAITING_NXT2:			//STATE: RS485_WAITING_NXT2
				if(read_header()==PACKET_NXT2_HEADER) //Attempt to read a 1-byte packet header. If correct response is received,
					state = RS485_RECEIVING_NXT2;		//Transition to state: RS485_RECEIVING_NXT2
				break;

			case RS485_RECEIVING_NXT2:			//STATE: RS485_RECEIVING_NXT2
				if(read_packet() == 0)				//Attempt to read packet. Check if full packet has been read (bytes_remaining == 0)
					state = RS485_WAITING_NXT3;			//Transition to state: RS485_WAITING_NXT3
				break;

			case RS485_WAITING_NXT3:			//STATE: RS485_WAITING_NXT3
				if(read_header()==PACKET_NXT3_HEADER) //Attempt to read a 1-byte packet header. If correct response is received,
					state = RS485_RECEIVING_NXT3;		//Transition to state: RS485_RECEIVING_NXT3
				break;

			case RS485_RECEIVING_NXT3:			//STATE: RS485_RECEIVING_NXT3
				if(read_packet() == 0)				//Attempt to read packet. Check if full packet has been read (bytes_remaining == 0)
				{
					state = RS485_SENDING_PACKET;		//Transition to state: RS485_SENDING_PACKET
					rs485_update_cycles++;
				}
				break;

			default:							//STATE: DEFAULT
				error_buzz();						//An error in the state machine has occurred.
				systick_wait_ms(1000);
				break;
		}

	#elif NXT == 2

		switch(state)
		{
			case RS485_UNINITIALIZED:			//STATE: RS485_UNINITIALIZED
				flush_buffer();
				beep();
				state = RS485_WAITING_NXT1;			//Start of the update cycle. Choose in what state the state machine begins.
				break;

			case RS485_WAITING_NXT1:			//STATE: RS485_WAITING_NXT1
				if(read_header()==PACKET_NXT1_HEADER)		//Attempt to read a 1-byte packet header. If correct response is received,
					state = RS485_RECEIVING_NXT1;		//Transition to state: RS485_RECEIVING_NXT1
				break;

			case RS485_RECEIVING_NXT1:			//STATE: RS485_RECEIVING_NXT1
				if(read_packet() == 0)				//Attempt to read packet. Check if full packet has been read (bytes_remaining == 0)
					state = RS485_SENDING_PACKET;		//Transition to state: RS485_SENDING_PACKET
				break;

			case RS485_SENDING_PACKET:			//STATE: RS485_SENDING_PACKET
				send_header();						//Send a 1-byte packet header.
				send_packet();						//Send this NXT's data packet.
				state = RS485_WAITING_NXT3;			//Transition to state: RS485_WAITING_NXT3
				break;

			case RS485_WAITING_NXT3:			//STATE: RS485_WAITING_NXT3
				if(read_header()==PACKET_NXT3_HEADER) //Attempt to read a 1-byte packet header. If correct response is received,
					state = RS485_RECEIVING_NXT3;		//Transition to state: RS485_RECEIVING_NXT3
				break;

			case RS485_RECEIVING_NXT3:			//STATE: RS485_RECEIVING_NXT3
				if(read_packet() == 0)				//Attempt to read packet. Check if full packet has been read (bytes_remaining == 0)
				{
					state = RS485_WAITING_NXT1;			//Transition to state: RS485_WAITING_NXT1
					rs485_update_cycles++;
				}
				break;

			default:							//STATE: DEFAULT
				error_buzz();						//An error in the state machine has occurred.
				systick_wait_ms(1000);
				break;
		}


	#elif NXT == 3

		switch(state)
		{
			case RS485_UNINITIALIZED:			//STATE: RS485_UNINITIALIZED
				flush_buffer();
				beep();
				state = RS485_WAITING_NXT1;			//Start of the update cycle. Choose in what state the state machine begins.
				break;

			case RS485_WAITING_NXT1:			//STATE: RS485_WAITING_NXT1
				if(read_header()==PACKET_NXT1_HEADER)		//Attempt to read a 1-byte packet header. If correct response is received,
					state = RS485_RECEIVING_NXT1;		//Transition to state: RS485_RECEIVING_NXT1
				break;

			case RS485_RECEIVING_NXT1:			//STATE: RS485_RECEIVING_NXT1
				if(read_packet() == 0)				//Attempt to read packet. Check if full packet has been read (bytes_remaining == 0)
					state = RS485_WAITING_NXT2;			//Transition to state: RS485_WAITING_NXT2
				break;

			case RS485_WAITING_NXT2:			//STATE: RS485_WAITING_NXT2
				if(read_header()==PACKET_NXT2_HEADER) //Attempt to read a 1-byte packet header. If correct response is received,
					state = RS485_RECEIVING_NXT2;		//Transition to state: RS485_RECEIVING_NXT2
				break;

			case RS485_RECEIVING_NXT2:			//STATE: RS485_RECEIVING_NXT2
				if(read_packet() == 0)				//Attempt to read packet. Check if full packet has been read (bytes_remaining == 0)
					state = RS485_SENDING_PACKET;		//Transition to state: RS485_SENDING_PACKET
				break;

			case RS485_SENDING_PACKET:			//STATE: RS485_SENDING_PACKET
				send_header();						//Send a 1-byte packet header.
				send_packet();						//Send this NXT's data packet.
				state = RS485_WAITING_NXT1;			//Transition to state: RS485_WAITING_NXT1
				rs485_update_cycles++;
				break;

			default:							//STATE: DEFAULT
				error_buzz();						//An error in the state machine has occurred.
				systick_wait_ms(1000);
				break;

		}


	#endif
}








//void rs485_startup_procedure(void)
//{
//	//***************************************************************************************************
//	//  COMM SYNCHRONIZATION ROUTINE
//	//	1. Start program on NXT1. NXT1 should sound a "beep" when the program starts.
//	//	2. Start program on NXT2. NXT1 should sound a "beepbeep" when the program starts on NXT2.
//	//  3. Start program on NXT3. NXT1 should sound a "beepbeepbeep" when the program starts on NXT3.
//	//  4. NXT1 sends the final packet. NXT2 and NXT3 should both sound a "beep".
//	//***************************************************************************************************
//	flush_buffer();
//	beep();
//	#if NXT == 1
//
//		state = RS485_WAITING_NXT2;
//		while(state != RS485_INITIALIZED)
//		{
//			switch(state)
//			{
//			case RS485_WAITING_NXT2:				//STATE: RS485_WAITING_NXT2
//				read_header();					//Attempt to read a 1-byte packet header.
//				switch(header)
//				{
//				case NO_PACKET: break;			//Received no response (yet). Continue waiting.
//				case NXT2_HEADER:				//Received correct response. Transition to state: RS485_WAITING_NXT3
//					beepbeep();
//					flush_buffer();
//					state = RS485_WAITING_NXT3;
//					break;
//				default:						//A breach of protocol has occurred. Attempt to recover.
//					error_buzz();
//					flush_buffer();
//					state = RS485_WAITING_NXT2;
//					break;
//				}
//				break;
//
//			case RS485_WAITING_NXT3:				//STATE: RS485_WAITING_NXT3
//				read_header();					//Attempt to read a 1-byte packet header.
//				switch(header)
//				{
//				case NO_PACKET: break;			//Received no response (yet). Continue waiting.
//				case NXT3_HEADER:				//Received correct response. Transition to state: RS485_SENDING_PACKET
//					beepbeepbeep();
//					flush_buffer();
//					state = RS485_SENDING_PACKET;
//					break;
//				default:						//A breach of protocol has occurred. Attempt to recover.
//					error_buzz();
//					flush_buffer();
//					state = RS485_WAITING_NXT2;
//					break;
//				}
//				break;
//
//			case RS485_SENDING_PACKET:			//STATE: RS485_SENDING_PACKET
//				send_header();					//Send a 1-byte packet header.
//				systick_wait_ms(2000);			//Wait for NXT2 and NXT3 to finish up before continuing.
//				state = RS485_INITIALIZED;			//Transition to state: RS485_INITIALIZED
//				break;
//
//			default:						//STATE: DEFAULT
//				error_buzz();					//An error in the state machine has occurred.
//				systick_wait_ms(1000);
//				break;
//
//			}
//
//		}
//
//	#elif NXT == 2
//		state = RS485_SENDING_PACKET;
//		while(state != RS485_INITIALIZED)
//		{
//			switch(state)
//			{
//			case RS485_SENDING_PACKET:			//STATE: RS485_SENDING_PACKET
//				send_header();					//Send a 1-byte packet header.
//				state = RS485_WAITING_NXT3;			//Transition to state: WAITING_NXT_3
//				break;
//
//			case RS485_WAITING_NXT3:				//STATE: RS485_WAITING_NXT3
//				read_header();					//Attempt to read a 1-byte packet header.
//				switch(header)
//				{
//				case NO_PACKET: break;			//Received no response (yet). Continue waiting.
//				case NXT3_HEADER:				//Received correct response. Transition to state: RS485_WAITING_NXT1
//					flush_buffer();
//					state = RS485_WAITING_NXT1;
//					break;
//				default:						//A breach of protocol has occurred. Attempt to recover.
//					error_buzz();
//					flush_buffer();
//					state = RS485_SENDING_PACKET;
//					break;
//				}
//				break;
//
//			case RS485_WAITING_NXT1:				//STATE: RS485_WAITING_NXT1
//				read_header();					//Attempt to read a 1-byte packet header.
//				switch(header)
//				{
//				case NO_PACKET: break;			//Received no response (yet). Continue waiting.
//				case PACKET_NXT1_HEADER:				//Received correct response. Transition to state: RS485_INITIALIZED
//					beep();
//					flush_buffer();
//					state = RS485_INITIALIZED;
//					break;
//				default:						//A breach of protocol has occurred. Attempt to recover.
//					error_buzz();
//					flush_buffer();
//					state = RS485_SENDING_PACKET;
//					break;
//				}
//				break;
//
//			default:						//STATE: DEFAULT
//				error_buzz();					//An error in the state machine has occurred.
//				systick_wait_ms(1000);
//				break;
//
//			}
//		}
//
//	#elif NXT == 3
//		state = RS485_SENDING_PACKET;
//		while(state != RS485_INITIALIZED)
//		{
//			switch(state)
//			{
//			case RS485_SENDING_PACKET:			//STATE: RS485_SENDING_PACKET
//				send_header();					//Send a 1-byte packet header.
//				state = RS485_WAITING_NXT1;			//Transition to state: WAITING_NXT_1
//				break;
//
//			case RS485_WAITING_NXT1:				//STATE: RS485_WAITING_NXT1
//				read_header();					//Attempt to read a 1-byte packet header.
//				switch(header)
//				{
//				case NO_PACKET: break;			//Received no response (yet). Continue waiting.
//				case PACKET_NXT1_HEADER:				//Received correct response. Transition to state: RS485_INITIALIZED
//					highbeep();
//					flush_buffer();
//					state = RS485_INITIALIZED;
//					break;
//				default:						//A breach of protocol has occurred. Attempt to recover.
//					error_buzz();
//					flush_buffer();
//					state = RS485_SENDING_PACKET;
//					break;
//				}
//				break;
//
//			default:						//STATE: DEFAULT
//				error_buzz();					//An error in the state machine has occurred.
//				systick_wait_ms(1000);
//				break;
//
//			}
//		}
//
//	#endif
//}
