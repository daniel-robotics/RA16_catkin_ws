#include "kernel.h"
#include "kernel_id.h"
#include "ecrobot_interface.h"
#include "stdint.h"

#include "Globals.h"
#include "Control/MotorRegulator.h"
#include "Control/Targeting.h"
#include "Control/Timing.h"
#include "Comms/Bluetooth.h"
#include "Comms/RS485.h"
#include "Sensors/Sensors.h"
#include "HumanInterface/LCD.h"
#include "HumanInterface/Sound.h"
#include "fix16.h"


DeclareTask(TASK_BACKGROUND);
DeclareCounter(SysTimerCnt);

void user_1ms_isr_type2(void)
{
	systick_ms = systick_get_ms();
	increment_timer_1s();
	(void)SignalCounter(SysTimerCnt); 	// Increment OSEK Alarm Counter
}

void ecrobot_device_initialize()
{
	init_timing();
	init_joint_states();
	init_motor_regulator();
	init_sensor_ports();
	init_rs485();
	#if NXT == 1
		init_bt();
	#endif
}

void ecrobot_device_terminate() {
	term_motor_regulator();
	term_sensor_ports();
	term_rs485();
	#if NXT == 1
		term_bt();
	#endif
}



TASK(TASK_BACKGROUND)
{
	#if NXT == 1
		while(get_ui_page() == PAGE_STARTUP){}	// Wait until user confirms they have started the program on all other NXTs
	#endif

	while(1)
	{
		update_rs485();

		#if NXT == 1
			update_bt();
		#endif

		update_targets();

	}
}
