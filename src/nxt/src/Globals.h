/*
 * Globals.h
 *
 *  Created on: Feb 6, 2020
 *      Author: Daniel
 */

#ifndef SRC_GLOBALS_H_
#define SRC_GLOBALS_H_

//********************************************************************************************************
// NXT COMPILE TARGET: Change to decide which NXT to compile for (1 2 or 3)
//
#define NXT 1
//
//********************************************************************************************************
// COMPILER PARAMETERS
//
#define FIXMATRIX_MAX_SIZE 4
#define FIXMATH_NO_OVERFLOW
#define FIXMATH_NO_ROUNDING
//
//********************************************************************************************************

#include "kernel.h"
#include "kernel_id.h"
#include "ecrobot_interface.h"
#include "stdint.h"

#include "fix16.h"
#include "fixmatrix.h"

// GLOBAL HELPER FUNCTIONS

static inline int32_t min_int(int32_t x, int32_t y)		// returns smaller of x and y
{	return ( (x<y) ? x : y);	}

static inline int32_t max_int(int32_t x, int32_t y)		// returns larger of x and y
{	return ( (x>y) ? x : y);	}

static inline int32_t clamp_int(int32_t n, int32_t lo, int32_t hi)	// If n>hi, returns hi. If n<lo, returns lo. Otherwise returns n unchanged.
{	return ( (n<lo) ? lo : ( (n>hi) ? hi : n) );		}

static inline int32_t inc_wrap(int32_t n, int32_t lo, int32_t hi)	// Increments n, wrapping around to lo if ++n > hi
{	return ( (++n)>hi ? lo : n	);	}

static inline int32_t dec_wrap(int32_t n, int32_t lo, int32_t hi)	// Decrements n, wrapping around to hi if --n < lo
{	return ( (--n)<lo ? hi : n	);	}



// JOINT STATE VARIABLES

#define DISABLE_PT fix16_maximum	// If pt == DISABLE_PT, disable position-level control; only match target velocity.
#define DISABLE_VT fix16_maximum	// If vt == DISABLE_VT, use max allowable velocity (vmax) while traveling to target position.

struct joint_state{
	fix16_t p;			//Joint angular position. Angle about previous z, from old x to new x. Initialized to jpmtr.prest.
	fix16_t v;			//Joint angular velocity.
	fix16_t pt;			//Joint angular position target.
	fix16_t vt;			//Joint angular velocity target.
	int8_t pwm;				//Joint effort from motor regulator (pwm duty cycle, 1-100)
};

extern struct joint_state j[6];

void init_joint_states(void);	// Should be called in ecrobot_device_initialization()

// MECHANICAL/ELECTRICAL PARAMETERS

static fix16_t zero = F16(0.0f);

#if NXT == 1

	#define NUM_CONTROLLERS 2		// Number of joints driven by this NXT
	static const uint8_t joint_list[NUM_CONTROLLERS] = {1,5};	// List of joints driven by this NXT

#elif NXT == 2

	#define NUM_CONTROLLERS 2		// Number of joints driven by this NXT
	static const uint8_t joint_list[NUM_CONTROLLERS] = {0,4};	// List of joints driven by this NXT

#elif NXT == 3

	#define NUM_CONTROLLERS 2		// Number of joints driven by this NXT
	static const uint8_t joint_list[NUM_CONTROLLERS] = {2,3};	// List of joints driven by this NXT

#endif

struct joint_parameter
{
	uint8_t	n;					// Which joint this set of parameters corresponds to (0-5)

	uint8_t	num_motors; 		// Can use up to three motors to drive this joint
	uint32_t ports[3];			// Motor ports to be driven for this joint. Position reading will be taken from encoder on ports[0].
	uint8_t	mirror[3];			// Reverses PWM signal for this motor. Ensure that a positive PWM causes joint angle to increase.
	fix16_t gear;			// Encoder counts per degree of joint angle change. Position reading will be taken from encoder on ports[0]. Use negative value if decreasing encoder results in increasing joint angle.
	fix16_t gear_rec;		// Degrees of joint angle change per encoder count. Reciprocal of gear.
	fix16_t	coaxial[6];		// Previous joint degrees per degree of joint angle change. Only applicable if torque is transmitted coaxially through a previous joint.
	fix16_t coaxial_rec[6];	// Degrees of joint angle change per degree of previous joint change. Reciprocal of coaxial.

	fix16_t b[6];			// Coefficients for two-variable quadratic regression model: pwm_estimate = b0 + b1x1 + b2x2 + b3x1^2 + b4x2^2 + b5x1x2
	fix16_t *x2;			// Pointer to secondary variable to be used in the model. x1 is always jvt_int (intermediate joint velocity target).
	fix16_t kp_p,ki_p,kd_p; // Position controller gains
	fix16_t kp_v,ki_v,kd_v; // Velocity controller gains

	fix16_t d;				// DH Param: offset along previous z to the common normal (mm)
	fix16_t r;				// DH Param: length of the common normal (radius about previous z) (mm)
	fix16_t a;				// DH Param: (alpha) angle about common normal, from old z axis to new z axis (rad)
	fix16_t prest;			// DH Param: (theta) Program assumes the joint starts in this resting position.

	fix16_t pmin;			// Min allowable joint angle (deg)
	fix16_t pmax;			// Max allowable joint angle (deg)
	fix16_t vmax;			// Max allowable joint velocity (deg/s). Calculated from: NXT-L max speed = 1000deg/s, EV3-M max speed = 1500deg/s, PF-XL max speed = 1300deg/s

	fix16_t phome;			// Angle at the center of the joint's homing switch. If phome=pmax or phome=pmin, switch is treated as a limit switch.
	uint8_t tmux_mask;			// Binary mask identifying which homing switch belongs to this joint.
};

static const struct joint_parameter jpmtr[6] = {
	{
		.n = 0,		//Link 1 starts at O0, ends at O1. J1 rotates around z0. J2 rotates around z1.

		.num_motors = 2,
		.ports		= {NXT_PORT_A, NXT_PORT_B},
		.mirror		= {FALSE,		 TRUE},
		.gear		= F16(5.0f),
		.gear_rec	= F16(1.0f/5.0f),
		.coaxial	= {0,0,0,0,0,0},
		.coaxial_rec= {0,0,0,0,0,0},

		.b = {F16(0.0f),	F16(1.0f),	F16(0.0f),	F16(0.0f),	F16(0.0f),	F16(0.0f)},
		.x2 = &zero,	// x1 is this joint's controller's internal velocity target
		.kp_p = F16(0.0f),	.ki_p = F16(0.0f),	.kd_p = F16(0.0f),
		.kp_v = F16(0.0f),	.ki_v = F16(0.0f),	.kd_v = F16(0.0f),

		.d		= F16(176.0f),
		.r		= F16(0.0f),
		.a		= F16(90.0f),
		.prest	= F16(0.0f),
		.pmin	= F16(-45.0f),
		.pmax	= F16(45.0f),
		.vmax	= F16(1000.0f/5.0f),	// 200 deg/s

		.phome	= F16(0.0f),
		.tmux_mask = (0x01 << 0)
	},
	{
		.n = 1,		//Link 2 starts at O1, ends at O2. J2 rotates around z1. J3 rotates around z2.

		.num_motors = 2,
		.ports		= {NXT_PORT_A,	NXT_PORT_B},
		.mirror		= {FALSE,		TRUE},
		.gear		= F16(21.0f),
		.gear_rec	= F16(1.0f/21.0f),
		.coaxial	= {0,0,0,0,0,0},
		.coaxial_rec= {0,0,0,0,0,0},

		.b = {F16(0.0f),	F16(1.0f),	F16(0.0f),	F16(0.0f),	F16(0.0f),	F16(0.0f)},
		.x2 = &zero,	// x1 is this joint's controller's internal velocity target
		.kp_p = F16(0.0f),	.ki_p = F16(0.0f),	.kd_p = F16(0.0f),
		.kp_v = F16(0.0f),	.ki_v = F16(0.0f),	.kd_v = F16(0.0f),

		.d		= F16(0.0f),
		.r		= F16(208.0f),
		.a		= F16(0.0f),
		.prest	= F16(90.0f),
		.pmin	= F16(38.0f),
		.pmax	= F16(142.0f),
		.vmax	= F16(1000.0f/21.0f),	// 47.6 deg/s

		.phome	= F16(142.0f),
		.tmux_mask = (0x01 << 1)
	},
	{
		.n = 2,		//Link 3 starts at O2, ends at O3. J3 rotates around z2. J4 rotates around z3.

		.num_motors = 2,
		.ports 		= {NXT_PORT_A, NXT_PORT_B},
		.mirror	 	= {FALSE,	   TRUE},
		.gear 		= F16(15.0f),
		.gear_rec	= F16(1.0f/15.0f),
		.coaxial	= {0,0,0,0,0,0},
		.coaxial_rec= {0,0,0,0,0,0},

		.b = {F16(0.0f),	F16(1.0f),	F16(0.0f),	F16(0.0f),	F16(0.0f),	F16(0.0f)},
		.x2 = &zero,	// x1 is this joint's controller's internal velocity target
		.kp_p = F16(0.0f),	.ki_p = F16(0.0f),	.kd_p = F16(0.0f),
		.kp_v = F16(0.0f),	.ki_v = F16(0.0f),	.kd_v = F16(0.0f),

		.d		= F16(0.0f),
		.r		= F16(48.0f),
		.a		= F16(90.0f),
		.prest	= F16(0.0f),
		.pmin	= F16(-52.0f),
		.pmax	= F16(52.0f),
		.vmax	= F16(1000.0f/15.0f),	// 66.7 deg/s

		.phome	= F16(0.0f),
		.tmux_mask = (0x01 << 2)
	},
	{
		.n = 3,		//Link 4 starts at O3, ends at O4. J4 rotates around z3. J5 rotates around z4.

		.num_motors = 1,
		.ports		= {NXT_PORT_C},
		.mirror		= {TRUE},
		.gear		= F16(-25.0f/3.0f),
		.gear_rec	= F16(3.0f/-25.0f),
		.coaxial	= {0,0,0,0,0,0},
		.coaxial_rec= {0,0,0,0,0,0},

		.b = {F16(0.0f),	F16(1.0f),	F16(0.0f),	F16(0.0f),	F16(0.0f),	F16(0.0f)},
		.x2 = &zero,	// x1 is this joint's controller's internal velocity target
		.kp_p = F16(0.0f),	.ki_p = F16(0.0f),	.kd_p = F16(0.0f),
		.kp_v = F16(0.0f),	.ki_v = F16(0.0f),	.kd_v = F16(0.0f),

		.d		= F16(184.0f),
		.r		= F16(0.0f),
		.a		= F16(90.0f),
		.prest	= F16(0.0f),
		.pmin	= F16(-110.0f),
		.pmax	= F16(110.0f),
		.vmax	= F16(1500.0f/(25.0f/3.0f)),	// 180 deg/s

		.phome	= F16(0.0f),
		.tmux_mask = (0x01 << 3)
	},
	{
		.n = 4,		//Link 5 starts at O4, ends at O5. J5 rotates around z4. J6 rotates around z5.

		.num_motors = 1,
		.ports		= {NXT_PORT_C},
		.mirror		= {FALSE},
		.gear		= F16(5.0f),
		.gear_rec	= F16(1.0f/5.0f),
		.coaxial	= {0,0,0,0,0,0},
		.coaxial_rec= {0,0,0,0,0,0},

		.b = {F16(0.0f),	F16(1.0f),	F16(0.0f),	F16(0.0f),	F16(0.0f),	F16(0.0f)},
		.x2 = &zero,	// x1 is this joint's controller's internal velocity target
		.kp_p = F16(0.0f),	.ki_p = F16(0.0f),	.kd_p = F16(0.0f),
		.kp_v = F16(0.0f),	.ki_v = F16(0.0f),	.kd_v = F16(0.0f),

		.d		= F16(0.0f),
		.r		= F16(0.0f),
		.a		= F16(-90.0f),
		.prest	= F16(0.0f),
		.pmin	= F16(-125.0f),
		.pmax	= F16(105.0f),
		.vmax	= F16(1000.0f/5.0f),	// 200 deg/s

		.phome	= F16(0.0f),
		.tmux_mask = (0x01 << 4)
	},
	{
		.n = 5,		//Link 6 starts at O5, ends at O6. J6 rotates around z5. End effector rotates around z6

		.num_motors = 1,
		.ports		= {NXT_PORT_C},
		.mirror		= {TRUE},
		.gear		= F16(-7.0f),
		.gear_rec	= F16(1.0f/-7.0f),
		.coaxial	= {0,0,0,F16(-7.0f),F16(-7.0f),0},
		.coaxial_rec= {0,0,0,F16(1.0f/-7.0f),F16(1.0f/-7.0f),0},

		.b = {F16(0.0f),	F16(1.0f),	F16(0.0f),	F16(0.0f),	F16(0.0f),	F16(0.0f)},
		.x2 = &zero,	// x1 is this joint's controller's internal velocity target
		.kp_p = F16(0.0f),	.ki_p = F16(0.0f),	.kd_p = F16(0.0f),
		.kp_v = F16(0.0f),	.ki_v = F16(0.0f),	.kd_v = F16(0.0f),

		.d		= F16(112.0f),
		.r		= F16(0.0f),
		.a		= F16(0.0f),
		.prest	= F16(0.0f),
		.pmin	= F16(-180.0f),
		.pmax	= F16(180.0f),
		.vmax	= F16(1500.0f/7.0f),	// 214.3 deg/s

		.phome	= F16(0.0f),
		.tmux_mask = (0x01 << 5)
	}
};

// JOINT HOMING SWITCHES

extern	uint8_t enable_joint_limits;
extern	uint8_t tmux;
static inline BOOL is_tmux_pressed(uint8_t ji)	{ return (tmux&jpmtr[ji].tmux_mask)==0; }
#if NXT == 1
	#define TMUX_PORT	NXT_PORT_S3		// Touch Multiplexer Hub
	#define TMUX_ADDR	(0x40 >> 1)		// Address gets shifted 1 bit left by I2C subsystem
#endif


// RCX
extern	uint8_t rcx;					// State of RCX pneumatic controller
#if NXT == 2
	#define IRLINK_PORT	NXT_PORT_S3		// Commlink to RCX controller
#endif


// END EFFECTOR

extern	uint8_t ea1;					// End effector sensor values
extern	uint8_t ea2;
extern	uint8_t ea3;
#if NXT == 3
	#define EA1_PORT	NXT_PORT_S1	// EA sensor 1
	#define EA2_PORT	NXT_PORT_S2	// EA sensor 2
	#define EA3_PORT	NXT_PORT_S3	// EA sensor 3
#endif


// OPERATING SYSTEM

extern uint32_t systick_ms;
extern uint16_t task_motorreg_duration_us;
extern uint16_t task_lcd_duration_us;
extern uint16_t task_targeting_duration_us;
extern uint16_t task_sensors_duration_us;
extern uint16_t task_bluetooth_duration_us;
#if NXT == 1
	#define LEFT_BUTTON_MASK	(0x01<<6)		// TMUX mask for left UI button
	#define RIGHT_BUTTON_MASK	(0x01<<7)		// TMUX mask for right UI button
	#define ENTER_BUTTON_PORT	NXT_PORT_S2
#endif



#endif /* SRC_GLOBALS_H_ */
