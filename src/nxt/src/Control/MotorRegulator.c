#include "MotorRegulator.h"



// JOINT INDEX:		 ji is joint index (0 to 5). ji = joint_list[ci]. Each joint has 1 or more motors driving it.
// CONTROLLER INDEX: ci is controller index (0 to (NUM_CONTROLLERS-1)). Each joint has 1 controller (though the controller may exist on another NXT)

static void set_enc(uint8_t ji, int32_t count);
static void apply_pwm(uint8_t ji, int32_t pwm);
static int32_t sample_enc_cnt(uint8_t ci);
static fix16_t estimate_enc_vel(uint8_t ci);		// returns velocity @ motor encoder, counts per second

static void update_home_sw(uint8_t ci);					// Updates state of jctrl[ci].home_sw
static inline BOOL is_joint_limit_enabled(uint8_t ji)
{	return ((enable_joint_limits>>ji)&0x01 );	}

static fix16_t enc_cnt_to_jp(uint8_t ji, int32_t count);		// convert encoder count to joint position
static fix16_t enc_vel_to_jv(uint8_t ji, fix16_t enc_v);	// convert encoder velocity (counts per second) to joint velocity
static int32_t     enc_cnt_from_jp(uint8_t ji, fix16_t jp);	// convert joint position to encoder count

static fix16_t pos_ctrl(uint8_t ci, fix16_t jp, fix16_t jpt, fix16_t jv_max);	// input position target, output velocity response
static fix16_t vel_ctrl(uint8_t ci, fix16_t jv, fix16_t jvt);					// input velocity target, output pwm response

enum homing_switch_states {
	RELEASED,
	PRESSED,
};

static struct joint_controller{
	uint8_t recent_sample;				// Index of the most recent encoder sample
	int32_t enc_cnt[NUM_SAMPLES];		// Stores last NUM_SAMPLES encoder counts
	uint32_t enc_cnt_ms[NUM_SAMPLES];	// Stores last NUM_SAMPLES timestamps (milliseconds)
	fix16_t dt;						// Elapsed time (seconds) between the most recent sample and the previous sample

	fix16_t pos_err;				// Position error
	fix16_t pos_err_acc;			// Sum of all previous error values (accumulator for integral term)

	fix16_t vel_err;				// Velocity error
	fix16_t vel_err_acc;			// Sum of all previous error values (accumulator for integral term)

	enum homing_switch_states home_sw;	// State of this joint's homing switch. Stays at RISING_EDGE or FALLING_EDGE for 1 cycle each.
	int32_t enc_cnt_rising_edge;		// Encoder count at the homing switch's rising edge
	int32_t enc_cnt_falling_edge;		// Encoder count at the homing switch's falling edge

} jctrl[NUM_CONTROLLERS];



//PUBLIC FUNCTIONS:

void init_motor_regulator()
{
	for(int ci=0; ci<NUM_CONTROLLERS; ci++)
	{
		uint8_t ji = joint_list[ci];	// get index of joint to control (0-5)
		apply_pwm(ji, 0);
		set_enc(ji, 0);
	}
}

void term_motor_regulator()
{

}

fix16_t lcdval = 0;
TASK(TASK_MOTORREG)
{
	uint32_t task_start_time = SYSTICK_TIMER_HIRES;
	GetResource(RES_MOTORS);

	for(int ci=0; ci<NUM_CONTROLLERS; ci++)	// loop through joint_list to repeat for all joints this controller is responsible for
	{
		uint8_t ji = joint_list[ci];		// Index of current joint (0-5)

		// Measure joint position and velocity
		int32_t enc_cnt = sample_enc_cnt(ci);
		fix16_t enc_vel = estimate_enc_vel(ci);

		fix16_t jp = enc_cnt_to_jp(ji, enc_cnt);	j[ji].p = jp;
		fix16_t jv = enc_vel_to_jv(ji, enc_vel);	j[ji].v = jv;

		// Update state of homing switch. Recalculate joint position based on rising/falling edge if in homing mode.
		update_home_sw(ci);

		// Position-level and velocity-level PID controllers
		fix16_t jpt = j[ji].pt;							// Position target
		fix16_t jvt = pos_ctrl(ci, jp, jpt, j[ji].vt);	// Velocity target set by position controller
		fix16_t pwm = vel_ctrl(ci, jv, jvt);			// Power level set by velocity controller

		// Apply calculated power level to this joint's motors.
		apply_pwm(ji, fix16_to_int(pwm));
	}

	ReleaseResource(RES_MOTORS);
	task_motorreg_duration_us = (uint16_t)elapsed_time_us_between(task_start_time, SYSTICK_TIMER_HIRES);
	TerminateTask();
}



//PRIVATE FUNCTIONS:

static fix16_t pos_ctrl(uint8_t ci, fix16_t jp, fix16_t jpt, fix16_t jv_max)		// input position target, output velocity response
{
	if(jpt == DISABLE_PT)		// If position tracking is disabled, bypass all this and set velocity target to jv_max
		return jv_max;

	uint8_t ji = joint_list[ci];		// Index of current joint (0-5)

	jpt = fix16_clamp(jpt, jpmtr[ji].pmin, jpmtr[ji].pmax);		// Clamp to safe values

	// Calculate error between current position and target position
	fix16_t error = fix16_sub(jpt, jp);						//current error = target - current
	jctrl[ci].pos_err_acc = fix16_clamp(fix16_add(jctrl[ci].pos_err_acc, error), -POS_ERR_ACC_MAX, POS_ERR_ACC_MAX);	//accumulate error, clamp to max absolute value
	fix16_t de = fix16_sub(error, jctrl[ci].pos_err);		//change in error since last run
	jctrl[ci].pos_err = error;

	// PID controller to reduce position error: vt = kp_p*error + ki_p*err_acc*dt + kd_p*de/dt
	fix16_t proportional = fix16_mul(jpmtr[ji].kp_p, error);
	fix16_t integral	 = fix16_mul(jpmtr[ji].ki_p, fix16_mul(jctrl[ci].pos_err_acc, jctrl[ci].dt));
	fix16_t derivative	 = fix16_div(fix16_mul(jpmtr[ji].kd_p, de), jctrl[ci].dt);
	fix16_t vt = fix16_add(proportional, fix16_add(integral, derivative));
	lcdval = proportional;
	jv_max = fix16_abs(jv_max);				// Do not target speeds over jv_max
	vt = fix16_clamp(vt, -jv_max, jv_max);

	return vt;
}

static fix16_t vel_ctrl(uint8_t ci, fix16_t jv, fix16_t jvt)			// input velocity target, output pwm response
{
	uint8_t ji = joint_list[ci];										// Index of current joint (0-5)

	jvt = fix16_clamp(jvt, -jpmtr[ji].vmax, jpmtr[ji].vmax);	// Clamp to safe values

	// Estimate pwm using two-variable quadratic regression model: pwm_base = b0 + b1x1 + b2x2 + b3x1^2 + b4x2^2 + b5x1x2
	//  - x1 is always jvt (joint velocity target).
	//	- x2 is an arbitrary second variable, specified in pmtr.
	fix16_t x1 = jvt, x2 = *(jpmtr[ji].x2);
	fix16_t pwm_base = jpmtr[ji].b[0];
	pwm_base = fix16_add(pwm_base, fix16_mul(jpmtr[ji].b[1], x1));
	pwm_base = fix16_add(pwm_base, fix16_mul(jpmtr[ji].b[2], x2));
	pwm_base = fix16_add(pwm_base, fix16_mul(jpmtr[ji].b[3], fix16_mul(x1, x1)));
	pwm_base = fix16_add(pwm_base, fix16_mul(jpmtr[ji].b[4], fix16_mul(x2, x2)));
	pwm_base = fix16_add(pwm_base, fix16_mul(jpmtr[ji].b[5], fix16_mul(x1, x2)));

	// Calculate error between current position and target position
	fix16_t error = fix16_sub(jvt, jv);						//current error = target - current
	jctrl[ci].vel_err_acc = fix16_clamp(fix16_add(jctrl[ci].vel_err_acc, error), -VEL_ERR_ACC_MAX, VEL_ERR_ACC_MAX);	//accumulate error, clamp to max absolute value
	fix16_t de = fix16_sub(error, jctrl[ci].vel_err);		//change in error since last run
	jctrl[ci].vel_err = error;

	// PID controller to reduce velocity error: pwm = pwm_base + kp_v*error + ki_v*vel_err_acc*dt + kd_v*de/dt
	fix16_t proportional = fix16_mul(jpmtr[ji].kp_v, error);
	fix16_t integral	 = fix16_mul(jpmtr[ji].ki_v, fix16_mul(jctrl[ci].vel_err_acc, jctrl[ci].dt));
	fix16_t derivative	 = fix16_div(fix16_mul(jpmtr[ji].kd_v, de), jctrl[ci].dt);
	fix16_t pwm = fix16_add(pwm_base, fix16_add(proportional, fix16_add(integral, derivative)));

	return pwm;
}





static void set_enc(uint8_t ji, int32_t count)
{
	for(int m=0; m<jpmtr[ji].num_motors; m++)
	{
		nxt_motor_set_count(jpmtr[ji].ports[m], (jpmtr[ji].mirror[m]) == TRUE ? -count : count);
	}
	j[ji].p = enc_cnt_to_jp(ji, count);
}


static int32_t sample_enc_cnt(uint8_t ci)
{
	uint8_t ji = joint_list[ci];								// sample encoder count
	int32_t enc_cnt = nxt_motor_get_count(jpmtr[ji].ports[0]);

	uint8_t new_sample = jctrl[ci].recent_sample + 1;
	if(new_sample == NUM_SAMPLES)
		new_sample = 0;

	jctrl[ci].enc_cnt[new_sample] = enc_cnt;			// store sample
	jctrl[ci].enc_cnt_ms[new_sample] = systick_get_ms();
	jctrl[ci].dt = fix16_mul(fix16_from_int(jctrl[ci].enc_cnt_ms[new_sample] - jctrl[ci].enc_cnt_ms[jctrl[ci].recent_sample]), S_PER_MS);
	jctrl[ci].recent_sample = new_sample;

	return enc_cnt;
}


static fix16_t estimate_enc_vel(uint8_t ci)
{
	uint8_t newest_sample = jctrl[ci].recent_sample;		// determine the index of the oldest sample in the circular buffer
	uint8_t oldest_sample = newest_sample + 1;
	if(oldest_sample == NUM_SAMPLES)
		oldest_sample = 0;
												// angular velocity (count/s) is deltaP/deltaT
	fix16_t dp = fix16_from_int(jctrl[ci].enc_cnt[newest_sample] - jctrl[ci].enc_cnt[oldest_sample]);
	fix16_t dt = fix16_from_int(jctrl[ci].enc_cnt_ms[newest_sample] - jctrl[ci].enc_cnt_ms[oldest_sample]);
	return fix16_mul(fix16_div(dp, dt), MS_PER_S);
}


static void apply_pwm(uint8_t ji, int32_t pwm)
{
	pwm = clamp_int(pwm, -100, 100);		// Clamp to allowable range (-100 to 100)

	if(is_joint_limit_enabled(ji))			// Implement joint angle limits (if enabled for this joint)
	{
		if(j[ji].p >= jpmtr[ji].pmax)			// Refuse to drive joint fwd if at upper angle limit
			pwm = clamp_int(pwm, -100, 0);
		else if(j[ji].p <= jpmtr[ji].pmin)		// Refuse to drive joint bwd if at lower angle limit
			pwm = clamp_int(pwm, 0, 100);
	}

	j[ji].pwm = (int8_t)pwm;

	for(int m=0; m<jpmtr[ji].num_motors; m++)	// apply to all motors attached to this joint
	{
		nxt_motor_set_speed(jpmtr[ji].ports[m], (jpmtr[ji].mirror[m]) == TRUE ? -pwm : pwm, BRAKEMODE);	// reverse pwm signal as necessary
	}
}


static fix16_t enc_cnt_to_jp(uint8_t ji, int32_t count)
{
	// jp = prest + count*gear_rec + sum( j[i].p*coaxial_rec[i] )
	//If homed properly, encoders should all be 0 at rest postion (prest).

	fix16_t jp = fix16_add(jpmtr[ji].prest, fix16_mul(fix16_from_int(count), jpmtr[ji].gear_rec));

	for(int i=0; i<6; i++)
	{
		fix16_t dep = jpmtr[ji].coaxial_rec[i];
		if(dep != 0)								// Perform this check before doing math because most coaxial dependencies are 0.
			jp = fix16_add(jp, fix16_mul(j[i].p, dep));
	}

	return jp;
}


static fix16_t enc_vel_to_jv(uint8_t ji, fix16_t enc_v)
{
	// jv = enc_v*gear_rec + sum( j[i].v*coaxial_rec[i] )

	fix16_t jv = fix16_mul(enc_v, jpmtr[ji].gear_rec);

	for(int i=0; i<6; i++)
	{
		fix16_t dep = jpmtr[ji].coaxial_rec[i];
		if(dep != 0)								// Perform this check before doing extra math because most coaxial dependencies are 0.
			jv = fix16_add(jv, fix16_mul(j[i].v, dep));
	}

	return jv;
}


static int32_t enc_cnt_from_jp(uint8_t ji, fix16_t jp)
{
	// count = (jp - prest - sum( j[i].p*coaxial_rec[i] ) ) * gear
	//If homed properly, encoders should all be 0 at rest postion (prest).

	fix16_t count = fix16_sub(jp, jpmtr[ji].prest);

	for(int i=0; i<6; i++)
	{
		fix16_t dep = jpmtr[ji].coaxial_rec[i];
		if(dep != 0)								// Perform this check before doing math because most coaxial dependencies are 0.
			count = fix16_sub(count, fix16_mul(j[i].p, dep));
	}

	count = fix16_mul(count, jpmtr[ji].gear);

	return fix16_to_int(count);
}


static void update_home_sw(uint8_t ci)
{
	uint8_t ji = joint_list[ci];

	BOOL sw_pressed = is_tmux_pressed(ji);

	switch(jctrl[ci].home_sw)
	{
		case RELEASED:
			if(sw_pressed)	// Rising edge of homing switch
			{
				jctrl[ci].enc_cnt_rising_edge = jctrl[ci].enc_cnt[jctrl[ci].recent_sample];		// use most recent enc_cnt
				jctrl[ci].home_sw = PRESSED;
			}
			break;

		case PRESSED:
			if(!sw_pressed)	// Falling edge of homing switch
			{
				jctrl[ci].enc_cnt_falling_edge = jctrl[ci].enc_cnt[jctrl[ci].recent_sample];	// use most recent enc_cnt

				if(!is_joint_limit_enabled(ji))	// homing mode is active
				{
					// encoder count at center of homing switch = rising + (falling-rising)/2
					int32_t rising = jctrl[ci].enc_cnt_rising_edge;
					int32_t falling = jctrl[ci].enc_cnt_falling_edge;	// also the current encoder count
					int32_t center = rising + ((falling - rising)/2);
					int32_t actual_center = enc_cnt_from_jp(ji, jpmtr[ji].phome);	// but it would be this if it were homed properly
					int32_t error = center - actual_center;
					int32_t corrected_current = rising - error;			// subtract error from current value to correct it
					set_enc(ji, corrected_current);
				}

				jctrl[ci].home_sw = RELEASED;
			}
			break;
	}
}










