/*
 * pid.c
 *
 *  Created on: Jan 19, 2024
 *      Author: USER
 */

#include "pid.h"

/**
 * Set PID gains
 */
void set_pid(pid_instance* pid_instance, uint16_t kp, uint16_t ki, uint16_t kd){

	pid_instance->p_gain = kp;
	pid_instance->i_gain = ki;
	pid_instance->d_gain = kd;
}

void apply_pid(pid_instance* pid_instance, int16_t input_error, unsigned long long sample_rate) {
	// sample rate is the frequency of the system

	pid_instance->error_integral += input_error;

	if(pid_instance->error_integral > INTEGRAL_MAX_GAIN) {
		pid_instance->error_integral = INTEGRAL_MAX_GAIN;
	}

	if(pid_instance->error_integral < INTEGRAL_MIN_GAIN) {
		pid_instance->error_integral = INTEGRAL_MIN_GAIN;
	}

	int16_t p = pid_instance->p_gain * input_error;
	int16_t i = pid_instance->i_gain * (pid_instance->error_integral) / sample_rate;
	int16_t d = pid_instance->d_gain *  sample_rate * (input_error - pid_instance->last_error);

	pid_instance->output = p + i + d;

	// we only want positive value of PID
//	if(pid_instance->output < 0) {
//		pid_instance->output *= -1;
//	}

	/* cap gain values  */
	if(pid_instance->output >= PID_MAX) {
		pid_instance->output = PID_MAX;
	}

	if(pid_instance->output <= PID_MIN) {
		pid_instance->output = PID_MIN;
	}


	pid_instance->last_error = input_error;

}

