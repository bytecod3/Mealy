/*
 * pid.h
 *
 *  Created on: Jan 19, 2024
 *      Author: Edwin Mwiti
 */

#ifndef INC_PID_H_
#define INC_PID_H_

#include <stdint.h>


extern double sample_time;

#define PID_MAX 600
#define PID_MIN	-600
#define INTEGRAL_MAX_GAIN 1000
#define INTEGRAL_MIN_GAIN -1000


/**
 * struct to hold PID values
 */
typedef struct {
	float p_gain;					// proportional constant
	float i_gain;					// integral constant
	float d_gain;					// derivative constant
	int16_t last_error;			// last error computed
	int32_t error_integral;	// integral error
	int16_t output;			// corrected value output

} pid_instance;

/**
 * Set PID values
 */
void set_pid(pid_instance* pid_instance, uint16_t kp, uint16_t ki, uint16_t kd);

/**
 * Compute PID
 */
void apply_pid(pid_instance* pid_instance, int16_t input_error, float sample_rate);


#endif /* INC_PID_H_ */
