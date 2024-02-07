/*
 * pid.h
 *
 *  Created on: Jan 19, 2024
 *      Author: Edwin Mwiti
 */

#ifndef INC_PID_H_
#define INC_PID_H_

#include <stdint.h>


/**
 * struct to hold PID values
 */
typedef struct {
	float kp;					// proportional constant
	float ki;					// integral constant
	float kd;					// derivative constant
	int16_t last_error;			// last error computed
	int32_t error_intergral;	// integral error
	int16_t pid_output;			// corrected value output

} pid_instance;

/**
 * Set PID values
 */
void set_pid_gain(pid_instance* pid_instance, uint16_t kp, uint16_t ki, uint16_t kd);

/**
 * Compute PID
 */
void apply_pid(pid_instance* pid_instance, uint16_t input_error, uint16_t smapling_rate);


#endif /* INC_PID_H_ */
