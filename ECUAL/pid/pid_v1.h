/*
 * pid_v1.h
 *
 *  Created on: Mar 14, 2024
 *      Author: devin
 */

#ifndef PID_PID_V1_H_
#define PID_PID_V1_H_

// Define PID structure
typedef struct {
    double target_value;
    double current_value;
    double prev_value;
    double error_integral;
    double prev_error;
    double kp, ki, kd;
    double output;
    double error;
    double dt; // Time step
    double upper_bound;
    double lower_bound;
} PIDController;


void PID_Init(PIDController *pid, double kp, double ki, double kd, double dt, double lower_bound, double upper_bound);
void PID_calculate(PIDController *pid);
void PID_reset(PIDController *pid);

#endif /* PID_PID_V1_H_ */
