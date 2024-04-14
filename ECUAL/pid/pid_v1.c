/*
 * generic_pid_v3.c
 *
 *  Created on: Mar 13, 2024
 *      Author: devin
 */

#include <stdio.h> // Include necessary libraries
#include "pid_v1.h"


// Define PID functions
void PID_Init(PIDController *pid, double kp, double ki, double kd, double dt, double lower_bound, double upper_bound) {
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->error_integral = 0;
    pid->prev_error = 0;
    pid->output = 0;
    pid->error = 0;
    pid->dt = dt;
    pid->lower_bound = lower_bound;
    pid->upper_bound = upper_bound;
}


void PID_calculate(PIDController *pid){ // No dt parameter
    pid->error =  pid->target_value -pid->current_value;
    pid->error_integral += (pid->error * pid->dt); // Scale the accumulated error by dt
    pid->output = (pid->error * pid->kp) + (pid->error_integral * pid->ki) + (((pid->error - pid->prev_error) / pid->dt) * pid->kd); // Include dt in derivative term
    pid->prev_error = pid->error;

    if(pid->upper_bound < pid->output){
        pid->output= pid->upper_bound;
    }
    else if(pid->lower_bound > pid->output){
        pid->output= pid->lower_bound;
    }
}


void PID_reset(PIDController *pid) {
    pid->error_integral = 0;
    pid->prev_error = 0;
    pid->output = 0;
    pid->error = 0;
}
