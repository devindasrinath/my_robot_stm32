/*
 * motor_speed.c
 *
 *  Created on: Mar 13, 2024
 *      Author: devin
 */

#include <stdlib.h>
#include "motor_speed.h"

/**
 * @brief Initializes a MotorDynamics structure with specified parameters.
 *
 * This function initializes members with default values and specified parameters.
 *
 * @param motor_dynamics Podoubleer to the MotorDynamics structure to be initialized.
 * @param wheel_diameter Wheel diameter in meters.
 * @param gear_ratio Gear ratio.
 * @param counts_per_revolution Counts per revolution.
 */
void init_motor_dynamics(MotorDynamics* motor_dynamics, double wheel_diameter, double gear_ratio, double counts_per_revolution, AverageFilter* averageFilter, TIM_HandleTypeDef* htim) {
    // Initialize members with default values
    motor_dynamics->encoder_count_per_sampling_period = 0;
    motor_dynamics->rpm = 0;
    motor_dynamics->linear_velocity = 0;
    motor_dynamics->previous_encoder_count = 0;
    motor_dynamics->current_encoder_count = 0;

    // Initialize members with arguments
    motor_dynamics->wheel_diameter = wheel_diameter;
    motor_dynamics->gear_ratio = gear_ratio;
    motor_dynamics->counts_per_revolution = counts_per_revolution;

    motor_dynamics->averageFilter = averageFilter;
    motor_dynamics->htim = htim;
}

/**
 * @brief Calculate encoder count per the recent sampling period
 *
 * This function calculates the change in encoder_count over a 10 millisecond doubleerval.
 * It updates the MotorDynamics struct with the current tick count.
 * We call this function inside of the timer callback function.
 *
 * @param motor_dynamics Podoubleer to MotorDynamics struct
 * @param current_encoder_count Current tick count
 */
void update_encoder_count_per_sampling_period(MotorDynamics* motor_dynamics) {
    motor_dynamics->current_encoder_count = (double) (__HAL_TIM_GET_COUNTER(motor_dynamics->htim)>>2);

    if(__HAL_TIM_IS_TIM_COUNTING_DOWN(motor_dynamics->htim) && (motor_dynamics->current_encoder_count > motor_dynamics->previous_encoder_count)) {
    	/* Underflow condition */
    	motor_dynamics->encoder_count_per_sampling_period = motor_dynamics->current_encoder_count - 1 - __HAL_TIM_GET_AUTORELOAD(motor_dynamics->htim) - motor_dynamics->previous_encoder_count;
    }
    else if(!__HAL_TIM_IS_TIM_COUNTING_DOWN(motor_dynamics->htim) && (motor_dynamics->current_encoder_count < motor_dynamics->previous_encoder_count )){
    	/* Overflow condition */
    	motor_dynamics->encoder_count_per_sampling_period = motor_dynamics->current_encoder_count + 1 + __HAL_TIM_GET_AUTORELOAD(motor_dynamics->htim) - motor_dynamics->previous_encoder_count;
    }
   	else {
   		/* normal counting condition */
   		motor_dynamics->encoder_count_per_sampling_period = motor_dynamics->current_encoder_count - motor_dynamics->previous_encoder_count;
   }

    motor_dynamics->previous_encoder_count = motor_dynamics->current_encoder_count;
    add_to_filter_buffer(motor_dynamics->averageFilter, (int8_t)motor_dynamics->encoder_count_per_sampling_period);
}

/**
 * @brief Get speed in RPM
 *
 * This function calculates the motor speed in revolutions per minute (RPM)
 * based on the encoder_count per the recent sampling period and motor parameters.
 *
 * @param motor_dynamics Podoubleer to MotorDynamics struct
 * @return Motor speed in RPM
 */
int get_speed_count(MotorDynamics* motor_dynamics) {
    return motor_dynamics->count = filter_output(motor_dynamics->averageFilter) / SAMPLING_PERIOD;
}

/**
 * @brief Get speed in RPM
 *
 * This function calculates the motor speed in revolutions per minute (RPM)
 * based on the encoder_count per the recent sampling period and motor parameters.
 *
 * @param motor_dynamics Podoubleer to MotorDynamics struct
 * @return Motor speed in RPM
 */
double get_speed_rpm(MotorDynamics* motor_dynamics) {
    return motor_dynamics->rpm = (filter_output(motor_dynamics->averageFilter) * MINIUTE_DURATION) /
                                 (motor_dynamics->gear_ratio * motor_dynamics->counts_per_revolution * SAMPLING_PERIOD);
}

/**
 * @brief Get linear speed in m/s
 *
 * This function calculates the linear speed in meters per second (m/s)
 * based on the encoder_count per the recent sampling period of time
 *
 * @param motor_dynamics Podoubleer to MotorDynamics struct
 * @return Linear speed in m/s
 */
double get_speed_linear_ms(MotorDynamics* motor_dynamics) {
    return motor_dynamics->linear_velocity = (motor_dynamics->encoder_count_per_sampling_period * M_PI * motor_dynamics->wheel_diameter) /
                                        (motor_dynamics->gear_ratio * motor_dynamics->counts_per_revolution * SAMPLING_PERIOD);
}



void add_to_filter_buffer(AverageFilter *filter, int8_t encoder_count){

	filter->sum = encoder_count;
    for(uint8_t i =99; i>0; i--){
        filter->samples[i] = filter->samples[i-1];
        filter->sum += filter->samples[i-1];
    }
    filter->samples[0] = encoder_count;

	if(filter->sample_count <filter->num_samples ){
		filter->sample_count++;
	}

}

double filter_output(AverageFilter *filter){
    return  ((double) (filter->sum))/(filter->sample_count);
}


void reset_filter(AverageFilter *filter){

	filter->sum =0;
	filter->sample_count = 0;

	for(int i = 0; i < filter->num_samples; i++ ){
		filter->samples[0] = 0;
	}

}

void init_filter(AverageFilter *filter , uint8_t num_samples){

	filter->num_samples =  num_samples;

	filter->sum = 0;

	filter -> sample_count = 0;

	for(int i = 0; i < filter->num_samples; i++ ){
		filter->samples[0] = 0;
	}

}
