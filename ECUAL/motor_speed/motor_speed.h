/*
 * motor_speed.h
 *
 *  Created on: Mar 13, 2024
 *      Author: devin
 */

#ifndef MOTOR_SPEED_MOTOR_SPEED_H_
#define MOTOR_SPEED_MOTOR_SPEED_H_

#include <stdint.h>
#include <math.h>

#define SAMPLING_PERIOD 0.01
#define MINIUTE_DURATION 60

typedef struct{
    int sum;
    uint8_t num_samples;
    int8_t samples[100];
    uint8_t sample_count;
}AverageFilter;

typedef struct {
	double encoder_count_per_sampling_period;
	double rpm ;
	int count;
	double linear_velocity;
	double previous_encoder_count;
	double current_encoder_count;
	double wheel_diameter;
	double gear_ratio ;
	double counts_per_revolution;
    AverageFilter* averageFilter;
}MotorDynamics;


void my_func(MotorDynamics* motor_dynamics);
void init_motor_dynamics(MotorDynamics* motor_dynamics, double wheel_diameter, double gear_ratio, double counts_per_revolution, AverageFilter* averageFilter);
void update_encoder_count_per_sampling_period(MotorDynamics* motor_dynamics, double current_encoder_count);
double get_speed_rpm(MotorDynamics* motor_dynamics);
double get_speed_liner_ms(MotorDynamics* motor_dynamics);
int get_speed_count(MotorDynamics* motor_dynamics) ;
static void add_to_filter_buffer(AverageFilter *filter, int8_t encoder_count);
static double filter_output(AverageFilter *filter);
void reset_filter(AverageFilter *filter);
void init_filter(AverageFilter *filter , uint8_t num_samples);

#endif /* MOTOR_SPEED_MOTOR_SPEED_H_ */
