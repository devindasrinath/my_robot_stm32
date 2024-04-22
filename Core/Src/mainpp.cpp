/*
 * main.cpp

 *
 *  Created on: 2018/01/17
 *      Author: yoneken
 */
#include <mainpp.h>
#include <ros.h>
#include <std_msgs/String.h>
#include <string>
#include <std_msgs/Float32MultiArray.h>
#include <FreeRTOS.h>
#include "cmsis_os.h"

#define VELOCITY_TO_TICK(x) roundf((x/60)*495)
#define TICK_TO_VELOCTIY(x) ((x*60.0)/495.0)

enum  robot_action {
 ROBOT_STOP = 0,
 ROBOT_MOVE  = 1
};

extern volatile uint8_t command;
extern volatile double left_vel;
extern volatile double right_vel;

volatile double real_left_vel = 0;
volatile double real_right_vel = 0;

ros::NodeHandle nh;

std_msgs::String str_msg;

ros::Publisher chatter("stm32_to_pc", &str_msg);


double vel_left = 0;
double vel_right = 0;

void stringCallback(const std_msgs::Float32MultiArray& msg)
{

	vel_left = msg.data[0];
	vel_right = msg.data[1];

	if ((vel_left == 0) &&(vel_right == 0)){
		command = ROBOT_STOP;
		left_vel = 0;
		right_vel = 0;
	}
	else {
		command = ROBOT_MOVE;
		left_vel = VELOCITY_TO_TICK(vel_left);
		right_vel = VELOCITY_TO_TICK(vel_right);
	}

}

ros::Subscriber<std_msgs::Float32MultiArray> listener("wheel_velocity", &stringCallback);


void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart){
  nh.getHardware()->flush();
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
  nh.getHardware()->reset_rbuf();
}



void setup(void)
{
  nh.initNode();
  nh.subscribe(listener);
  nh.advertise(chatter);
}

char str[100];

void loop(void)
{

	std::sprintf(str, "[%f,%f],[%f,%f]", real_left_vel,real_right_vel,TICK_TO_VELOCTIY(left_vel),TICK_TO_VELOCTIY(right_vel));
	str_msg.data = str;
	chatter.publish(&str_msg);

	nh.spinOnce();
}



