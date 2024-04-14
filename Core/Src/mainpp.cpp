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

extern uint8_t command;
extern double left_vel;
extern double right_vel;
extern osSemaphoreId myVelWriteBinarySem01Handle;
extern osSemaphoreId myVelReadBinarySem02Handle;

double real_left_vel = 0;
double real_right_vel = 0;

ros::NodeHandle nh;

std_msgs::String str_msg;

ros::Publisher chatter("stm32_to_pc", &str_msg);

double vel_left = 0;
double vel_right = 0;

void stringCallback(const std_msgs::Float32MultiArray& msg)
{

	double vel_left = msg.data[0];
	double vel_right = msg.data[1];

	if ((vel_left == 0) &&(vel_right == 0)){
		command = 0;
		left_vel =0;
		right_vel = 0;
	}
	else {
		command = 1;
		left_vel = roundf((vel_left/60)*495);
		right_vel = roundf((vel_right/60)*495);
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

double internal_real_left_vel = 0;
double internal_real_right_vel = 0;
char str[100];

void loop(void)
{

	std::sprintf(str, "[%f,%f],[%f,%f]", real_left_vel,real_right_vel,left_vel*60.0/495,right_vel*60.0/495);
	str_msg.data = str;
	chatter.publish(&str_msg);

	nh.spinOnce();
}



