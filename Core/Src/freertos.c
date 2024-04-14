/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "mainpp.h"
#include "../../ECUAL/motor_driver/motor_driver.h"
#include "../../ECUAL/motor_speed/motor_speed.h"
#include "../../ECUAL/pid/pid_v1.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define ROS_LOOP_PERIOD_MS 100
#define PID_LOOP_PERIOD_MS 10
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
int internal_command = 0;
extern PIDController pid_controller_left_motor;
extern PIDController pid_controller_right_motor;
extern uint8_t command;
extern double left_vel;
extern double right_vel;
extern MotorDynamics left_motor_dynamics ;
extern MotorDynamics right_motor_dynamics ;
extern double real_left_vel;
extern double real_right_vel;
extern Motor motor1;
extern Motor motor2;
/* USER CODE END Variables */
osThreadId defaultROSTaskHandle;
osTimerId myPIDTimer01Handle;
osTimerId myROSTimer02Handle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void const * argument);
void Callback01(void const * argument);
void Callback02(void const * argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* GetIdleTaskMemory prototype (linked to static allocation support) */
void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize );

/* GetTimerTaskMemory prototype (linked to static allocation support) */
void vApplicationGetTimerTaskMemory( StaticTask_t **ppxTimerTaskTCBBuffer, StackType_t **ppxTimerTaskStackBuffer, uint32_t *pulTimerTaskStackSize );

/* USER CODE BEGIN GET_IDLE_TASK_MEMORY */
static StaticTask_t xIdleTaskTCBBuffer;
static StackType_t xIdleStack[configMINIMAL_STACK_SIZE];

void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize )
{
  *ppxIdleTaskTCBBuffer = &xIdleTaskTCBBuffer;
  *ppxIdleTaskStackBuffer = &xIdleStack[0];
  *pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
  /* place for user code */
}
/* USER CODE END GET_IDLE_TASK_MEMORY */

/* USER CODE BEGIN GET_TIMER_TASK_MEMORY */
static StaticTask_t xTimerTaskTCBBuffer;
static StackType_t xTimerStack[configTIMER_TASK_STACK_DEPTH];

void vApplicationGetTimerTaskMemory( StaticTask_t **ppxTimerTaskTCBBuffer, StackType_t **ppxTimerTaskStackBuffer, uint32_t *pulTimerTaskStackSize )
{
  *ppxTimerTaskTCBBuffer = &xTimerTaskTCBBuffer;
  *ppxTimerTaskStackBuffer = &xTimerStack[0];
  *pulTimerTaskStackSize = configTIMER_TASK_STACK_DEPTH;
  /* place for user code */
}
/* USER CODE END GET_TIMER_TASK_MEMORY */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* Create the timer(s) */
  /* definition and creation of myPIDTimer01 */
  osTimerDef(myPIDTimer01, Callback01);
  myPIDTimer01Handle = osTimerCreate(osTimer(myPIDTimer01), osTimerPeriodic, NULL);

  /* definition and creation of myROSTimer02 */
  osTimerDef(myROSTimer02, Callback02);
  myROSTimer02Handle = osTimerCreate(osTimer(myROSTimer02), osTimerPeriodic, NULL);

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of defaultROSTask */
  osThreadDef(defaultROSTask, StartDefaultTask, osPriorityNormal, 0, 128);
  defaultROSTaskHandle = osThreadCreate(osThread(defaultROSTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultROSTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const * argument)
{
  /* USER CODE BEGIN StartDefaultTask */
	osTimerStart (myPIDTimer01Handle,PID_LOOP_PERIOD_MS );
	osTimerStart (myROSTimer02Handle,ROS_LOOP_PERIOD_MS );

  /* Infinite loop */
  for(;;)
  {

	  osDelay(1);
  }
  /* USER CODE END StartDefaultTask */
}

/* Callback01 function */
void Callback01(void const * argument)
{
  /* USER CODE BEGIN Callback01 */
	update_encoder_count_per_sampling_period(&left_motor_dynamics,((int)(TIM3->CNT) >> 2));
	update_encoder_count_per_sampling_period(&right_motor_dynamics,((int)(TIM4->CNT) >> 2));

	//if(osSemaphoreWait (myVelWriteBinarySem01Handle,  0) == pdTRUE){
		pid_controller_left_motor.target_value = left_vel;
		pid_controller_right_motor.target_value = right_vel;
		internal_command = command;

	//}

	if(internal_command != 0)
	{

		pid_controller_left_motor.current_value = get_speed_count(&left_motor_dynamics);
		pid_controller_right_motor.current_value = get_speed_count(&right_motor_dynamics);

		real_left_vel = get_speed_rpm(&left_motor_dynamics);
		real_right_vel = get_speed_rpm(&right_motor_dynamics);

		//osSemaphoreRelease (myVelReadBinarySem02Handle);

		PID_calculate(&pid_controller_left_motor);
		PID_calculate(&pid_controller_right_motor);

		motor_run(&motor2,pid_controller_left_motor.output/1024.0);
		motor_run(&motor1,pid_controller_right_motor.output/1024.0);
	}
	else
	{

		real_left_vel = 0;
		real_right_vel = 0;
		motor_stop(&motor2);
		motor_stop(&motor1);
		PID_reset(&pid_controller_left_motor);
		PID_reset(&pid_controller_right_motor);
	}
  /* USER CODE END Callback01 */
}

/* Callback02 function */
void Callback02(void const * argument)
{
  /* USER CODE BEGIN Callback02 */
	loop();
  /* USER CODE END Callback02 */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

