#ifndef MOTOR_DRIVER_H
#define MOTOR_DRIVER_H

#include "main.h"
#include <math.h>

/* Motor structure definition */
typedef struct {
    GPIO_TypeDef *port;         // GPIO port of motor control pins
    uint16_t pin_f;             // Forward pin of the motor
    uint16_t pin_b;             // Backward pin of the motor
    TIM_HandleTypeDef *timer;   // Timer used for PWM control
    uint32_t channel;           // Timer channel used for PWM control
} Motor;

typedef struct {
    uint8_t pwm_resolution;     // Number of bits used for the resolution
    uint32_t pwm_freq;          // PWM frequency in Hertz
    uint32_t pwm_input_freq;    // Input clock frequency in Hertz
} PWM_Config;

/* Public function prototypes */
void motor_init(Motor *motor, GPIO_TypeDef *port, uint16_t pin_f, uint16_t pin_b, TIM_HandleTypeDef *timer, uint32_t channel);
void motor_run(Motor *motor, double speed);
void motor_stop(Motor *motor);
void enable_motor(Motor *motor);
void calculate_pwm_values(PWM_Config *config, uint32_t *PSC_value, uint32_t *ARR_value);
#endif /* MOTOR_DRIVER_H */
