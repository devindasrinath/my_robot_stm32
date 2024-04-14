#include "../motor_driver/motor_driver.h"

/* Initialize motor */
void motor_init(Motor *motor, GPIO_TypeDef *port, uint16_t pin_f, uint16_t pin_b, TIM_HandleTypeDef *timer, uint32_t channel) {
    motor->port = port;
    motor->pin_f = pin_f;
    motor->pin_b = pin_b;
    motor->timer = timer;
    motor->channel = channel;
}


/* Run motor at specified speed */
void motor_run(Motor *motor, double speed) {
    if (speed >= 0) {
        HAL_GPIO_WritePin(motor->port, motor->pin_f, GPIO_PIN_SET);
        HAL_GPIO_WritePin(motor->port, motor->pin_b, GPIO_PIN_RESET);
    } else {
        HAL_GPIO_WritePin(motor->port, motor->pin_f, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(motor->port, motor->pin_b, GPIO_PIN_SET);
        speed = -speed; // Make speed positive for PWM duty cycle
    }
    // Set PWM duty cycle based on speed
    __HAL_TIM_SET_COMPARE(motor->timer, motor->channel, (uint16_t)(speed * motor->timer->Instance->ARR));
}

/* Stop motor */
void motor_stop(Motor *motor) {
    HAL_GPIO_WritePin(motor->port, motor->pin_f, GPIO_PIN_SET);
    HAL_GPIO_WritePin(motor->port, motor->pin_b, GPIO_PIN_SET);
    // Set PWM duty cycle to 0
    __HAL_TIM_SET_COMPARE(motor->timer, motor->channel, 0);
}

/* Enable motors and start PWM */
void enable_motor(Motor *motor) {
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_SET); // Example GPIO pin for motor enable
    HAL_TIM_PWM_Start(motor->timer, motor->channel); // Start PWM for the specified motor
}

/* Enable motors and start PWM */
void disable_motor(Motor *motor) {
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_RESET); // Example GPIO pin for motor disable
    HAL_TIM_PWM_Stop(motor->timer, motor->channel); // Stop PWM for the specified motor
}

void calculate_pwm_values(PWM_Config *config, uint32_t *PSC_value, uint32_t *ARR_value) {
    *ARR_value = pow(2, config->pwm_resolution) - 2; // Ideal ARR value
    *PSC_value = (uint32_t)((config->pwm_input_freq) / ((*ARR_value + 1) * config->pwm_freq)) - 1;
}

