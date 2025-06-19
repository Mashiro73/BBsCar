#ifndef __MOTOR_H
#define __MOTOR_H

#include "main.h"

#define MOTOR_BASE_SPEED 200
#define MOTOR_MAX_SPEED 1000
#define MOTOR_MIN_SPEED 0

typedef struct {
    int speed;
    int direction; // 1 for forward, -1 for backward
    int pwm_channel; // PWM channel for the motor
    TIM_HandleTypeDef *htim; // Timer handle for PWM
    GPIO_TypeDef *dir_port; // GPIO port for direction control
    uint16_t dir_pin; // GPIO pin for direction control
} Motor;

void Motor_Init(Motor *motor);

void Motor_SetSpeed(Motor *motor, int speed);

#endif