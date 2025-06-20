#include "Motor.h"

void Motor_Init(Motor *motor)
{
    HAL_TIM_PWM_Start(motor->htim, motor->pwm_channel);
}

 void Motor_SetSpeed(Motor *motor, int speed)
{
    if (speed < 0)
    {
        speed = -speed;
        motor->direction = -1; // Set direction to backward
    }
    else
    {
        motor->direction = 1; // Set direction to forward
    }
    if (speed > MOTOR_MAX_SPEED)
    {
        speed = MOTOR_MAX_SPEED;
    }
    else if (speed < MOTOR_MIN_SPEED)
    {
        speed = MOTOR_MIN_SPEED;
    }
    motor->speed = speed;
    __HAL_TIM_SET_COMPARE(motor->htim, motor->pwm_channel, speed);
    HAL_GPIO_WritePin(motor->dir_port, motor->dirIN1, (GPIO_PinState)((motor->direction == 1) ? 1 : 0));
    HAL_GPIO_WritePin(motor->dir_port, motor->dirIN2, (GPIO_PinState)((motor->direction == -1) ? 1 : 0));
}
