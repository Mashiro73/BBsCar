/******************************************************************************
 * @file    pid_controller.c
 * @author  AI Assistant
 * @brief   Implementation of PID controller with encapsulated measurement.
 ******************************************************************************/

#include "pid_controller.h"
#include <float.h>
static inline float clamp(float value, float min_val, float max_val)
{
    if (value > max_val)
        return max_val; // Ensure this order for NaN propagation if desired
    if (value < min_val)
        return min_val;
    return value;
}

void PID_Init(PID_Controller_t *pid, uint8_t initial_mode,
              float Kp, float Ki, float Kd, float dt,
              float out_min, float out_max,
              uint8_t initial_derivative_on_measurement,
              uint8_t initial_enable_output_clamping,
              uint8_t initial_enable_integral_clamping)
{
    if (!pid)
        return;

    pid->mode = (initial_mode == PID_MODE_INCREMENTAL) ? PID_MODE_INCREMENTAL : PID_MODE_POSITIONAL;
    pid->Kp = Kp;
    pid->Ki = Ki;
    pid->Kd = Kd;
    pid->dt = dt;

    pid->out_min = out_min;
    pid->out_max = out_max;
    pid->integral_min = out_min;
    pid->integral_max = out_max;

    pid->derivative_on_measurement = (initial_derivative_on_measurement != 0);
    pid->enable_output_clamping = (initial_enable_output_clamping != 0);
    pid->enable_integral_clamping = (initial_enable_integral_clamping != 0);

    PID_Reset(pid); // Call reset to initialize all state variables
}

void PID_SetMeasurement(PID_Controller_t *pid, float measurement)
{ // <--- 新增函数的实现
    if (!pid)
        return;
    pid->current_measurement = measurement;
}

float PID_Compute(PID_Controller_t *pid)
{ // <--- 修改：移除了 measurement 参数
    if (!pid)
        return 0.0f;
    if (pid->dt <= 0.0f)
    {
        return (pid->mode == PID_MODE_INCREMENTAL) ? pid->prev_output : pid->current_measurement; // Use current_measurement
    }

    // 使用内部存储的 current_measurement 计算误差
    float error = pid->setpoint - pid->current_measurement;
    float calculated_output;
    uint8_t was_first_run = pid->first_run;

    if (pid->mode == PID_MODE_POSITIONAL)
    {
        float p_term = pid->Kp * error;
        float i_term;

        pid->integral_term += pid->Ki * error * pid->dt;
        if (pid->enable_integral_clamping)
        {
            pid->integral_term = clamp(pid->integral_term, pid->integral_min, pid->integral_max);
        }
        i_term = pid->integral_term;

        float d_term = 0.0f;
        if (pid->Kd > 0.0f)
        {
            if (was_first_run)
            {
                if (pid->derivative_on_measurement)
                {
                    // prev_measurement 在 Reset 中初始化 (通常为0)
                    d_term = pid->Kd * (pid->prev_measurement - pid->current_measurement) / pid->dt;
                }
                else
                {
                    d_term = 0.0f;
                }
            }
            else
            {
                if (pid->derivative_on_measurement)
                {
                    d_term = pid->Kd * (pid->prev_measurement - pid->current_measurement) / pid->dt;
                }
                else
                {
                    d_term = pid->Kd * (error - pid->prev_error) / pid->dt;
                }
            }
        }
        calculated_output = p_term + i_term + d_term;
    }
    else
    { // PID_MODE_INCREMENTAL
        float delta_output;
        if (was_first_run)
        {
            float delta_p_term = pid->Kp * error;
            float delta_i_term = pid->Ki * error * pid->dt;
            delta_output = delta_p_term + delta_i_term;
        }
        else
        {
            float delta_p_term = pid->Kp * (error - pid->prev_error);
            float delta_i_term = pid->Ki * error * pid->dt;
            float delta_d_term = 0.0f;
            if (pid->Kd > 0.0f)
            {
                delta_d_term = pid->Kd * (error - 2.0f * pid->prev_error + pid->prev_prev_error) / pid->dt;
            }
            delta_output = delta_p_term + delta_i_term + delta_d_term;
        }
        calculated_output = pid->prev_output + delta_output;
    }

    float final_output = calculated_output;
    if (pid->enable_output_clamping)
    {
        final_output = clamp(calculated_output, pid->out_min, pid->out_max);
    }

    // 更新状态为下一次迭代做准备
    if (pid->mode == PID_MODE_INCREMENTAL)
    {
        pid->prev_output = final_output;
        if (!was_first_run)
        {
            pid->prev_prev_error = pid->prev_error;
        }
        else
        {
            pid->prev_prev_error = error;
        }
    }
    // 更新 prev_measurement (用于位置式 DoM)
    // prev_measurement 存储的是 *本次* compute 使用的 current_measurement，供 *下次* DoM 计算
    if (pid->mode == PID_MODE_POSITIONAL && pid->derivative_on_measurement)
    {
        pid->prev_measurement = pid->current_measurement;
    }
    pid->prev_error = error; // 当前误差成为下一次迭代的 "上一个误差"

    if (was_first_run)
    {
        pid->first_run = 0; // 清除首次运行标志
    }

    return final_output;
}

void PID_SetSetpoint(PID_Controller_t *pid, float setpoint)
{
    if (!pid)
        return;
    pid->setpoint = setpoint;
}

void PID_SetTunings(PID_Controller_t *pid, float Kp, float Ki, float Kd)
{
    if (!pid)
        return;
    if (Kp < 0.0f || Ki < 0.0f || Kd < 0.0f)
        return;
    if (pid->dt <= 0.0f)
        return;
    pid->Kp = Kp;
    pid->Ki = Ki;
    pid->Kd = Kd;
}

void PID_SetOutputLimits(PID_Controller_t *pid, float min_val, float max_val)
{
    if (!pid)
        return;
    if (min_val >= max_val)
        return;
    pid->out_min = min_val;
    pid->out_max = max_val;
    if (pid->mode == PID_MODE_INCREMENTAL && pid->enable_output_clamping)
    {
        pid->prev_output = clamp(pid->prev_output, pid->out_min, pid->out_max);
    }
}

void PID_SetIntegralLimits(PID_Controller_t *pid, float min_val, float max_val)
{
    if (!pid)
        return;
    if (min_val >= max_val)
        return;
    pid->integral_min = min_val;
    pid->integral_max = max_val;
    if (pid->mode == PID_MODE_POSITIONAL && pid->enable_integral_clamping)
    {
        pid->integral_term = clamp(pid->integral_term, pid->integral_min, pid->integral_max);
    }
}

void PID_Reset(PID_Controller_t *pid)
{ // <--- 修改：初始化 current_measurement
    if (!pid)
        return;

    pid->integral_term = 0.0f;
    pid->prev_error = 0.0f;
    pid->prev_prev_error = 0.0f;
    pid->current_measurement = 0.0f; // 初始化当前测量值
    pid->prev_measurement = 0.0f;    // 初始化上一个测量值 (用于DoM)

    float initial_prev_output = 0.0f;
    if (pid->enable_output_clamping)
    {
        pid->prev_output = clamp(initial_prev_output, pid->out_min, pid->out_max);
    }
    else
    {
        pid->prev_output = initial_prev_output;
    }
    pid->first_run = 1;
}

void PID_SetMode(PID_Controller_t *pid, uint8_t mode)
{
    if (!pid)
        return;
    uint8_t new_mode = (mode == PID_MODE_INCREMENTAL) ? PID_MODE_INCREMENTAL : PID_MODE_POSITIONAL;
    if (pid->mode != new_mode)
    {
        pid->mode = new_mode;
        PID_Reset(pid);
    }
}

void PID_SetDerivativeMode(PID_Controller_t *pid, uint8_t use_derivative_on_measurement)
{
    if (!pid)
        return;
    pid->derivative_on_measurement = (use_derivative_on_measurement != 0);
}

void PID_EnableOutputClamping(PID_Controller_t *pid, uint8_t enable)
{
    if (!pid)
        return;
    pid->enable_output_clamping = (enable != 0);
    if (pid->enable_output_clamping && pid->mode == PID_MODE_INCREMENTAL)
    {
        pid->prev_output = clamp(pid->prev_output, pid->out_min, pid->out_max);
    }
}

void PID_EnableIntegralClamping(PID_Controller_t *pid, uint8_t enable)
{
    if (!pid)
        return;
    pid->enable_integral_clamping = (enable != 0);
    if (pid->enable_integral_clamping && pid->mode == PID_MODE_POSITIONAL)
    {
        pid->integral_term = clamp(pid->integral_term, pid->integral_min, pid->integral_max);
    }
}

