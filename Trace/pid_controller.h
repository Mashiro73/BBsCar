/******************************************************************************
 * @file    pid_controller.h
 * @author  AI Assistant
 * @brief   A portable PID controller library with configurable strategies
 * and encapsulated measurement update.
 ******************************************************************************/

#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

#include <stdint.h> // For standard integer types
#include <float.h>  // For FLT_MAX, FLT_MIN

#ifdef __cplusplus
extern "C"
{
#endif

// PID Controller Modes
#define PID_MODE_POSITIONAL 0
#define PID_MODE_INCREMENTAL 1

    // PID Controller Structure
    typedef struct
    {
        // Controller gains
        float Kp;
        float Ki;
        float Kd;

        // Setpoint
        float setpoint;
        float current_measurement; // <--- 新增：存储当前测量值

        // Output limits values
        float out_min;
        float out_max;

        // Integral term limits values (for anti-windup in positional mode)
        float integral_min;
        float integral_max;

        // Controller state
        float integral_term; // (Positional mode)
        float prev_error;
        float prev_prev_error;  // (Incremental mode D-term)
        float prev_measurement; // (Positional mode DoM) - stores the measurement from the previous cycle
        float prev_output;      // (Incremental mode)

        // Time step
        float dt;

        // Modes and Strategy Enables
        uint8_t mode;
        uint8_t derivative_on_measurement;
        uint8_t enable_output_clamping;
        uint8_t enable_integral_clamping;
        uint8_t first_run;

    } PID_Controller_t;

    /**
     * @brief Initializes the PID controller structure.
     * (Parameters for clamping enables are included)
     */
    void PID_Init(PID_Controller_t *pid, uint8_t initial_mode,
                  float Kp, float Ki, float Kd, float dt,
                  float out_min, float out_max,
                  uint8_t initial_derivative_on_measurement,
                  uint8_t initial_enable_output_clamping,
                  uint8_t initial_enable_integral_clamping);

    /**
     * @brief Sets the current measurement value for the PID controller.
     * This should be called before each call to PID_Compute().
     *
     * @param pid Pointer to the PID_Controller_t structure.
     * @param measurement The current measured value from the system.
     */
    void PID_SetMeasurement(PID_Controller_t *pid, float measurement); // <--- 新增函数

    /**
     * @brief Computes the PID output using the internally stored setpoint and measurement.
     * Call this function at regular intervals specified by 'dt'.
     *
     * @param pid Pointer to the PID_Controller_t structure.
     * @return Calculated PID output (absolute value for both modes).
     */
    float PID_Compute(PID_Controller_t *pid); // <--- 修改：移除了 measurement 参数

    // ... (其他函数声明 PID_SetSetpoint, PID_SetTunings, etc. 保持不变) ...
    void PID_SetSetpoint(PID_Controller_t *pid, float setpoint);
    void PID_SetTunings(PID_Controller_t *pid, float Kp, float Ki, float Kd);
    void PID_SetOutputLimits(PID_Controller_t *pid, float min_val, float max_val);
    void PID_SetIntegralLimits(PID_Controller_t *pid, float min_val, float max_val);
    void PID_Reset(PID_Controller_t *pid);
    void PID_SetMode(PID_Controller_t *pid, uint8_t mode);
    void PID_SetDerivativeMode(PID_Controller_t *pid, uint8_t use_derivative_on_measurement);
    void PID_EnableOutputClamping(PID_Controller_t *pid, uint8_t enable);
    void PID_EnableIntegralClamping(PID_Controller_t *pid, uint8_t enable);

#ifdef __cplusplus
}
#endif

#endif // PID_CONTROLLER_H