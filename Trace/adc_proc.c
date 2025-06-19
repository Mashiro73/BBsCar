#include "adc_proc.h"
#include "pid_controller.h"
#include <math.h>
#include "stm32f1xx_hal_adc.h"

uint16_t adc_raw_values[NUM_CHANNELS]; // Or uint32_t if using 12-bit resolution aligned to the right in a 32-bit register
uint8_t FlagADCDataReady = 0;          // 标志位，指示ADC数据是否准备好

// static uint16_t adc_val1[SAMPLES_PER_CHANNEL];
// static uint16_t adc_val2[SAMPLES_PER_CHANNEL];
// static uint16_t adc_val3[SAMPLES_PER_CHANNEL];
// static uint16_t adc_val4[SAMPLES_PER_CHANNEL];
float adc_samples[NUM_CHANNELS][SAMPLES_PER_CHANNEL];
float adc_ave_values[NUM_CHANNELS] = {0.0f, 0.0f, 0.0f, 0.0f};  // 用于存储每个通道的平均值
float adc_norm_values[NUM_CHANNELS] = {0.0f, 0.0f, 0.0f, 0.0f}; // 用于存储归一化后的值

#ifdef ADC_SERCH_MAX
float induc_ave_val1 = 0.0f;
float induc_ave_val2 = 0.0f;
float induc_ave_val3 = 0.0f;
float induc_ave_val4 = 0.0f;
#endif

/**
 * @brief 计算去除最大值和最小值后的平均值。
 *
 * @param values 包含4个元素的数组。
 * @param numofvals 数组中的元素数量，应该为4。
 * @return float 返回中间两个值的平均值。
 */
float trimmed_mean_filter(uint16_t *values, uint16_t numofvals)
{
    // 检查输入是否有效 (对于固定大小数组参数，此检查更多是形式上的，
    // 因为调用者应确保传入一个有效的含4个元素的数组)
    if (values == NULL)
    {
        return 0.0f;
    }

    float min_val = values[0];
    float max_val = values[0];
    float sum = 0.0f;

    // 1. 计算总和，并找出最大值和最小值
    for (int i = 0; i < numofvals; ++i)
    {
        sum += values[i];
        if (values[i] < min_val)
        {
            min_val = values[i];
        }
        if (values[i] > max_val)
        {
            max_val = values[i];
        }
    }

    // 2. 从总和中减去最大值和最小值，得到中间两个值的和
    float sum_of_middle_two = sum - min_val - max_val;

    // 3. 计算中间两个值的平均值 (总共4个数，去掉2个，剩下2个)
    return sum_of_middle_two / (numofvals - 2);
}
/**
 * @brief 将一个值从原始范围归一化到目标范围。
 *
 * @param value 要归一化的值。
 * @param original_min 原始值的理论或实际最小值。
 * @param original_max 原始值的理论或实际最大值。
 * @param target_min 目标范围的最小值。
 * @param target_max 目标范围的最大值。
 * @return float 归一化后的值。如果 original_min == original_max，则行为特定。
 */
float normalize_value(float value,
                      float original_min, float original_max,
                      float target_min, float target_max)
{
    // 处理原始范围为零的情况，避免除以零
    if (original_max == original_min)
    {
        // 如果原始范围是一个点，可以将该点映射到目标范围的起点、中点或返回一个固定值。
        // 这里我们假设如果值等于这个单点，就映射到目标范围的起点。
        return (value == original_min) ? target_min : (target_min + target_max) / 2.0f;
    }

    // 执行最小-最大归一化
    float normalized = ((value - original_min) / (original_max - original_min)) * (target_max - target_min) + target_min;

    // 可选：将结果严格限制在目标范围内，以防原始值超出 original_min/max 或浮点误差导致。

    return normalized;
}

/**
 * @brief 处理ADC值并计算平均值和归一化值。
 *
 * @return float 返回处理后的平均值或其他相关信息（如果需要）。
 */
float ADCValProc(void)
{
    for (uint8_t i = 0; i < NUM_CHANNELS; i++)
    {
        adc_ave_values[i] = trimmed_mean_filter(adc_samples[i], SAMPLES_PER_CHANNEL); // 计算每个通道的平均值
    }
    for (uint8_t j = 0; j < NUM_CHANNELS; j++)
    {
        adc_norm_values[j] = normalize_value(adc_ave_values[j], j == 0 || j == 3 ? TRANSVERSE_INDUCTANCE_MIN : VERTICAL_INDUCTANCE_MIN, j == 0 || j == 3 ? TRANSVERSE_INDUCTANCE_MAX : VERTICAL_INDUCTANCE_MAX, 0, 100); // 归一化每个通道的平均值
    }
}
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
    if (hadc->Instance == ADC1)
    {
        static uint8_t sample_count = 0;
        for (int i = 0; i < NUM_CHANNELS; i++)
        {
            adc_samples[i][sample_count] = adc_raw_values[i]; // 获取ADC值并存储到对应通道的样本数组中
        }
        sample_count++;
        if (sample_count >= SAMPLES_PER_CHANNEL)
        {
            sample_count = 0;     // 重置样本计数器
            FlagADCDataReady = 1; // 设置标志位，表示ADC数据已准备好
        }
    }
}
