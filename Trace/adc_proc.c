#include "adc_proc.h"
#include "pid_controller.h"
#include <math.h>
#include "stm32f1xx_hal_adc.h"

uint8_t FlagADCDataReady = 0;                   // 标志位，指示ADC数据是否准备好

volatile uint16_t adc_raw_values[NUM_CHANNELS]; // Or uint32_t if using 12-bit resolution aligned to the right in a 32-bit register
volatile float adc_samples[NUM_CHANNELS][SAMPLES_PER_CHANNEL];
volatile float adc_ave_values[NUM_CHANNELS] = {0.0f, 0.0f, 0.0f, 0.0f};  // 用于存储每个通道的平均值
volatile float adc_norm_values[NUM_CHANNELS] = {0.0f, 0.0f, 0.0f, 0.0f}; // 用于存储归一化后的值

static uint16_t INMAXVAL[4] = {1750, 2450, 2000, 2250}; // 归一化最大值
static uint16_t INMINVAL[4] = {0, 0, 0, 0};             // 归一化最小值

volatile float adc_final_diff;
/**
 * @brief 计算去除最大值和最小值后的平均值。
 *
 * @param values 包含4个元素的数组。
 * @param numofvals 数组中的元素数量，应该为4。
 * @return float 返回中间两个值的平均值。
 */
static void trimmed_mean_filter(void)
{
    for (uint8_t i = 0; i < NUM_CHANNELS; i++)
    {
        float sum = 0.0f;
        float min_val = adc_samples[i][0];
        float max_val = adc_samples[i][0];

        // 1. 计算总和，并找出最大值和最小值
        for (int j = 0; j < SAMPLES_PER_CHANNEL; ++j)
        {
            sum += adc_samples[i][j];
            if (adc_samples[i][j] < min_val)
            {
                min_val = adc_samples[i][j];
            }
            if (adc_samples[i][j] > max_val)
            {
                max_val = adc_samples[i][j];
            }
        }

        // 2. 从总和中减去最大值和最小值，得到中间两个值的和
        float sum_of_middle_two = sum - min_val - max_val;

        // 3. 计算中间两个值的平均值 (总共4个数，去掉2个，剩下2个)
        adc_ave_values[i] = sum_of_middle_two / (SAMPLES_PER_CHANNEL - 2);
    }
}

static void normalize_value(void)
{
    for (uint8_t j = 0; j < NUM_CHANNELS; j++)
    {
        adc_norm_values[j] = ((adc_ave_values[j] - INMINVAL[j]) / (INMAXVAL[j] - INMINVAL[j])) * 100.0f; // 归一化每个通道的平均值到0-100范围
        if (adc_norm_values[j] < 0.0f)                                                                   // 确保归一化值不小于0
        {
            adc_norm_values[j] = 0.0f;
        }
        else if (adc_norm_values[j] > 100.0f) // 确保归一化值不大于100
        {
            adc_norm_values[j] = 100.0f;
        }
    }
}

static void calculate_final_diff(void)
{
    adc_final_diff = adc_norm_values[0] + adc_norm_values[1] - adc_norm_values[2] - adc_norm_values[3]; // 计算归一化值的差异
    if(adc_final_diff <-200.0f) // 确保差异值不小于-200
    {
        adc_final_diff = -200.0f;
    }
    else if(adc_final_diff > 200.0f) // 确保差异值不大于200
    {
        adc_final_diff = 200.0f;
    }
}
/**
 * @brief 处理ADC值并计算平均值和归一化值。
 *
 * @return float 返回处理后的平均值或其他相关信息（如果需要）。
 */
float ADCValProc(void)
{
    if (FlagADCDataReady) // 检查标志位是否设置
    {
        FlagADCDataReady = 0;
        trimmed_mean_filter();
        normalize_value();
        calculate_final_diff(); // 计算归一化值的差异
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
