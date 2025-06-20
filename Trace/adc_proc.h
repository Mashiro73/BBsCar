#ifndef __ADCPROC_H
#define __ADCPROC_H

#include "main.h"

//#define ADC_TEST_RAW
#define ADC_TEST_AVE
//#define ADC_TEST_NORM

#define NUM_CHANNELS 4
#define SAMPLES_PER_CHANNEL 10
#define TOTAL_SAMPLES (NUM_CHANNELS * SAMPLES_PER_CHANNEL)

#define VERTICAL_INDUCTANCE_MAX 4096
#define TRANSVERSE_INDUCTANCE_MAX 4096

#define VERTICAL_INDUCTANCE_MIN 0
#define TRANSVERSE_INDUCTANCE_MIN 0


extern uint8_t FlagADCDataReady; // 标志位，指示ADC数据是否准备好


extern uint16_t adc_raw_values[NUM_CHANNELS]; // Or uint32_t if using 12-bit resolution aligned to the right in a 32-bit register
extern float adc_ave_values[NUM_CHANNELS]; // 用于存储每个通道的平均值
extern float adc_norm_values[NUM_CHANNELS] ; // 用于存储归一化后的值

float ADCValProc(void);

#endif