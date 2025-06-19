#ifndef __ADCPROC_H
#define __ADCPROC_H

#include "main.h"

#define ADC_SERCH_MAX

#define NUM_CHANNELS 4
#define SAMPLES_PER_CHANNEL 10
#define TOTAL_SAMPLES (NUM_CHANNELS * SAMPLES_PER_CHANNEL)

#define VERTICAL_INDUCTANCE_MAX 4096
#define TRANSVERSE_INDUCTANCE_MAX 4096

#define VERTICAL_INDUCTANCE_MIN 0
#define TRANSVERSE_INDUCTANCE_MIN 0

extern uint16_t adc_raw_values[TOTAL_SAMPLES]; // Or uint32_t if using 12-bit resolution aligned to the right in a 32-bit register

extern uint8_t FlagADCDataReady; // 标志位，指示ADC数据是否准备好

#ifdef ADC_SERCH_MAX
extern float induc_ave_val1;
extern float induc_ave_val2;
extern float induc_ave_val3;
extern float induc_ave_val4;
#endif

extern float adc_ave_values[NUM_CHANNELS] = {0.0f, 0.0f, 0.0f, 0.0f}; // 用于存储每个通道的平均值
float ADCValProc(void);

#endif