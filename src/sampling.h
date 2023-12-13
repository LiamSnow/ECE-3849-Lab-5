#ifndef SAMPLING_H_
#define SAMPLING_H_

#include <stdint.h>

#define ADC_INT_PRIORITY 0X00
#define ADC_SEQUENCE_0 0
#define ADC_STEP_0 0
#define ADC_BUFFER_SIZE 2048 // size must be a power of 2
#define ADC_BUFFER_WRAP(i) ((i) & (ADC_BUFFER_SIZE - 1))

extern volatile int32_t gADCBufferIndex;
extern volatile uint16_t gADCBuffer[ADC_BUFFER_SIZE]; // circular buffer
extern volatile uint32_t gADCErrors; // number of missed ADC deadlines

void adc_init();
void adc_isr();
int32_t get_adc_buffer_index();




#endif
