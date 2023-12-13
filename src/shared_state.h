#ifndef SHARED_STATE_H_
#define SHARED_STATE_H_

#include <stdint.h>
#include <stdbool.h>
#include "driverlib/udma.h"

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 128

#define ADC_BUFFER_WRAP(i) ((i) & (ADC_BUFFER_SIZE - 1))
#define ADC_OFFSET 2048
#define ADC_BITS 12

#define PI 3.14159265358979f
#define NFFT 1024 // FFT length
#define KISS_FFT_CFG_SIZE (sizeof(struct kiss_fft_state)+sizeof(kiss_fft_cpx)*(NFFT-1))

#define PWM_FREQUENCY 20000

extern uint32_t gSystemClock; // [Hz] system clock frequency
extern volatile uint32_t gTime; // 1/100 sec

extern int16_t fftBuffer[NFFT];
extern int32_t rawADCBuffer[SCREEN_WIDTH];
extern int32_t processedADCBuffer[SCREEN_WIDTH];
extern int32_t processedADCBufferDone;

#pragma DATA_ALIGN(gDMAControlTable, 1024) // address alignment required
tDMAControlTable gDMAControlTable[64]; // uDMA control table (global)
extern bool gDMAPrimary;

extern bool inFFTMode;
extern bool triggerModeRising;
extern bool signalDisconnected;

extern uint16_t timeScaleIndex;
extern const float TIME_SCALES[];
extern const int TIME_SCALES_SIZE;

extern uint8_t voltageScaleIndex;
extern const float VOLTAGE_SCALES[];
extern const int VOLTAGE_SCALES_SIZE;

#endif
