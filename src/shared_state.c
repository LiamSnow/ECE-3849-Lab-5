#include "shared_state.h"

uint32_t gSystemClock = 120000000; // [Hz] system clock frequency
volatile uint32_t gTime = 8345; // 1/100 sec

int16_t fftBuffer[NFFT];
int32_t rawADCBuffer[SCREEN_WIDTH];
int32_t processedADCBuffer[SCREEN_WIDTH];
int32_t processedADCBufferDone = false;

bool gDMAPrimary = true;

bool inFFTMode = false;
bool triggerModeRising = false;
bool signalDisconnected = true;

uint16_t timeScaleIndex = 0;
const float TIME_SCALES[] = {20, 50, 10000, 20000, 50000, 100000};
const int TIME_SCALES_SIZE = sizeof(TIME_SCALES) / sizeof(TIME_SCALES[0]);

uint8_t voltageScaleIndex = 3;
const float VOLTAGE_SCALES[] = {0.1, 0.2, 0.5, 1, 2, 4, 8};
const int VOLTAGE_SCALES_SIZE = sizeof(VOLTAGE_SCALES) / sizeof(VOLTAGE_SCALES[0]);
