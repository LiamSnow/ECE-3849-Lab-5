#include "waveform.h"
#include "shared_state.h"
#include "sampling.h"

#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <math.h>

#include <xdc/std.h>
#include <xdc/runtime/System.h>
#include <xdc/cfg/global.h>

#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Task.h>

#include "driverlib/interrupt.h"
#include "driverlib/fpu.h"
#include "driverlib/sysctl.h"
#include "driverlib/gpio.h"
#include "driverlib/pwm.h"
#include "driverlib/pin_map.h"
#include "driverlib/adc.h"
#include "driverlib/timer.h"

#include "inc/hw_types.h"
#include "inc/hw_memmap.h"
#include "inc/tm4c1294ncpdt.h"

int find_rising_trigger_index() {
    int triggerIndex = ADC_BUFFER_WRAP(get_adc_buffer_index() - SCREEN_WIDTH / 2);
    int searchStartIndex = ADC_BUFFER_WRAP(triggerIndex - ADC_BUFFER_SIZE / 2);
    
    for (; triggerIndex != searchStartIndex; triggerIndex = ADC_BUFFER_WRAP(triggerIndex - 1)) {
        if (gADCBuffer[triggerIndex] >= ADC_OFFSET &&
            gADCBuffer[ADC_BUFFER_WRAP(triggerIndex - 1)] < ADC_OFFSET) {
            signalDisconnected = false;
            return triggerIndex; // Trigger found
        }
    }

    signalDisconnected = true;
    
    // Trigger not found, use the most recent samples
    return ADC_BUFFER_WRAP(get_adc_buffer_index() - SCREEN_WIDTH / 2);
}
int find_falling_trigger_index() {
    int triggerIndex = ADC_BUFFER_WRAP(get_adc_buffer_index() - SCREEN_WIDTH / 2);
    int searchStartIndex = ADC_BUFFER_WRAP(triggerIndex - ADC_BUFFER_SIZE / 2);
    
    for (; triggerIndex != searchStartIndex; triggerIndex = ADC_BUFFER_WRAP(triggerIndex - 1)) {
        if (gADCBuffer[triggerIndex] < ADC_OFFSET &&
            gADCBuffer[ADC_BUFFER_WRAP(triggerIndex - 1)] >= ADC_OFFSET) {
            signalDisconnected = false;
            return triggerIndex; // Trigger found
        }
    }
    
    signalDisconnected = true;

    // Trigger not found, use the most recent samples
    return ADC_BUFFER_WRAP(get_adc_buffer_index() - SCREEN_WIDTH / 2);
}
void waveform_task_func() {
    IntMasterEnable();

    while (true) {
        Semaphore_pend(waveform_sem, BIOS_WAIT_FOREVER);

        //FFT Mode
        if (inFFTMode) {
            int index = get_adc_buffer_index();
            int i;
            for (i = 0; i < 1024; i++) {
                fftBuffer[i] = gADCBuffer[index];
                index = ADC_BUFFER_WRAP(index - 1);
            }
        }

        //Oscilliscope Mode
        else {
            int i, triggerIndex = (triggerModeRising) ? find_rising_trigger_index() : find_falling_trigger_index();
            for (i = 0; i < SCREEN_WIDTH; i++) {
                rawADCBuffer[i] = gADCBuffer[ADC_BUFFER_WRAP(triggerIndex - SCREEN_WIDTH / 2 + i)];
            }
        }

        Semaphore_post(processing_sem);
    }
}
