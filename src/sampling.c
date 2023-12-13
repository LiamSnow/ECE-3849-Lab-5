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
#include "driverlib/udma.h"


#include "inc/hw_types.h"
#include "inc/hw_memmap.h"
#include "inc/tm4c1294ncpdt.h"

#include "kiss_fft.h"
#include "_kiss_fft_guts.h"

#include "sampling.h"
#include "buttons.h"
#include "src/shared_state.h"

volatile uint32_t gADCValue = 0;
volatile uint32_t gADCSampleCount = 0;
//volatile int32_t gADCBufferIndex = ADC_BUFFER_SIZE - 1;
volatile uint16_t gADCBuffer[ADC_BUFFER_SIZE]; // circular buffer
volatile uint32_t gADCErrors = 0; // number of missed ADC deadlines

// ADC Initialization
void adc_init(void) {
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
    GPIOPinTypeADC(GPIO_PORTE_BASE, GPIO_PIN_0); // GPIO setup for analog input AIN3
    // initialize ADC peripherals
    SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC0);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC1);
    // ADC clock
    uint32_t pll_frequency = SysCtlFrequencyGet(CRYSTAL_FREQUENCY);
    uint32_t pll_divisor = (pll_frequency - 1) / (16 * ADC_SAMPLING_RATE) + 1; // round up
    ADCClockConfigSet(ADC0_BASE, ADC_CLOCK_SRC_PLL |
    ADC_CLOCK_RATE_FULL,
    pll_divisor);
    ADCClockConfigSet(ADC1_BASE, ADC_CLOCK_SRC_PLL |
    ADC_CLOCK_RATE_FULL,
    pll_divisor);
    // choose ADC1 sequence 0; disable before configuring
    ADCSequenceDisable(ADC1_BASE, ADC_SEQUENCE_0);
    ADCSequenceConfigure(ADC1_BASE, ADC_SEQUENCE_0, ADC_TRIGGER_ALWAYS, ADC_INT_PRIORITY); // specify the "Always" trigger
    // in the 0th step, sample channel 3 (AIN3)
    // enable interrupt, and make it the end of sequence
    ADCSequenceStepConfigure(ADC1_BASE, ADC_SEQUENCE_0, ADC_STEP_0, ADC_CTL_CH3 | ADC_CTL_IE | ADC_CTL_END);
    // enable the sequence. it is now sampling

    ADCSequenceEnable(ADC1_BASE, 0);   // enable the sequence. it is now sampling
    ADCSequenceDMAEnable(ADC1_BASE, 0); // enable DMA for ADC1 sequence 0
    ADCIntEnableEx(ADC1_BASE, ADC_INT_DMA_SS0); // enable ADC1 sequence 0 DMA interrupt

//    ADCSequenceEnable(ADC1_BASE, ADC_SEQUENCE_0);
    // enable sequence 0 interrupt in the ADC1 peripheral
//    ADCIntEnable(ADC1_BASE, ADC_SEQUENCE_0);
    // IntPrioritySet(INT_ADC1SS0, ADC_INT_PRIORITY); // set ADC1 sequence 0 interrupt priority
    // enable ADC1 sequence 0 interrupt in int. controller
    // IntEnable(INT_ADC1SS0);
}

// ADC ISR
void adc_isr(void) {
    ADCIntClearEx(ADC1_BASE, ADC_INT_DMA_SS0); // clear the ADC1 sequence 0 DMA interrupt flag
    // Check the primary DMA channel for end of transfer, and restart if needed.
    if (uDMAChannelModeGet(UDMA_SEC_CHANNEL_ADC10 | UDMA_PRI_SELECT) == UDMA_MODE_STOP) {
        uDMAChannelTransferSet(UDMA_SEC_CHANNEL_ADC10 | UDMA_PRI_SELECT,
                               UDMA_MODE_PINGPONG, (void*)&ADC1_SSFIFO0_R,
                               (void*)&gADCBuffer[0], ADC_BUFFER_SIZE/2); // restart the primary channel (same as setup)
        gDMAPrimary = false; // DMA is currently occurring in the alternate buffer
    }//end if
    // Check the alternate DMA channel for end of transfer, and restart if needed.
    // Also set the gDMAPrimary global.
    if(uDMAChannelModeGet(UDMA_SEC_CHANNEL_ADC10 | UDMA_ALT_SELECT) == UDMA_MODE_STOP) {
        uDMAChannelTransferSet(UDMA_SEC_CHANNEL_ADC10 | UDMA_ALT_SELECT,
                               UDMA_MODE_PINGPONG, (void*)&ADC1_SSFIFO0_R,
                               (void*)&gADCBuffer[ADC_BUFFER_SIZE/2], ADC_BUFFER_SIZE/2); // restart alternate channel
        gDMAPrimary = true;
    }//end if
    // The DMA channel may be disabled if the CPU is paused by the debugger.
    if (!uDMAChannelIsEnabled(UDMA_SEC_CHANNEL_ADC10)) {
        uDMAChannelEnable(UDMA_SEC_CHANNEL_ADC10); // re-enable the DMA channel
    }
}

int32_t get_adc_buffer_index()
{
    int32_t index;
    IArg gatekey = GateHwi_enter(gateHwi0);
    if (gDMAPrimary) { // DMA is currently in the primary channel
        index = ADC_BUFFER_SIZE/2 - 1 -
                uDMAChannelSizeGet(UDMA_SEC_CHANNEL_ADC10 | UDMA_PRI_SELECT);
        GateHwi_leave(gateHwi0, gatekey);
    }
    else { // DMA is currently in the alternate channel
        index = ADC_BUFFER_SIZE - 1 -
                uDMAChannelSizeGet(UDMA_SEC_CHANNEL_ADC10 | UDMA_ALT_SELECT);
        GateHwi_leave(gateHwi0, gatekey);

    }
    return index;
}
