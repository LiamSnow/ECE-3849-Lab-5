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

#include "src/buttons.h"
#include "src/sampling.h"
#include "src/display.h"
#include "src/cpu_load.h"
#include "src/shared_state.h"

void pwm_init() {
    // configure M0PWM2, at GPIO PF2, BoosterPack 1 header C1 pin 2
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
    GPIOPinTypePWM(GPIO_PORTF_BASE, GPIO_PIN_2);
    GPIOPinConfigure(GPIO_PF2_M0PWM2);
    GPIOPadConfigSet(GPIO_PORTF_BASE, GPIO_PIN_2,
    GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD);
    // configure the PWM0 peripheral, gen 1, outputs 2 and 3
    SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM0);
    // use system clock without division
    PWMClockSet(PWM0_BASE, PWM_SYSCLK_DIV_1);
    PWMGenConfigure(PWM0_BASE, PWM_GEN_1,
    PWM_GEN_MODE_DOWN | PWM_GEN_MODE_NO_SYNC);
    PWMGenPeriodSet(PWM0_BASE, PWM_GEN_1,
    roundf((float)gSystemClock/PWM_FREQUENCY));
    PWMPulseWidthSet(PWM0_BASE, PWM_OUT_2,
    roundf((float)gSystemClock/PWM_FREQUENCY*0.4f));
    PWMOutputState(PWM0_BASE, PWM_OUT_2_BIT, true);
    PWMGenEnable(PWM0_BASE, PWM_GEN_1);
}

void dma_init() {
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UDMA);
    uDMAEnable();
    uDMAControlBaseSet(gDMAControlTable);
    uDMAChannelAssign(UDMA_CH24_ADC1_0); // assign DMA channel 24 to ADC1 sequence 0
    uDMAChannelAttributeDisable(UDMA_SEC_CHANNEL_ADC10, UDMA_ATTR_ALL);
    // primary DMA channel = first half of the ADC buffer
    uDMAChannelControlSet(UDMA_SEC_CHANNEL_ADC10 | UDMA_PRI_SELECT,
                          UDMA_SIZE_16 | UDMA_SRC_INC_NONE | UDMA_DST_INC_16 | UDMA_ARB_4);
    uDMAChannelTransferSet(UDMA_SEC_CHANNEL_ADC10 | UDMA_PRI_SELECT,
                           UDMA_MODE_PINGPONG, (void*)&ADC1_SSFIFO0_R,
                           (void*)&gADCBuffer[0], ADC_BUFFER_SIZE/2);
    // alternate DMA channel = second half of the ADC buffer
    uDMAChannelControlSet(UDMA_SEC_CHANNEL_ADC10 | UDMA_ALT_SELECT,
                          UDMA_SIZE_16 | UDMA_SRC_INC_NONE | UDMA_DST_INC_16 | UDMA_ARB_4);
    uDMAChannelTransferSet(UDMA_SEC_CHANNEL_ADC10 | UDMA_ALT_SELECT,
                           UDMA_MODE_PINGPONG, (void*)&ADC1_SSFIFO0_R,
                           (void*)&gADCBuffer[ADC_BUFFER_SIZE/2], ADC_BUFFER_SIZE/2);
    uDMAChannelEnable(UDMA_SEC_CHANNEL_ADC10);
}

int main(void) {
    IntMasterDisable();

    display_init();
    button_init();
    adc_init();
    dma_init();
    pwm_init();
    cpu_load_init();

    BIOS_start();
    return (0);
}

void ui_task_func() {
    Button btn;
    bool success;
    while (true) {
        success = Mailbox_pend(buttons_mail, &btn, BIOS_WAIT_FOREVER);
        if (success) {
            if (btn == BTN_S1) {
//                timeScaleIndex = (timeScaleIndex + 1) % TIME_SCALES_SIZE;
                voltageScaleIndex = (voltageScaleIndex == 0 ? VOLTAGE_SCALES_SIZE - 1 : voltageScaleIndex - 1);
            }
            else if (btn == BTN_S2) {
                voltageScaleIndex = (voltageScaleIndex + 1) % VOLTAGE_SCALES_SIZE;
            }
            else if (btn == BTN_USR_SW_1) {
                triggerModeRising = !triggerModeRising;
            }
            else if (btn == BTN_USR_SW_2) {
                inFFTMode = !inFFTMode;
            }
        }
    }
}

static char kiss_fft_cfg_buffer[KISS_FFT_CFG_SIZE];
size_t buffer_size = KISS_FFT_CFG_SIZE;
kiss_fft_cfg cfg;
static kiss_fft_cpx in[NFFT], out[NFFT];
float out_db[NFFT];                                     // buffer of db readings

void processing_task_func() {
    cfg = kiss_fft_alloc(NFFT, 0, kiss_fft_cfg_buffer, &buffer_size);
    int i;

    while (true) {
        Semaphore_pend(processing_sem, BIOS_WAIT_FOREVER);

        processedADCBufferDone = false;

        //FFT
        if (inFFTMode) {
            for (i = 0; i < NFFT; i++) { // generate an input waveform
//                in[i].r = sinf(20 * PI * i / NFFT);
                in[i].r = fftBuffer[i];
                in[i].i = 0; // imaginary part of waveform
            }
            kiss_fft(cfg, in, out); // compute FFT

            // convert first 128 bins of out[] to dB for display
            for (i = 0; i < SCREEN_HEIGHT - 1; i++){
                processedADCBuffer[i] = 160 - ((10.0f) * log10f((out[i].r * out[i].r) + (out[i].i * out[i].i)));
            }
        }

        //Osciliscope
        else {
            float voltageScale = VOLTAGE_SCALES[voltageScaleIndex];
            float fScale = (3.3f * GRID_GAP_Y) / ((1 << ADC_BITS) * voltageScale);

            uint16_t x;
            for (x = 0; x < SCREEN_WIDTH; ++x) {
                processedADCBuffer[x] = SCREEN_HEIGHT / 2 - (int)roundf(fScale * ((int)rawADCBuffer[x] - ADC_OFFSET));
            }
        }


        processedADCBufferDone = true;

        Semaphore_post(display_sem);
    }
}


