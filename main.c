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
#include "src/audio_waveform.h"

#define PWM_PERIOD_ADJUSTMENT 1
#define PWM_DUTY_CYCLE 0.4f

#define PWM_AUDIO_PERIOD 258

volatile uint32_t pwmPeriod;

uint32_t gPWMSample = 0; // PWM sample counter
uint32_t gSamplingRateDivider; // sampling rate divider

void pwm_init() {
    // configure M0PWM2, at GPIO PF2, BoosterPack 1 header C1 pin 2
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
    GPIOPinTypePWM(GPIO_PORTF_BASE, GPIO_PIN_2 | GPIO_PIN_3); // Include GPIO_PIN_3
    GPIOPinConfigure(GPIO_PF2_M0PWM2);
    GPIOPinConfigure(GPIO_PF3_M0PWM3); // Configure PF3 for M0PWM3
    GPIOPadConfigSet(GPIO_PORTF_BASE, GPIO_PIN_2,
                     GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD);
    GPIOPadConfigSet(GPIO_PORTF_BASE, GPIO_PIN_3, // Configure pad for PF3
                     GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD);

    // configure the PWM0 peripheral, gen 1, outputs 2 and 3
    SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM0);
    // use system clock without division
    PWMClockSet(PWM0_BASE, PWM_SYSCLK_DIV_1);
    PWMGenConfigure(PWM0_BASE, PWM_GEN_1,
                    PWM_GEN_MODE_DOWN | PWM_GEN_MODE_NO_SYNC);

    pwmPeriod = roundf((float)gSystemClock/20000);
    PWMGenPeriodSet(PWM0_BASE, PWM_GEN_1,
                    pwmPeriod);
    PWMPulseWidthSet(PWM0_BASE, PWM_OUT_2,
                     roundf(pwmPeriod*PWM_DUTY_CYCLE));
    PWMPulseWidthSet(PWM0_BASE, PWM_OUT_3, // Set pulse width for PF3
                     roundf(pwmPeriod*PWM_DUTY_CYCLE));
    PWMOutputState(PWM0_BASE, PWM_OUT_2_BIT | PWM_OUT_3_BIT, true); // Enable both outputs
    PWMGenEnable(PWM0_BASE, PWM_GEN_1);
}

void audio_pwm_init() {
    // Enable the GPIOG peripheral for use with PWM
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOG);
    // Configure PG1 as M0PWM5
    GPIOPinTypePWM(GPIO_PORTG_BASE, GPIO_PIN_1);
    GPIOPinConfigure(GPIO_PG1_M0PWM5);
    GPIOPadConfigSet(GPIO_PORTG_BASE, GPIO_PIN_1,
                     GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD);

    SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM0);
    PWMClockSet(PWM0_BASE, PWM_SYSCLK_DIV_1);
    PWMGenConfigure(PWM0_BASE, PWM_GEN_2,
                    PWM_GEN_MODE_DOWN | PWM_GEN_MODE_NO_SYNC);
    PWMGenPeriodSet(PWM0_BASE, PWM_GEN_2, PWM_AUDIO_PERIOD);
    PWMPulseWidthSet(PWM0_BASE, PWM_OUT_5,
                     PWM_AUDIO_PERIOD / 2); // 50% duty cycle
    PWMOutputState(PWM0_BASE, PWM_OUT_5_BIT, true);
    PWMGenEnable(PWM0_BASE, PWM_GEN_2);
    // Configure but do not enable PWM interrupts for generator 2 (counter reaches 0)
    PWMGenIntTrigEnable(PWM0_BASE, PWM_GEN_2, PWM_INT_CNT_ZERO);

    gSamplingRateDivider = (gSystemClock / PWM_AUDIO_PERIOD) / AUDIO_SAMPLING_RATE;
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

void freq_counter_init() {
    // config GPIO PD0 as timer input T0CCP0 at BoosterPack Connector #1 pin 14
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);
    GPIOPinTypeTimer(GPIO_PORTD_BASE, GPIO_PIN_0);
    GPIOPinConfigure(GPIO_PD0_T0CCP0);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);
    TimerDisable(TIMER0_BASE, TIMER_BOTH);
    TimerConfigure(TIMER0_BASE, TIMER_CFG_SPLIT_PAIR | TIMER_CFG_A_CAP_TIME_UP);
    TimerControlEvent(TIMER0_BASE, TIMER_A, TIMER_EVENT_POS_EDGE);
    // use maximum load value
    TimerLoadSet(TIMER0_BASE, TIMER_A, 0xffff);
    // use maximum prescale value
    TimerPrescaleSet(TIMER0_BASE, TIMER_A, 0xff);
    TimerIntEnable(TIMER0_BASE, TIMER_CAPA_EVENT);
    TimerEnable(TIMER0_BASE, TIMER_A);
}

int main(void) {
    IntMasterDisable();

    display_init();
    button_init();
    adc_init();
    dma_init();
    pwm_init();
    cpu_load_init();
    freq_counter_init();
    audio_pwm_init();

    BIOS_start();
    return (0);
}

void audio_pwm_hwi_func(void) {
    PWMGenIntClear(PWM0_BASE, PWM_GEN_2, PWM_INT_CNT_ZERO); // clear PWM interrupt flag
    // waveform sample index
    int i = (gPWMSample++) / gSamplingRateDivider;
    // write directly to the PWM compare B register
    PWM0_2_CMPB_R = 1 + gWaveform[i];
    if (i == gWaveformSize-1) { // if at the end of the waveform array
        // disable these interrupts
        PWMIntDisable(PWM0_BASE, PWM_INT_GEN_2);
        // reset sample index so the waveform starts from the beginning
        gPWMSample = 0;
    }
}

void ui_task_func() {
    Button btn;
    bool success;
    while (true) {
        success = Mailbox_pend(buttons_mail, &btn, BIOS_WAIT_FOREVER);
        if (success) {
            if (btn == BTN_S1) {
                //timeScaleIndex = (timeScaleIndex + 1) % TIME_SCALES_SIZE;
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
            else if (btn == BTN_LEFT) {
                // Increase the PWM period to decrease the frequency
                pwmPeriod += PWM_PERIOD_ADJUSTMENT;
                PWMGenPeriodSet(PWM0_BASE, PWM_GEN_1, pwmPeriod);
                PWMPulseWidthSet(PWM0_BASE, PWM_OUT_2, roundf(pwmPeriod * PWM_DUTY_CYCLE));
                PWMPulseWidthSet(PWM0_BASE, PWM_OUT_3, roundf(pwmPeriod * PWM_DUTY_CYCLE));
            }
            else if (btn == BTN_RIGHT) {
                // Decrease the PWM period to increase the frequency
                if (pwmPeriod > PWM_PERIOD_ADJUSTMENT) { // Prevent underflow
                    pwmPeriod -= PWM_PERIOD_ADJUSTMENT;
                }
                PWMGenPeriodSet(PWM0_BASE, PWM_GEN_1, pwmPeriod);
                PWMPulseWidthSet(PWM0_BASE, PWM_OUT_2, roundf(pwmPeriod * PWM_DUTY_CYCLE));
                PWMPulseWidthSet(PWM0_BASE, PWM_OUT_3, roundf(pwmPeriod * PWM_DUTY_CYCLE));
            }
            else if (btn == BTN_UP) {
                PWMIntEnable(PWM0_BASE, PWM_INT_GEN_2);
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
            // generate an input waveform
            for (i = 0; i < NFFT; i++) {
                in[i].r = fftBuffer[i]; //sinf(20 * PI * i / NFFT)
                in[i].i = 0;
            }
            kiss_fft(cfg, in, out);

            // convert first 128 bins of out[] to dB for display
            for (i = 0; i < SCREEN_HEIGHT - 1; i++){
                processedADCBuffer[i] = 160 - ((10.0f) * log10f((out[i].r * out[i].r) + (out[i].i * out[i].i)));
            }
        }

        //Oscilloscope
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

void timer0_hwi_func(void) {
    static uint32_t last_count = 0;
    TimerIntClear(TIMER0_BASE, TIMER_CAPA_EVENT);

    uint32_t count = TimerValueGet(TIMER0_BASE, TIMER_A);
    timer0Period = (count - last_count) & 0xffffff;
    last_count = count;
}
