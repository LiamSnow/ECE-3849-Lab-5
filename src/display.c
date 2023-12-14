#include "display.h"
#include "cpu_load.h"
#include "shared_state.h"

#include "Crystalfontz128x128_ST7735.h"

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

tContext sContext;

int32_t localADCBuffer[SCREEN_WIDTH];

void display_init() {
    Crystalfontz128x128_Init(); // Initialize the LCD display driver
    Crystalfontz128x128_SetOrientation(LCD_ORIENTATION_UP); // set screen orientation

    GrContextInit(&sContext, &g_sCrystalfontz128x128); // Initialize the grlib graphics context
    GrContextFontSet(&sContext, &g_sFontFixed6x8); // select font
}

void draw_grid() {
    uint16_t gridAxisNum = (NUM_GRID_LINES - 1) / 2, i;

    for (i = 0; i < NUM_GRID_LINES; i++) {
        uint32_t x = (i + 0.5) * GRID_GAP_X;
        uint32_t y = (i + 0.5) * GRID_GAP_Y;

        GrContextForegroundSet(&sContext, (i == gridAxisNum) ? ClrBlue : ClrMidnightBlue);

        GrLineDraw(&sContext, x, 0, x, SCREEN_HEIGHT - 1);
        GrLineDraw(&sContext, 0, y, SCREEN_WIDTH - 1, y);
    }
}

void draw_signal() {
    GrContextForegroundSet(&sContext, ClrYellow);

    //copy into local buffer
    uint16_t x;
    if (processedADCBufferDone) {
        for (x = 0; x < SCREEN_WIDTH; ++x) {
            localADCBuffer[x] = processedADCBuffer[x];
        }
    }

    //draw signal
    int32_t prevY = localADCBuffer[0];
    for (x = 1; x < SCREEN_WIDTH; ++x) {
        GrLineDraw(&sContext, x-1, prevY, x, localADCBuffer[x]);
        prevY = localADCBuffer[x];
    }
}

void display_task_func() {
    tRectangle clearScreenRect = {0, 0, SCREEN_WIDTH-1, SCREEN_HEIGHT-1};
    char str[50];

    while (true) {
        Semaphore_pend(display_sem, BIOS_WAIT_FOREVER);

        measure_cpu_load();

        //clear screen
        GrContextForegroundSet(&sContext, ClrBlack);
        GrRectFill(&sContext, &clearScreenRect);

        //draw screen
        draw_grid();
        draw_signal();

        //set text color
        GrContextForegroundSet(&sContext, ClrWhite);

        //FFT Mode
        if (inFFTMode) {
            GrStringDraw(&sContext, "20kHz  20dB", -1, 5, 0, false);
        }
        
        //Osciliscope Mode
        else {
            //draw time scale
            if (TIME_SCALES[timeScaleIndex] < 1000) {
                snprintf(str, sizeof(str), "%.0fus", TIME_SCALES[timeScaleIndex]);
            }
            else {
                snprintf(str, sizeof(str), "%.0fms", TIME_SCALES[timeScaleIndex] / 1000.0f);
            }
            GrStringDraw(&sContext, str, -1, 5, 0, false);

            //draw voltage scale
            if (VOLTAGE_SCALES[voltageScaleIndex] < 1) {
                snprintf(str, sizeof(str), "%.0fmV", VOLTAGE_SCALES[voltageScaleIndex] * 1000);
            }
            else {
                snprintf(str, sizeof(str), "%.0fV", VOLTAGE_SCALES[voltageScaleIndex]);
            }
            GrStringDraw(&sContext, str, -1, 50, 0, false);
        }

        //draw trigger type
        if (triggerModeRising) {
            GrLineDrawH(&sContext, 114, 121, 0);
            GrLineDrawV(&sContext, 114, 0, 7);
            GrLineDrawH(&sContext, 107, 114, 7);
            GrLineDraw(&sContext, 114, 2, 111, 5);
            GrLineDraw(&sContext, 114, 2, 117, 5);
        }
        else {
            GrLineDrawH(&sContext, 107, 114, 0);
            GrLineDrawV(&sContext, 114, 0, 7);
            GrLineDrawH(&sContext, 114, 121, 7);
            GrLineDraw(&sContext, 114, 5, 111, 2);
            GrLineDraw(&sContext, 114, 5, 117, 2);
        }

        //draw frequency and period
        float freq = (float)gSystemClock / timer0Period;
        snprintf(str, sizeof(str), "f=%.1f  T=%d", freq, timer0Period);
        GrStringDraw(&sContext, str, -1, 5, 110, false);

        //draw cpu load
        snprintf(str, sizeof(str), "CPU load: %.1f%%", cpuLoad);
        GrStringDraw(&sContext, str, -1, 5, 120, false);

        //draw signal disconnected
        if (signalDisconnected) {
            snprintf(str, sizeof(str), "X");
            GrStringDraw(&sContext, str, -1, 120, 120, false);
        }

        GrFlush(&sContext);

        Semaphore_post(waveform_sem);
    }
}
