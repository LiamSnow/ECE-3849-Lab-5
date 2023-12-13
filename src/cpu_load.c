#include "cpu_load.h"
#include "shared_state.h"

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

float cpuLoad = 0.0f;
uint32_t unloadedCount = 0;

uint32_t measure_cpu_load_cycles() {
    uint32_t count = 0;
    TimerEnable(TIMER3_BASE, TIMER_A);
    while (!(TimerIntStatus(TIMER3_BASE, false) & TIMER_TIMA_TIMEOUT)) {
        count++;
    }
    TimerIntClear(TIMER3_BASE, TIMER_TIMA_TIMEOUT);
    TimerDisable(TIMER3_BASE, TIMER_A);
    return count;
}

void cpu_load_init() {
    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER3);
    TimerDisable(TIMER3_BASE, TIMER_BOTH);
    TimerConfigure(TIMER3_BASE, TIMER_CFG_ONE_SHOT);
    // Load the timer with the count corresponding to 10 ms
    TimerLoadSet(TIMER3_BASE, TIMER_A, gSystemClock / 100);
    unloadedCount = measure_cpu_load_cycles();
    cpuLoad = 100.0f - (unloadedCount / (float)unloadedCount) * 100.0f;
}

void measure_cpu_load() {
    uint32_t loadedCount = measure_cpu_load_cycles();
    cpuLoad = (1.0f - (float)loadedCount / (float)unloadedCount) * 100.0f;
}
