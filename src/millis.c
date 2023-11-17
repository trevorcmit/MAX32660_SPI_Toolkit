#include <stdio.h>
#include <stdint.h>
#include "tmr.h"


volatile uint32_t msTicks = 0;

uint32_t millis(void)
{
    return msTicks;
}

// Parameters for Continuous timer
#define CONT_FREQ       1000 // (Hz), having this run every 1ms
#define CONT_TIMER      MXC_TMR2 // Can be MXC_TMR0 through MXC_TMR5

// Toggles GPIO when continuous timer repeats
void ContinuousTimerHandler(void)
{
    msTicks++;
    // Clear interrupt
    MXC_TMR_ClearFlags(CONT_TIMER);
}


void ContinuousTimer(void)
{
    // Declare variables
    mxc_tmr_cfg_t tmr;
    uint32_t periodTicks = MXC_TMR_GetPeriod(CONT_TIMER, 1, CONT_FREQ);

    /******************************************** 
     * Steps for configuring a timer for PWM mode:
     * 1. Disable the timer
     * 2. Set the prescale value
     * 3  Configure the timer for continuous mode
     * 4. Set polarity, timer parameters
     * 5. Enable Timer
    ********************************************/

    MXC_TMR_Shutdown(CONT_TIMER);

    tmr.pres = TMR_PRES_1;
    tmr.mode = TMR_MODE_CONTINUOUS;
    tmr.cmp_cnt = periodTicks;       // SystemCoreClock*(1/interval_time);
    tmr.pol = 0;

    MXC_TMR_Init(CONT_TIMER, &tmr);
    MXC_TMR_Start(CONT_TIMER);
}
