#include "timer_support.h"


uint8_t isTimerOverFlowed(void)
{
    // Overflowed if TOV0 is set to 0
    if ((TIFR0 & (1 << TOV0)))
    {
        return 0;
    }

    return 1;
}

void resetTimer(void)
{
    TCNT0 = 0x00;
}

void clearOverflowFlag(void)
{
    TIFR0 |= (1 << TOV0);
}

void resetTimerAndOvr(void)
{
    resetTimer();
    clearOverflowFlag();
}

void setTimer(uint8_t timerVal)
{
    TCNT0 = timerVal;
}
