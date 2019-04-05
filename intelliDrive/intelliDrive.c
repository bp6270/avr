#include <avr/io.h>
#include <avr/interrupt.h>

#include "config.h"
#include "isr.h"
#include "timer_support.h"

extern volatile int16_t sharedSteeringDuration;
extern volatile uint8_t sharedTxFlags;
extern uint16_t steeringStart;

int main(void)
{
    static uint16_t steeringDuration;
    //static uint8_t timerOvrCnt;
    
    bootstrapHardware();

    while (1)
    { 
        if (sharedTxFlags)
        {
            cli();
            steeringDuration = sharedSteeringDuration;
            sharedTxFlags = 0x00;
            sei();
        }
        else
        {
            resetTimerAndOvr();
            //timerOvrCnt = 0;
            continue;
        }
       
        OCR1A = steeringDuration;
    }

    return 0;
}

