#include "isr.h"

ISR(PCINT2_vect)
{
    // If pin change is L->H, record start of pulse
    if (bit_is_set(PIND, PD7))
    {
        steeringStart = TCNT1;
    }
    else
    {
        // Do not log if start sampling is 0 (never sampled)
        if (steeringStart != 0)
        {
            sharedSteeringDuration = TCNT1 - steeringStart;
        }

        // If duration is negative, counter has overflowed so compenstate
        if (sharedSteeringDuration < 0)
        {
            sharedSteeringDuration += 20000;
            sharedTxFlags |= STEERING_FLAG;
        }
    }
}

