poop
#include <avr/io.h>
#include <avr/interrupt.h>

#include "config.h"
#include "isr.h"
#include "support.h"

#define DEBUG_EN 0
#define CLR_TX_FLAGS 0x00

extern volatile int16_t sharedSteeringDuration;
extern volatile uint8_t sharedTxFlags;
extern uint16_t steeringStart;

static int32_t scaledFilteredAccelX[2];
static int32_t velocityX[2];
static int32_t refYaw[2];
static uint8_t noMovementCount;
static uint16_t steeringDuration;
static int32_t steeringRadSc;
static int16_t filteredAccelLSB;
static int32_t sessionAccelOffset;
static int32_t sessionGyroOffset;

int main(void)
{
    bootstrapHardware();
    setIMUOffsets(&sessionAccelOffset, &sessionGyroOffset, DEBUG_EN);

    while (1)
    { 
        // TODO: Take time sample here
        
        if (sharedTxFlags)
        {
            cli();
            steeringDuration = sharedSteeringDuration;
            sharedTxFlags = CLR_TX_FLAGS;
            sei();
        }
        else
        {
            // TODO: Reset time sample
            continue;
        }

        steeringRadSc = pwmToRadSc(steeringDuration, DEBUG_EN);
        filteredAccelLSB = filterAccelLSB(sessionAccelOffset, DEBUG_EN); 
        scaledFilteredAccelX[1] = scaleFilteredAccel(filteredAccelLSB, DEBUG_EN);
        
        // TODO: Insert time sample into velocity computation
        getCurrNormalizedVelocity(velocityX, scaledFilteredAccelX, 
            &noMovementCount, DEBUG_EN);
       
        if (velocityX[1] != 0)
        {
            int32_t yawNum, yawDen;
            int32_t currYawRad =
                getScaledYawInRad(
                    filterGyroYawLSB(sessionGyroOffset, DEBUG_EN), 
                    DEBUG_EN
                );
            int8_t yawDiff = 
                getRefYaw(refYaw, velocityX, steeringRadSc, &yawNum,
                    &yawDen, DEBUG_EN) 
                - currYawRad;

            printYawDebugInfo(currYawRad, yawDiff, DEBUG_EN);
            OCR1A = getCorrectedSteeringPwm(yawNum, yawDen, yawDiff, DEBUG_EN);

            // TODO:Reset time sample
        }
        else
        {
            OCR1A = steeringDuration;
            // TODO: Reset time sample
        }
    }

    return 0;
}

