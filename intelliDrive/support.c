#include "support.h"

//=============================================================================

int32_t divideRoundInt(int32_t dividend, int32_t divisor)
{
    int32_t halfDivisor = divisor / 2;

    if (dividend < 0)
    {
        halfDivisor *= -1;
    }

    return (dividend + halfDivisor) / divisor;
}

//=============================================================================

int32_t getSessionOffset(
    uint8_t devAddr, 
    uint8_t regAddrHi, 
    uint8_t regAddrLo,
    uint8_t debugEnabled
)
{
    int32_t sessionOffset = 0;
    uint16_t offsetCnt = 0;

    if (debugEnabled)
    {   
        printString("Calibrating...\r\n");
    }

    do {
        // Grab the accel counts
        int8_t LSBHi = twiReadByte(devAddr, regAddrHi);
        int8_t LSBLo = twiReadByte(devAddr, regAddrLo);
        int16_t LSB = (LSBHi << 8) | LSBLo;
        
        sessionOffset += LSB;
        offsetCnt++;

    } while (offsetCnt != 1024);

    sessionOffset = sessionOffset >> 10;

    if (debugEnabled)
    {
        printString("Calibration completed!\r\n");
        printString("Offset: ");
        printVal32(sessionOffset);
        printString("\r\n");
    }

    return sessionOffset;
}

//=============================================================================

int32_t getAccelOffset(uint8_t debugEnabled)
{
    if (debugEnabled)
    {
        printString("Calibrating acceleration...\r\n");
    }

    return getSessionOffset(MPU_ADDR, ACCEL_XOUT_H, ACCEL_XOUT_L, debugEnabled);
}

//=============================================================================

int32_t getGyroYawOffset(uint8_t debugEnabled)
{
    if (debugEnabled)
    {
        printString("Calibrating gyro yaw...\r\n");
    }

    return getSessionOffset(MPU_ADDR, GYRO_ZOUT_H, GYRO_ZOUT_L, debugEnabled);
}

//=============================================================================

void setIMUOffsets(int32_t *accOs, int32_t *yawOs, uint8_t debugEnabled)
{
    *accOs = getAccelOffset(debugEnabled);
    *yawOs = getGyroYawOffset(debugEnabled);
}

//=============================================================================

int32_t pwmToRadSc(uint16_t steeringDuration, uint8_t debugEnabled)
{
    // Bound the input
    if ((steeringDuration < 850) || (steeringDuration > 2200) ||
        (steeringDuration > 1480 && steeringDuration < 1520))
    {
        steeringDuration = 1500;
    }

    if (debugEnabled)
    {
        printString("Corrected steering duration: ");
        printVal(steeringDuration);
        printString("\r\n");
    }

    int32_t steeringRadSc = ((int32_t) -33 * steeringDuration) + 50154;
    steeringRadSc =
        divideRoundInt(divideRoundInt(steeringRadSc * 3217, 180), 1024);

    if (debugEnabled)
    {
        printString("PWM in: ");
        printVal(steeringDuration);
        printString("\r\n");

        printString("Steering Rad Scaled: ");
        printVal32(steeringRadSc);
        printString("\r\n");
    }

    return steeringRadSc;
}

//=============================================================================

int16_t filterAccelLSB(int32_t sessionOffset, uint8_t debugEnabled)
{
    uint8_t filterAccelCnt = 0;
    int32_t filterAccelTotalLSB = 0;
    int16_t filteredAccelLSB = 0;

    do {
        int8_t accelXCntHi = twiReadByte(MPU_ADDR, ACCEL_XOUT_H);
        int8_t accelXCntLo = twiReadByte(MPU_ADDR, ACCEL_XOUT_L);
        int16_t accelXCnt = (accelXCntHi << 8) | accelXCntLo;

        filterAccelTotalLSB += accelXCnt;
        filterAccelCnt++;

    } while (filterAccelCnt != 64);

    filteredAccelLSB = (filterAccelTotalLSB >> 6);

    if (debugEnabled)
    {
        printString("Pre-filtered accel LSB: ");
        printVal(filteredAccelLSB);
        printString("\r\n");
    }

    // Apply calibrationa and zero filter
    filteredAccelLSB -=  sessionOffset;

    if (debugEnabled)
    {
        printString("Session offset: ");
        printVal(sessionOffset);
        printString("\r\n");
        printString("Filtered accel LSB: ");
        printVal(filteredAccelLSB);
        printString("\r\n");
    }

    // Discrimination window for X acceleration
    if ((filteredAccelLSB < 300) && (filteredAccelLSB > -300))
    {
        filteredAccelLSB = 0;
    }

    return filteredAccelLSB;
}

//=============================================================================

int32_t scaleFilteredAccel(int16_t filteredAccelLSB, uint8_t debugEnabled)
{
    // Get scaled acceleration in terms of 9.8 m/s^2
    int32_t scaledFilteredAccelX =
        divideRoundInt((int32_t) filteredAccelLSB * 1024, ACCEL_X_SSF);

    if (debugEnabled)
    {
        printString("Scaled Filtered Accel: ");
        printVal32(scaledFilteredAccelX);
        printString("\r\n");
    }
    
    return scaledFilteredAccelX;
}

//=============================================================================

int32_t getCurrNormalizedVelocity(
    int32_t *velocityArr, 
    int32_t *scaledFilteredAccArr, 
    uint8_t *noMovementCnt, 
    uint8_t debugEnabled
)
{
    // Account for no-movement condition
    if (*(scaledFilteredAccArr + 1) == 0)
    {
        *noMovementCnt += 1;
    }
    else
    {
        *noMovementCnt = 0;
    }

    if (*noMovementCnt >= 32)
    {
        *(velocityArr + 1) = 0;
    }
    else
    {
        // Integrate to get velocity (scaled by 1024)
        *(velocityArr + 1) = *velocityArr + *scaledFilteredAccArr +
            divideRoundInt(
                (*(scaledFilteredAccArr + 1) - *scaledFilteredAccArr),
                2
            );
    }

    // Shift readings for acceleration and velocity (scaled by 1024)
    *scaledFilteredAccArr = *(scaledFilteredAccArr + 1);
    *velocityArr = *(velocityArr + 1);

    if (debugEnabled)
    {
        printString("Scaled Velocity 1: ");
        printVal32(*(velocityArr + 1));
        printString("\r\n");

        printString("Scaled Velocity 0: ");
        printVal32(*velocityArr);
        printString("\r\n");
    }
    
    // Normalize velocity
    *(velocityArr + 1) = divideRoundInt(*(velocityArr + 1), 1024);

    if (debugEnabled)
    {
        printString("Normalized Velocity 1: ");
        printVal32(*(velocityArr + 1));
        printString("\r\n");
    }

    return *(velocityArr + 1);
}

//=============================================================================

int32_t getRefYaw(
    int32_t *refYawArr, 
    int32_t *velocityArr, 
    int32_t scaledSteeringRad,
    int32_t *yawNum,
    int32_t *yawDen,
    uint8_t debugEnabled
)
{
    // Calculate model constants
    int32_t dYaw = divideRoundInt(*(refYawArr + 1) - *refYawArr, 2);
    int32_t K1 = 13;
    int16_t K2 = divideRoundInt(654, *(velocityArr + 1));
    int32_t K4 = divideRoundInt(8, *(velocityArr + 1));
    int32_t K5 =
        divideRoundInt(
            (186 + ((int32_t) 3 * *(velocityArr + 1) * *(velocityArr + 1))),
            (int32_t) *(velocityArr + 1) * *(velocityArr + 1)
        );

    *yawNum = (K1 * dYaw) + K2;
    *yawDen = (K4 * dYaw) + K5;

    // Get ref yaw rate based on steering angle
    int32_t currRefYaw = 
        divideRoundInt(
            divideRoundInt(*yawNum * scaledSteeringRad, *yawDen),
            1024
        );

    // Shift the yaw rates
    *refYawArr = *(refYawArr + 1);
    *(refYawArr + 1) = currRefYaw;

    if (debugEnabled)
    {
        printString("K1: ");
        printVal32(K1);
        printString("\r\n");

        printString("K2: ");
        printVal32(K2);
        printString("\r\n");

        printString("K4: ");
        printVal32(K4);
        printString("\r\n");

        printString("K5: ");
        printVal32(K5);
        printString("\r\n");

        printString("yawNum: ");
        printVal32(*yawNum);
        printString("\r\n");

        printString("yawDen: ");
        printVal32(*yawDen);
        printString("\r\n");

        printString("Current Ref Yaw: ");
        printVal32(currRefYaw);
        printString("\r\n");

        printString("refYaw[1]: ");
        printVal32(*(refYawArr + 1));
        printString("\r\n");

        printString("refYaw[0]: ");
        printVal32(*refYawArr);
        printString("\r\n");
    }

    return currRefYaw;
}

//=============================================================================

int16_t filterGyroYawLSB(int32_t sessionOffset, uint8_t debugEnabled)
{
    uint8_t filterYawCnt = 0;
    int32_t filterYawTotalLSB = 0;
    int16_t filteredYawLSB = 0;

    // Filter gyro yaw readings
    do {
        int8_t yawLSBHi = twiReadByte(MPU_ADDR, GYRO_ZOUT_H);
        int8_t yawLSBLo = twiReadByte(MPU_ADDR, GYRO_ZOUT_L);
        int16_t yawLSB = (yawLSBHi << 8) | yawLSBLo;

        filterYawTotalLSB += yawLSB;

        filterYawCnt++;

    } while (filterYawCnt != 64);

    filteredYawLSB = (filterYawTotalLSB >> 6);

    if (debugEnabled)
    {
        printString("Pre-filtered yaw LSB: ");
        printVal(filteredYawLSB);
        printString("\r\n");
    }

    // Apply calibration and offset filters
    filteredYawLSB -= (GYRO_Z_OFFSET + sessionOffset);

    if (debugEnabled)
    {
        printString("Calibration offset: ");
        printVal(GYRO_Z_OFFSET);
        printString("\r\n");
        printString("Session offset: ");
        printVal(sessionOffset);
        printString("\r\n");
        printString("Filtered gyro LSB: ");
        printVal(filteredYawLSB);
        printString("\r\n");
    }

    // Discrimination window for Z gyro
    if ((filteredYawLSB < 300) && (filteredYawLSB > -300))
    {
        filteredYawLSB = 0;
    }

    return filteredYawLSB;
}

//=============================================================================

int32_t getScaledYawInRad(int16_t filteredGyroYawLSB, uint8_t debugEnabled)
{
    return  divideRoundInt(
                divideRoundInt(
                    (int32_t) filteredGyroYawLSB * 1024,
                    GYRO_Z_SSF
                ) * 60,
                1024
            );
}

//=============================================================================

void printYawDebugInfo(int32_t gyroYaw, int8_t yawDiff, uint8_t debugEnabled)
{
    printString("Gyro Yaw: ");
    printVal32(gyroYaw);
    printString("\r\n");

    printString("Yaw Diff: ");
    printVal(yawDiff);
    printString("\r\n");

}

//=============================================================================

int16_t getCorrectedSteeringPwm(
    int32_t yawNum,
    int32_t yawDen,
    int8_t yawDiff,
    uint8_t debugEnabled
)
{
    int32_t steeringPwm = yawDen * yawDiff * 1024;

    steeringPwm =
        divideRoundInt(
            ((int32_t)divideRoundInt(steeringPwm, yawNum) * 60) - 50154,
            -33
        );

    if (steeringPwm > 2000)
    {
        steeringPwm = 2000;
    }

    if (steeringPwm < 1000)
    {
        steeringPwm = 1000;
    }

    if (debugEnabled)
    {
        printString("Corrected PWM: ");
        printVal32(steeringPwm);
        printString("\r\n\r\n");
    }

    return steeringPwm;
}

//=============================================================================
