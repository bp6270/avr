/* support.h */
#ifndef SUPPORT_H_
#define SUPPORT_H_

#include <avr/io.h>
#include <avr/interrupt.h>

#include "MPU_6050.h"
#include "twi.h"
#include "USART.h"

/* Used to round up after division on integers */
int32_t divideRoundInt(int32_t dividend, int32_t divisor);

/* Grab the session offset for MPU_6050 sensor */
int32_t getSessionOffset(
    uint8_t devAddr,
    uint8_t regAddrHi,
    uint8_t regAddrLo,
    uint8_t debugEnabled
);

/* Wrapper to get acceleration offset */
int32_t getAccelOffset(uint8_t debugEnabled);

/* Wrapper to get gyro yaw offset */
int32_t getGyroYawOffset(uint8_t debugEnabled);

/* Wrapper to set accel and gyro offset */
void setIMUOffsets(int32_t *accOs, int32_t *yawOs, uint8_t debugEnabled);

/* Converts PWM duration of steering input to scaled radian value */
int32_t pwmToRadSc(uint16_t steeringDuration, uint8_t debugEnabled);

/* Returns filtered LSB for acceleration readings (with offset factored) */
int16_t filterAccelLSB(int32_t sessionOffset, uint8_t debugEnabled);

/* Converts LSB to meters/sec^2 then scales it by 1024 */
int32_t scaleFilteredAccel(int16_t filteredAccelLSB, uint8_t debugEnabled);

/* Generates a normalized (non-scaled) velocity value */
int32_t getCurrNormalizedVelocity(
    int32_t *velocityArr,
    int32_t *scaledFilteredAccArr,
    uint8_t *noMovementCnt,
    uint8_t debugEnabled
);

/* Generates the reference yaw given a steering and velocity input */
int32_t getRefYaw(
    int32_t *refYawArr,
    int32_t *velocityArr,
    int32_t scaledSteeringRad,
    int32_t *yawNum,
    int32_t *yawDen,
    uint8_t debugEnabled
);

/* Returns filtered LSB for gyro readings (with offset factored) */
int16_t filterGyroYawLSB(int32_t sessionOffset, uint8_t debugEnabled);

/* Converts gyro yaw LSB to normalized radian value */
int32_t getScaledYawInRad(int16_t filteredGyroYawLSB, uint8_t debugEnabled);

/* Debug function to print IMU yaw reading and difference with ref yaw */
void printYawDebugInfo(int32_t gyroYaw, int8_t yawDiff, uint8_t debugEnabled);

/* Given yaw readings, returns corrected PWM steering duration */
int16_t getCorrectedSteeringPwm(
    int32_t yawNum,
    int32_t yawDen,
    int8_t yawDiff,
    uint8_t debugEnabled
);

#endif /* SUPPORT_H_ */
