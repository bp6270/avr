#ifndef MPU6050_SUPPORT_H_
#define MPU6050_SUPPORT_H_

#include "MPU_6050.h"
#include "twi.h"

/* Finds zero-point of sensor in static condition */
int16_t calibrate_sensor(
    uint8_t device_addr,
    uint8_t reg_addr_hi,
    uint8_t reg_addr_lo
);

/* Wrapper to calibrate both yaw and acc (x) sensors */
void calibrate_sensors(int16_t *acc_x_offset_addr, int16_t *yaw_offset_addr);

/* Basic low pass filter for acceleration reading */
int16_t read_avg_acc_lsb(int16_t acc_offset);

/* Converts a given acc LSB to micrometer/s^2) */
int32_t conv_acc_lsb_to_micrometer(int16_t lsb);

/* Basic low pass filter for yaw reading */
int16_t read_avg_yaw_lsb(int16_t yaw_offset);

/* Converts a given yaw LSB to millirads */
int16_t conv_yaw_lsb_to_millirad(int16_t lsb);

#endif
