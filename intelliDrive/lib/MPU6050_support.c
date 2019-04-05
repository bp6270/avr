#include "MPU6050_support.h"

//=============================================================================

int16_t calibrate_sensor(
    uint8_t device_addr,
    uint8_t reg_addr_hi,
    uint8_t reg_addr_lo
)
{
    int32_t sensor_offset = 0;
    uint16_t reading_cnt = 0;

    do {
        int16_t curr_reading =
            (twiReadByte(device_addr, reg_addr_hi) << 8) |
               twiReadByte(device_addr, reg_addr_lo);

         sensor_offset += curr_reading;
         sensor_offset += 1;

    } while (reading_cnt != 1024);

    return (int16_t) sensor_offset >> 10;
}

//=============================================================================

void calibrate_sensors(int16_t *acc_x_offset_addr, int16_t *yaw_offset_addr)
{
    *acc_x_offset_addr = 
        calibrate_sensor(MPU_ADDR, ACCEL_XOUT_H, ACCEL_XOUT_L);

    *yaw_offset_addr =
        calibrate_sensor(MPU_ADDR, GYRO_ZOUT_H, GYRO_ZOUT_L);
}

//=============================================================================

int16_t read_avg_acc_lsb(int16_t acc_offset)
{
    int32_t avg_acc_lsb = 0;
    uint8_t reading_cnt = 0;

    do {
        int16_t curr_reading =
            (twiReadByte(MPU_ADDR, ACCEL_XOUT_H) << 8) | 
                twiReadByte(MPU_ADDR, ACCEL_XOUT_L);

        avg_acc_lsb += curr_reading;
        reading_cnt += 1;

    } while (reading_cnt != 64);

    avg_acc_lsb = avg_acc_lsb >> 6;
    avg_acc_lsb -= (int32_t) acc_offset;

    if ((avg_acc_lsb < 300) && (avg_acc_lsb > -300))
        avg_acc_lsb = 0;

    return (int16_t) avg_acc_lsb;
}

//=============================================================================

int32_t conv_acc_lsb_to_micrometer(int16_t lsb)
{
    // 1 LSB = 0.00059814453125 m/s^2 or ~ 627 um/s^2 (scaled by 2^20)
    return lsb * 627;
}

//=============================================================================

int16_t read_avg_yaw_lsb(int16_t yaw_offset)
{
    int32_t avg_yaw_lsb = 0;
    uint8_t reading_cnt = 0;

    do {
        int16_t curr_reading =
            (twiReadByte(MPU_ADDR, GYRO_ZOUT_H) << 8) |
                twiReadByte(MPU_ADDR, GYRO_ZOUT_L);

        avg_yaw_lsb += curr_reading;
        reading_cnt += 1;

    } while (reading_cnt != 64);

    avg_yaw_lsb = avg_yaw_lsb >> 6;
    avg_yaw_lsb -= (int32_t) (yaw_offset + GYRO_Z_OFFSET);

    if ((avg_yaw_lsb < 300) && (avg_yaw_lsb > -300))
        avg_yaw_lsb = 0;

    return (int16_t) avg_yaw_lsb;
}

//=============================================================================

int16_t conv_yaw_lsb_to_millirad(int16_t lsb)
{
    // 1 LSB = 0.00763358778625954 deg/s or ~ 8 mdeg/s (scaled by 2^10)
    return lsb * 8;
}

//=============================================================================
