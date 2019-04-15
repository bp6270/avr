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
            (twi_read_byte(device_addr, reg_addr_hi) << 8) |
               twi_read_byte(device_addr, reg_addr_lo);
			   
         sensor_offset += curr_reading;
         reading_cnt += 1;

    } while (reading_cnt != 1024);
	
    return (int16_t) sensor_offset >> 10;
}

//=============================================================================

void calibrate_sensors(int16_t *acc_x_offset_addr, int16_t *yaw_offset_addr)
{
    *acc_x_offset_addr = 
        calibrate_sensor(MPU_ADDR, ACCEL_XOUT_H, ACCEL_XOUT_L);
    
    //print_string("Acc Offset: " );
    //print_val(*acc_x_offset_addr);
    //print_string("\r\n");

    *yaw_offset_addr =
        calibrate_sensor(MPU_ADDR, GYRO_ZOUT_H, GYRO_ZOUT_L);

    //print_string("Yaw Offset: " );
    //print_val(*yaw_offset_addr);
    //print_string("\r\n");
}

//=============================================================================

int16_t read_avg_acc_lsb(int16_t acc_offset)
{
    int32_t avg_acc_lsb = 0;
    uint8_t reading_cnt = 0;
	

    do {
        int16_t curr_reading =
            (twi_read_byte(MPU_ADDR, ACCEL_XOUT_H) << 8) | 
                twi_read_byte(MPU_ADDR, ACCEL_XOUT_L);
		
        avg_acc_lsb += curr_reading;
        reading_cnt += 1;

    } while (reading_cnt != 64);

    avg_acc_lsb = avg_acc_lsb >> 6;
    avg_acc_lsb -= (int32_t) acc_offset;
    //avg_acc_lsb += ACCEL_XOUT_OFFSET;
    avg_acc_lsb -= 500;

    if ((avg_acc_lsb < 500) && (avg_acc_lsb > -500))
        avg_acc_lsb = 0;
    
    //print_string("Acc LSB: ");
    //print_val(avg_acc_lsb);
    //print_string("\r\n");

    return (int16_t) avg_acc_lsb;
}

//=============================================================================

int32_t conv_acc_lsb_to_micrometer(int16_t lsb)
{
    // 1 LSB = 0.00059814453125 m/s^2 or ~ 627 um/s^2 (scaled by 2^20)
    return lsb * 627;
}

//=============================================================================

float conv_acc_lsb_to_meter(int16_t lsb)
{
    // 1 LSB = 0.00059814453125 m/s^2
    return lsb * 0.000598;
}

//=============================================================================

int16_t read_avg_yaw_lsb(int16_t yaw_offset)
{
    int32_t avg_yaw_lsb = 0;
    uint8_t reading_cnt = 0;

    do {
        int16_t curr_reading =
            (twi_read_byte(MPU_ADDR, GYRO_ZOUT_H) << 8) |
                twi_read_byte(MPU_ADDR, GYRO_ZOUT_L);
		
        avg_yaw_lsb += curr_reading;
        reading_cnt += 1;

    } while (reading_cnt != 64);

    avg_yaw_lsb = avg_yaw_lsb >> 6;
    avg_yaw_lsb -= (int32_t) (yaw_offset + GYRO_Z_OFFSET);
    avg_yaw_lsb += 700;

    if ((avg_yaw_lsb < 300) && (avg_yaw_lsb > -300))
        avg_yaw_lsb = 0;

    //print_string("Yaw LSB: ");
    //print_val(avg_yaw_lsb);
    //print_string("\r\n");
    
    return (int16_t) avg_yaw_lsb;
}

//=============================================================================

int16_t conv_yaw_lsb_to_millirad(int16_t lsb)
{
    // 1 LSB = 0.00763358778625954 deg/s or ~ 8 mdeg/s (scaled by 2^10)
    return lsb * 8;
}

//=============================================================================

float conv_yaw_to_rad(int16_t lsb)
{
    // lsb to degrees/sec
    float deg = lsb / 131;

    // 1 deg/sec = 
    return deg * 0.01745329;
}

//=============================================================================
