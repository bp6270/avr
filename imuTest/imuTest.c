#include <avr/io.h>
#include <avr/power.h>
#include <util/delay.h>

#include "twi.h"
#include "USART.h"

// i2c device address
#define MPU_ADDR 0x68

// i2c registers
#define ACCEL_CONFIG 0x1C
#define ACCEL_XOUT_H 0x3B
#define ACCEL_XOUT_L 0x3C
#define ACCEL_YOUT_H 0x3D
#define ACCEL_YOUT_L 0x3E
#define ACCEL_ZOUT_H 0x3F
#define ACCEL_ZOUT_L 0x40

#define GYRO_CONFIG 0x1B
#define GYRO_XOUT_H 0x43
#define GYRO_XOUT_L 0x44
#define GYRO_YOUT_H 0x45
#define GYRO_YOUT_L 0x46
#define GYRO_ZOUT_H 0x47
#define GYRO_ZOUT_L 0x48

#define PWR_MGMT_1 0x6B

void printLSB(int16_t accel_cnt);

int main(void)
{
    // Run CPU at 8MHz
    clock_prescale_set(clock_div_1);

    // Config and enable USART
    initUSART();

    // Disable pull-ups for TWI and TWI-INT
    PORTB &= ~(1 << PB0);
    PORTC &= ~((1 << PC4) | (1 << PC5));

    // Config and enable TWI
    printString("Initializing TWI interface...\r\n");
    initTwi();

    // Config accelerometer
    printString("Configuring accelerometer...\r\n");
    twiSendByte(MPU_ADDR, ACCEL_CONFIG, 0x00);

    // Config gyroscope
    printString("Configuring gyroscope...\r\n");
    twiSendByte(MPU_ADDR, GYRO_CONFIG, 0x00);

    // Wake up the IMU (defaults to sleep mode on power up)
    printString("Waking up IMU!\r\n");
    twiSendByte(MPU_ADDR, PWR_MGMT_1, 0x00);
    
    printString("Taking readings...\r\n\n");

    while (1)
    {
        for (uint8_t i=0; i < 100; i++)
        {
            // Accelerometer readings
            /*
            uint8_t accel_x_h_cnt = twiReadByte(MPU_ADDR, ACCEL_XOUT_H);
            uint8_t accel_x_l_cnt = twiReadByte(MPU_ADDR, ACCEL_XOUT_L);
            int16_t accel_x = (accel_x_h_cnt << 8) | accel_x_l_cnt;
            
            uint8_t accel_y_h_cnt = twiReadByte(MPU_ADDR, ACCEL_YOUT_H);
            uint8_t accel_y_l_cnt = twiReadByte(MPU_ADDR, ACCEL_YOUT_L);
            int16_t accel_y = (accel_y_h_cnt << 8) | accel_y_l_cnt;
       
            uint8_t accel_z_h_cnt = twiReadByte(MPU_ADDR, ACCEL_ZOUT_H);
            uint8_t accel_z_l_cnt = twiReadByte(MPU_ADDR, ACCEL_ZOUT_L);
            int16_t accel_z = (accel_z_h_cnt << 8) | accel_z_l_cnt;
            printLSB(accel_x);
            printString(",");
            printLSB(accel_y);
            printString(",");
            printLSB(accel_z);
            printString("\r\n");
            */
            
            // Gyroscope readings
            uint8_t gyro_x_h_cnt = twiReadByte(MPU_ADDR, GYRO_XOUT_H);
            uint8_t gyro_x_l_cnt = twiReadByte(MPU_ADDR, GYRO_XOUT_L);
            int16_t gyro_x = (gyro_x_h_cnt << 8) | gyro_x_l_cnt;

            uint8_t gyro_y_h_cnt = twiReadByte(MPU_ADDR, GYRO_YOUT_H);
            uint8_t gyro_y_l_cnt = twiReadByte(MPU_ADDR, GYRO_YOUT_L);
            int16_t gyro_y = (gyro_y_h_cnt << 8) | gyro_y_l_cnt;

            uint8_t gyro_z_h_cnt = twiReadByte(MPU_ADDR, GYRO_ZOUT_H);
            uint8_t gyro_z_l_cnt = twiReadByte(MPU_ADDR, GYRO_ZOUT_L);
            int16_t gyro_z = (gyro_z_h_cnt << 8) | gyro_z_l_cnt;

            printLSB(gyro_x);
            printString(",");
            printLSB(gyro_y);
            printString(",");
            printLSB(gyro_z);
            printString("\r\n");

            _delay_ms(1000);
        }
        
        printString("\r\nReading Complete!");

        // Infinite loop after completion
        while (1) {}
    }


    return 0;
}

void printLSB(int16_t accel_cnt)
{
    // print negative sign if first bit is 1
    if (accel_cnt & (1 << 15))
    {
        sendByte('-');
        accel_cnt = accel_cnt * -1;
    }

    sendByte('0' + ((accel_cnt / 10000) % 10));
    sendByte('0' + ((accel_cnt / 1000) % 10));
    sendByte('0' + ((accel_cnt / 100) % 10));
    sendByte('0' + ((accel_cnt / 10) % 10));
    sendByte('0' + (accel_cnt % 10));
}

