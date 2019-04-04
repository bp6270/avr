#include "config.h"

//=============================================================================

void enablePWMSteeringOutput(void)
{
    // Initialize 16-bit timer
    TCCR1A |= (1 << WGM11);
    TCCR1B |= (1 << WGM12) | (1 << WGM13);
    TCCR1B |= (1 << CS11);
    ICR1 = 20000;

    // Enable output for corrected PWM
    TCCR1A |= (1 << COM1A1);
    DDRB |= (1 << PB1);

    // Enable hardware interrupts for PWM sampling
    PCICR |= (1 << PCIE2);
    PCMSK2 |= (1 << PCINT23);
    sei();
}

//=============================================================================

void enableIMU(void)
{
    // Disable pull-ups for TWI end TWI-INT
    PORTC &= ~((1 << PC4) | (1 << PC5));

    initTwi();

    // Config IMU and wake up
    twiSendByte(MPU_ADDR, GYRO_CONFIG, 0x00);
    twiSendByte(MPU_ADDR, ACCEL_CONFIG, 0x00);
    twiSendByte(MPU_ADDR, PWR_MGMT_1, 0x00);
}

//=============================================================================

void bootstrapHardware(void)
{
    clock_prescale_set(clock_div_1);
    initUSART();
    enablePWMSteeringOutput();
    enableIMU();
}

//=============================================================================
