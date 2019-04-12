#include "config.h"

//=============================================================================

void enable_timer(void)
{
    // Enable 8-bit timer to operate at 125kHz (2ms/overflow)
    TCCR0B |= (1 << CS01) | (1 << CS00);
	
	// Make sure interrupt is fired after overflow
	TIMSK0 = (1 << TOIE0);
}

//=============================================================================

void enable_pwm_input_sampling(void)
{
    PCICR |= (1 << PCIE2);
    PCMSK2 |= (1 << PCINT23);
    sei();
}

//=============================================================================

void enable_pwm_steering_output(void)
{
    //  Enable 16-bit timer to operate in Fast PWM mode @ 1MHz
    TCCR1A |= (1 << WGM11);
    TCCR1B |= (1 << WGM12) | (1 << WGM13);
    TCCR1B |= (1 << CS11);
    ICR1 = PWM_DURATION;

    // Enable output for corrected PWM
    TCCR1A |= (1 << COM1A1);
    DDRB |= (1 << PB1);
}

//=============================================================================

void enable_IMU(void)
{
    // Disable pull-ups for TWI end TWI-INT
    PORTC &= ~((1 << PC4) | (1 << PC5));

    init_twi();

    // Config IMU and wake up
    twi_send_byte(MPU_ADDR, GYRO_CONFIG, GYRO_CONFIG_DEFAULT);
    twi_send_byte(MPU_ADDR, ACCEL_CONFIG, ACCEL_CONFIG_DEFAULT);
    twi_send_byte(MPU_ADDR, PWR_MGMT_1, PWR_MGMT_1_CONFIG_DEFAULT);
}

//=============================================================================

void bootstrap_hardware(void)
{
    clock_prescale_set(clock_div_1);
    init_USART();
    enable_timer();
    enable_pwm_input_sampling();
    enable_pwm_steering_output();
    enable_IMU();
}

//=============================================================================
