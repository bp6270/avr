/* pwm.h */
#ifndef PWM_H_
#define PWM_H_

#include <avr/io.h>
#include <avr/interrupt.h>

#include "physics.h"

#define STEERING_FLAG 0x01

volatile static int16_t shared_steering_duration = 0;
volatile static uint8_t shared_tx_flags = 0x00;
static uint16_t steering_start_time = 0;
static uint16_t steering_duration = 0;

/* Checks if PWM duration is available and logs result */
uint8_t is_pwm_signal_available(void);

/* Retrieve last logged PWM duration */
uint16_t get_pwm_signal_duration(void);

/* Converts PWM duration to equivalent millirad angle */
int16_t pwm_to_steering_rad_milli(uint16_t steering_duration);

/* Converts PWM duration to rad in floating point units */
float pwm_to_steering_rad(uint16_t steering_duration);

/* Converts millirad angle to bounded PWM duration */
uint16_t steering_rad_milli_to_pwm(int16_t steering_rad_milli);

/* Converts rad to bounded PWM duration using floating point */
uint16_t steering_rad_to_pwm(float steering_rad);

#endif /* PWM_H_ */
