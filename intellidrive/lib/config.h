#ifndef CONFIG_H_
#define CONFIG_H_

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/power.h>

#include "MPU_6050.h"
#include "twi.h"
#include "USART.h"

#define PWM_DURATION 20000

/* Configures timer used for sample time of velocity/ref yaw/PID */
void enabletimer(void);

/* Configures AVR interrupt for PWM signal input sampling */
void enable_pwm_input_sampling(void);

/* Configures AVR timer and output pins for steering PWM generation */
void enable_pwm_steering_output(void);

/* Configures AVR pins, MPU_6050 accelerometer and gyro, then enables */
void enable_IMU(void);

/* Wrapper to setup the hardware being used */
void bootstrap_hardware(void);

#endif /* CONFIG_H_ */
