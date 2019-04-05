#ifndef CONFIG_H_
#define CONFIG_H_

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/power.h>

#include "MPU_6050.h"
#include "twi.h"
#include "USART.h"

/* Configures timer used for sample time of velocity/ref yaw/PID */
void enableTimer(void);

/* Configures AVR interrupt for PWM signal input sampling */
void enablePWMInputSampling(void);

/* Configures AVR timer and output pins for steering PWM generation */
void enablePWMSteeringOutput(void);

/* Configures AVR pins, MPU_6050 accelerometer and gyro, then enables */
void enableIMU(void);

/* Wrapper to setup the hardware being used */
void bootstrapHardware(void);

#endif /* CONFIG_H_ */
