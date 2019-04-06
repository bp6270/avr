#ifndef INTELLIDRIVE_H_
#define INTELLIDRIVE_H_

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

#include "config.h"
#include "pid.h"
#include "pwm.h"
#include "timer_support.h"

#define VEL_SAMPLE_MS 500
#define YAW_SAMPLE_MS 100
#define TIMEOUT_US 200

#endif /* INTELLIDRIVE_H_ */
