#ifndef PID_H_
#define PID_H_

#include <avr/io.h>

#include "MPU6050_support.h"
#include "pwm.h"
#include "USART.h"

/* PID function */
uint16_t compute_corrected_yaw(
    float *ref_yaw, 
    uint16_t pwm_duration,
    int16_t *yaw_offset, 
    float t,
    float tf_num,
    float tf_den
);

#endif /* PID_H_ */
