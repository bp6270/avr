#ifndef PID_H_
#define PID_H_

#include <avr/io.h>

#include "MPU6050_support.h"

/* PID function */
float compute_corrected_yaw(float *ref_yaw, int16_t *yaw_offset, float t);

#endif /* PID_H_ */
