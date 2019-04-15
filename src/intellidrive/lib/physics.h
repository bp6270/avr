#ifndef PHYSICS_H_
#define PHYSICS_H_

#include "MPU6050_support.h"
#include "USART.h"

/* Division operator that rounds up result */
int64_t divide_round_up(int64_t dividend, int64_t divisor);

/* Generates velocities (in micrometers) given a set of accelerations */
void gen_velocities_micro(
    int32_t *vel_x_micro,
    int32_t *acc_x_micro,
    int16_t acc_offset,
    uint16_t t_ms
);

/* Generates velocities (using floating point arith) */
void gen_velocities(float *vel_x, float *acc_x, int16_t acc_offset, float t);

/* Converts micrometer/second to meters/second value */
int16_t normalize_curr_velocity(int32_t vel_x_micro);

/* Generates millyaw model ref value for a given velocity and steering input */
void gen_ref_yaw_milli(
    int16_t *ref_yaw_milli,
    int32_t *vel_x_micro,
    int16_t *steer_rad_milli,
    int64_t *tf_num,
    int64_t *tf_den,
    int8_t t_ms
);

/* Generates model ref value using floating point operations */
void gen_ref_yaw(
    float *ref_yaw, 
    float *vel_x, 
    float *steer_rad,
    float *tf_num,
    float *tf_den,
    float t
);

#endif /* PHYSICS_H_ */
