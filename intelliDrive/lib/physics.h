#ifndef PHYSICS_H_
#define PHYSICS_H_

#include "MPU6050_support.h"

/* Division operator that rounds up result */
int64_t divideRoundInt(int64_t dividend, int64_t divisor);

/* Generates velocities (in micrometers) given a set of accelerations */
void gen_velocities_micro(
    int32_t *vel_x_micro,
    int32_t *acc_x_micro,
    int16_t acc_offset,
    uint16_t t_ms
);

/* Converts micrometer/second to meters/second value */
int16_t normalize_curr_velocity(int32_t vel_x_micro);

/* Generates millyaw model ref value for a given velocity and steering input */
void gen_ref_yaw_milli(
    int16_t *ref_yaw_milli,
    int32_t *vel_x_micro,
    int16_t *steer_rad_milli,
    int64_t *tf_num,
    int64_t *tf_den
);

#endif /* PHYSICS_H_ */
