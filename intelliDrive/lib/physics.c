#include "physics.h"

//=============================================================================

int64_t divide_round_up(int64_t dividend, int64_t divisor)
{
    int64_t half_divisor = divisor / 2;

    if (dividend < 0)
    {
        half_divisor *= -1;
    }

    return (dividend + half_divisor) / divisor;
}

//=============================================================================

void gen_velocities_micro(
    int32_t *vel_x_micro,
    int32_t *acc_x_micro,
    int16_t acc_offset,
    uint16_t t_ms
)
{
    static uint8_t no_movement_cnt = 0;

    if (*acc_x_micro == 0)
        no_movement_cnt += 1;
    else
        no_movement_cnt = 0;

    if (no_movement_cnt >= 32)
        *(vel_x_micro + 1) = 0;
    else
    {
        *acc_x_micro = conv_acc_lsb_to_micrometer(read_avg_acc_lsb(acc_offset));
        *(vel_x_micro + 1) = *vel_x_micro + divide_round_up(*acc_x_micro, t_ms);
    }

    *vel_x_micro = *(vel_x_micro + 1);
}

//=============================================================================

int16_t normalize_curr_velocity(int32_t vel_x_micro)
{
    return vel_x_micro >> 20;
}

//=============================================================================

void gen_ref_yaw_milli(
    int16_t *ref_yaw_milli, 
    int32_t *vel_x_micro, 
    int16_t *steer_rad_milli,
    int64_t *tf_num,
    int64_t *tf_den
)
{
    // K1= 13.2727, K2= 642.7287/U, K3= 0.0799, K4= 7.6971/U, K5= 15.5343
    // K6= 2.4490 -- these are scaled by 2^7 to make K3 non-zero
    uint16_t K1 = 1699;
    uint32_t K2 = divide_round_up(
                    82269,
                    normalize_curr_velocity(*(vel_x_micro + 1))
                  );
    uint8_t K3 = 10;
    uint16_t K4 = divide_round_up(
                    985,
                    normalize_curr_velocity(*(vel_x_micro + 1))
                  );
    uint16_t K5 = divide_round_up(
                    1988,
                    normalize_curr_velocity(*(vel_x_micro + 1))
                  );
    uint16_t K6 = 313;
    int8_t t = 100; 
    int32_t s = ((int32_t) *(ref_yaw_milli + 2) - *(ref_yaw_milli + 1)) * t;
    int32_t s2 = 
        ((int32_t) *(ref_yaw_milli + 2) - 
            2 * *(ref_yaw_milli + 1) + 
            *ref_yaw_milli) * (t*t);

    *(tf_num) = (int64_t) (K1*s + K2);
    *(tf_den) = (int64_t) (K3*s2 + K4*s + K5 + K6);
    *(ref_yaw_milli + 3) = divide_round_up(*tf_num * *steer_rad_milli, *tf_den);
    *ref_yaw_milli = *(ref_yaw_milli + 1);
    *(ref_yaw_milli + 2) = *(ref_yaw_milli + 3);
}

//=============================================================================

