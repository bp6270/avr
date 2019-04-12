#include "pid.h"

float compute_corrected_yaw(float *ref_yaw, int16_t *yaw_offset, float t)
{
    static float last_error = 0.0;
    static float error_sum = 0.0;

    float imu_yaw = conv_yaw_to_rad(read_avg_yaw_lsb(*yaw_offset));
    float error = *(ref_yaw + 3) - imu_yaw;
    float d_err = (error - last_error) / t;
    
    // PID parameters obtained through matlab pidtuner
    float kp = 0.12840000;
    float ki = 1.57200000;
    float kd = 0.00126800;

    error_sum += (error * t);
    last_error = error;

    return (kp * error) + (ki * error_sum) + (kd * d_err);
}
