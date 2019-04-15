#include "pid.h"

uint16_t compute_corrected_yaw(
    float *ref_yaw, 
    uint16_t pwm_duration,
    int16_t *yaw_offset, 
    float t,
    float tf_num,
    float tf_den
)
{
    static float last_error = 0.0;
    static float error_sum = 0.0;
    

    if ((tf_den > 0.0) || (tf_den < 0.0))
    {
        // PID parameters obtained through matlab pidtuner
        float kp = 0.12840000;
        float ki = 1.57200000;
        float kd = 0.00126800;
        
        // Get sensor yaw
        float imu_yaw = conv_yaw_to_rad(read_avg_yaw_lsb(*yaw_offset));
        
        //print_string("IMU yaw: ");
        //print_val32((int32_t) imu_yaw);
        //print_string("\r\n");

        // Get error between ref and sensor
        float error = *(ref_yaw + 3) - imu_yaw;
        
        //print_string("Yaw Error: ");
        //print_val32((int32_t) error);
        //print_string("\r\n");
        
        float d_err = (error - last_error) / t;

        error_sum += (error * t);
        last_error = error;
        
        float pid_out =  (kp * error) + (ki * error_sum) + (kd * d_err);

        //print_string("PID Out: ");
        //print_val32((int32_t) pid_out);
        //print_string("\r\n");
        
        float corrected_pwm  = steering_rad_to_pwm(pid_out, tf_num, tf_den);

        return corrected_pwm;
    }
    else
        return pwm_duration;

}
