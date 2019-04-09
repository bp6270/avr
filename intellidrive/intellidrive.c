#include "intellidrive.h"

int main(void)
{
    float vel_x[2] = {0.0};
    float acc_x = 0.0;
    int16_t acc_x_offset = 0;
    float ref_yaw [4] = {0.0};
    int16_t yaw_offset = 0;
    float steering_rad = 0.0;
    float tf_num = 0.0;
    float tf_den = 0.0;
    int16_t timer_overhead = 0;

    bootstrap_hardware();
    calibrate_sensors(&acc_x_offset, &yaw_offset);
    reset_ovr_cnt_and_timer();

    while (1)
    { 
        timer_overhead = get_ovr_cnt();

        if (!is_pwm_signal_available())
        {
            reset_ovr_cnt_and_timer();
            timer_overhead = get_ovr_cnt();
            continue;
        }

        while ((get_ovr_cnt() + timer_overhead) < 1)
            _delay_us(TIMEOUT_US);
        
        reset_ovr_cnt_and_timer();
        gen_velocities(vel_x, &acc_x, acc_x_offset, 0.002);
        timer_overhead = get_ovr_cnt();
        reset_ovr_cnt_and_timer();

        while ((get_ovr_cnt() + timer_overhead) < 4)
            _delay_us(TIMEOUT_US);
        
        reset_ovr_cnt_and_timer();
        steering_rad = pwm_to_steering_rad(get_pwm_signal_duration());
        gen_ref_yaw(ref_yaw, vel_x, &steering_rad, &tf_num, &tf_den, 0.010);
        timer_overhead = get_ovr_cnt();
        reset_ovr_cnt_and_timer();

        while ((get_ovr_cnt() + timer_overhead) < 2)
            _delay_us(TIMEOUT_US);
            
        reset_ovr_cnt_and_timer();
        // PID
        timer_overhead = get_ovr_cnt();
    }

    return 0;
}

