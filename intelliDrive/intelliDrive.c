#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

#include "config.h"
#include "pwm.h"
#include "timer_support.h"

#define VEL_SAMPLE_MS 500
#define YAW_SAMPLE_MS 100
#define TIMEOUT_US 200

int main(void)
{
    int32_t vel_x_micro[2] = {0};
    int32_t acc_x_micro = 0;
    int16_t acc_x_offset = 0;
    int16_t ref_yaw_milli[4] = {0};
    int16_t yaw_offset = 0;
    int16_t steering_rad_milli = 0;
    int64_t tf_num = 0;
    int64_t tf_den = 0;
    int16_t timer_overhead = 0;

    bootstrapHardware();
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

        acc_x_micro = read_avg_acc_lsb(acc_x_offset);

        while ((get_ovr_cnt() + timer_overhead) < 1)
            _delay_us(TIMEOUT_US);
        
        reset_ovr_cnt_and_timer();
        gen_velocities_micro(
            vel_x_micro, 
            &acc_x_micro, 
            acc_x_offset, 
            VEL_SAMPLE_MS
        );
        timer_overhead = get_ovr_cnt();
        reset_ovr_cnt_and_timer();

        while ((get_ovr_cnt() + timer_overhead) < 4)
            _delay_us(TIMEOUT_US);
        
        reset_ovr_cnt_and_timer();
        steering_rad_milli = 
            pwm_to_steering_rad_milli(get_pwm_signal_duration());
        gen_ref_yaw_milli(
            ref_yaw_milli,
            vel_x_micro,
            &steering_rad_milli,
            &tf_num,
            &tf_den
        );
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

