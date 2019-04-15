#include "pwm.h"

//==============================================================================

ISR(PCINT2_vect)
{
    static uint16_t steering_start_time = 0;

    // If pin change is L->H, record start of pulse
    if (bit_is_set(PIND, PD7))
    {
        steering_start_time = TCNT1;
    }
    else
    {
        // Do not log if start sampling is 0 (never sampled)
        if (steering_start_time != 0)
        {
            shared_steering_duration = TCNT1 - steering_start_time;
        }

        // If duration is negative, counter has overflowed so compenstate
        if (shared_steering_duration < 0)
        {
            shared_steering_duration += 20000;
            shared_tx_flags |= STEERING_FLAG;
        }
    }
}

//==============================================================================

uint8_t is_pwm_signal_available(void)
{
    if (shared_tx_flags)
    {   
        cli();
        shared_tx_flags = 0x00;
        sei();

        return 1;
    }
    else
        return 0;
}

//==============================================================================

uint16_t get_pwm_signal_duration(void)
{
    cli();
    uint16_t steering_duration = shared_steering_duration;
    sei();
    
    //Debug
    print_string("PWM In: ");
    print_val(steering_duration);
    print_string("\r\n");

    return steering_duration;
}

//==============================================================================

int16_t pwm_to_steering_rad_milli(uint16_t steering_duration)
{
    // slope = -0.0326, int = 50.8788 err = -1.9
    // we must scale by 1024 to accomodate slope
    // then rescale to get normalized degrees
    // 1 deg ~= 17.8722 millirads
    int32_t scaled_deg = (-33 * steering_duration) + 50154;
    int8_t deg = scaled_deg >> 10;

    return deg * 18;
}

//==============================================================================

float pwm_to_steering_rad(uint16_t steering_duration)
{
    float deg = (-0.03264 * steering_duration) + 48.9788;

    return deg * 0.01745;
}

//==============================================================================

uint16_t steering_rad_milli_to_pwm(int16_t steering_rad_milli)
{
    // see pwm_to_steering_rad_milli() on how to create inverse
    int8_t deg = divide_round_up(steering_rad_milli, 18);
    int32_t scaled_deg = deg << 10;
    
    uint16_t pwm_duration = divide_round_up(scaled_deg - 50154, -33);

    // bind to the acceptable PWM range
    if (pwm_duration < 1000)
        return 1000;

    if (pwm_duration > 2000)
        return 2000;

     return pwm_duration;
}

//==============================================================================

uint16_t steering_rad_to_pwm(float yaw_diff, float tf_num, float tf_den)
{
    float steering_rad = yaw_diff / (tf_num / tf_den);
    float deg = 57.2958 * steering_rad;
    int32_t pwm_out = (int32_t) ((deg - 48.9788) / -0.03264);

    return pwm_out;
}

//==============================================================================

void write_pwm(uint16_t pwm_duration)
{
    print_string("Wanted PWM Out: ");
    print_val(pwm_duration);
    print_string("\r\n");
    
    // bind to the acceptable PWM range
    if (pwm_duration < 1000)
        pwm_duration = 1000;

    if (pwm_duration > 2000)
        pwm_duration =  2000;

    OCR1A = pwm_duration;
    
    print_string("Actual PWM Out: ");
    print_val(pwm_duration);
    print_string("\r\n");
}

//==============================================================================
