#include "timer_support.h"

ISR(TIMER0_OVF_vect)
{
    shared_ovr_cnt += 1;
}

void reset_timer(void)
{
    TCNT0 = 0x00;
}

void reset_ovr_cnt(void)
{
    cli();
    shared_ovr_cnt = 0;
    sei();
}

void reset_ovr_cnt_and_timer(void)
{
    reset_ovr_cnt();
    reset_timer();
}

uint16_t get_ovr_cnt(void)
{
    cli();
    uint16_t ovr_cnt = shared_ovr_cnt;
    sei();
    
    return ovr_cnt;
}

void set_timer(uint8_t timer_val)
{
    TCNT0 = timer_val;
}
