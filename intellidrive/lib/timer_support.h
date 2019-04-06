/* This is used for AVR atmega328p timer0 
 *
 * Assumes that timer0 has been configured for normal operation
 *  
 */

#ifndef TIMER_SUPPORT_H_
#define TIMER_SUPPORT_H_

#include <avr/io.h>
#include <avr/interrupt.h>

static volatile uint16_t shared_ovr_cnt = 0;

void reset_timer(void);

void reset_ovr_cnt(void);

void reset_ovr_cnt_and_timer(void);

uint16_t get_ovr_cnt(void);

void set_timer(uint8_t timer_val);

#endif /* TIMER_SUPPORT_H_ */
