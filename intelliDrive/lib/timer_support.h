/* This is used for AVR atmega328p timer0 
 *
 * Assumes that timer0 has been configured for normal operation
 *  
 */

#ifndef TIMER_SUPPORT_H_
#define TIMER_SUPPORT_H_

#include <avr/io.h>

/* Checks if timer has oveflowed */
uint8_t isTimerOverFlowed(void);

/* Used to reset timer to 0 */
void resetTimer(void);

/* Used to clear overflow flag */
void clearOverflowFlag(void);

/* Used to reset timer to 0 and clear overflow flag */
void resetTimerAndOvr(void);

/* Sets timer to an unsigned 8 bit value */
void setTimer(uint8_t timerVal);

#endif /* TIMER_SUPPORT_H_ */
