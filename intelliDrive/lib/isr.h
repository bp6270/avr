/* isr.h */
#ifndef ISR_H_
#define ISR_H_

#include <avr/io.h>
#include <avr/interrupt.h>

#define STEERING_FLAG 0x01

volatile int16_t sharedSteeringDuration;
volatile uint8_t sharedTxFlags;
uint16_t steeringStart;

#endif /* ISR_H_ */
