/**
 * Simple lib for i2c communication
 *
 */
#ifndef TWI_H_
#define TWI_H_

#include <avr/io.h>

// Init bus speed
void initTwi(void);

// Wait until TWI communication ready
void twiWaitUntilReady(void);

// Sends start signal
void twiStart(void);

// Sends stop signal
void twiStop(void);

// Send out 8-bit data
void twiSend(uint8_t data);

// Read from slave with ACK
uint8_t twiReadWithAck(void);

// Read from slave with NACK
uint8_t twiReadWithNack(void);

// Send byte to register in TWI device
void twiSendByte(uint8_t i2cAddr, uint8_t regAddr, uint8_t byte);

// Read byte from register in TWI device
uint8_t twiReadByte(uint8_t i2cAddr, uint8_t regAddr);

#endif /* TWI_H_ */
