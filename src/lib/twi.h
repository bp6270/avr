/**
 * Simple lib for i2c communication
 *
 */
#ifndef TWI_H_
#define TWI_H_

#include <avr/io.h>

// Init bus speed
void init_twi(void);

// Wait until TWI communication ready
void twi_wait_until_ready(void);

// Sends start signal
void twi_start(void);

// Sends stop signal
void twi_stop(void);

// Send out 8-bit data
void twi_send(uint8_t data);

// Read from slave with ACK
uint8_t twi_read_with_ack(void);

// Read from slave with NACK
uint8_t twi_read_with_nack(void);

// Send byte to register in TWI device
void twi_send_byte(uint8_t twi_addr, uint8_t reg_addr, uint8_t byte);

// Read byte from register in TWI device
uint8_t twi_read_byte(uint8_t twi_addr, uint8_t reg_addr);

#endif /* TWI_H_ */
