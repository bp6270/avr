#ifndef USART_H_
#define USART_H_

#ifndef BAUD
#define BAUD 9600
#endif

#include <avr/io.h>
#include <util/setbaud.h>

/* Configures hardware USART
 *
 * This requires F_CPU and BAUD macros to be set
 * in order to calculate the proper bit-clock
 * multiplier.
 *
 * */
void init_USART(void);

/* Receive 8-bit data to data register */
uint8_t receive_byte(void);

/* Transmits 8-bit data to data register */
void transmit_byte(uint8_t byte);

/* Used to transmit chars to data register */
void print_string(const char string[]);

/* Used to take a 16-bit word and print as decimal */
void print_word_as_decimal(uint16_t word);

/* Used to take a 16 bit signed number and print as decimal */
void print_val(int16_t val);

/* Used to take a 32 bit signed number and print as decimal */
void print_val32(int32_t val);

/* Converts 16 bit string to integer */
uint16_t get_number(void);

#endif /* USART_H_ */
