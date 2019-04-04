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
void initUSART(void);

/* Receive 8-bit data to data register */
uint8_t receiveByte(void);

/* Transmits 8-bit data to data register */
void sendByte(uint8_t byte);

/* Used to transmit chars to data register */
void printString(const char myString[]);

/* Used to take a 16-bit word and print as decimal */
void printWordAsDecimal(uint16_t);

/* Used to take a 16 bit signed number and print as decimal */
void printVal(int16_t val);

/* Used to take a 32 bit signed number and print as decimal */
void printVal32(int32_t val);

/* Converts 16 bit string to integer */
uint16_t getNumber(void);

#endif /* USART_H_ */
