#include "USART.h"

void initUSART(void)
{
    // Set BAUD rate (must have macro defined)
    UBRR0H = UBRRH_VALUE;
    UBRR0L = UBRRL_VALUE;
    // Attempt to use 2X baud rate if within 2% tolerance
#if USE_2X
    UCSR0A |= (1 << U2X0);
#else
    UCSR0A &= ~(1 << U2X0);
#endif

    // Enable USART Tx with 8 data bits/1 stop bit
    UCSR0B = (1 << TXEN0) | (1 << RXEN0);
    UCSR0C = (1 << UCSZ01) | (1 << UCSZ00);
}

uint8_t getByte(void)
{
    loop_until_bit_is_set(UCSR0A, RXC0);
    return UDR0;
}

void sendByte(uint8_t byte)
{
    loop_until_bit_is_set(UCSR0A, UDRE0);
    UDR0 = byte;
}

void printString(const char myString[]) 
{
  uint8_t i = 0;
  
  while (myString[i]) 
  {
    sendByte(myString[i]);
    i++;
  }
}

void printWordAsDecimal(uint16_t word)
{
    sendByte('0' + (word / 10000));
    sendByte('0' + ((word / 1000) % 10));
    sendByte('0' + ((word / 100) % 10));
    sendByte('0' + ((word / 10) % 10));
    sendByte('0' + (word % 10));
}
void printVal(int16_t val)
{
    if (val < 0)
    {   
        sendByte('-');
        val = val * -1; 
    }   

    sendByte('0' + ((val / 10000) % 10));
    sendByte('0' + ((val / 1000) % 10));
    sendByte('0' + ((val / 100) % 10));
    sendByte('0' + ((val / 10) % 10));
    sendByte('0' + (val % 10));
}


void printVal32(int32_t val)
{
    if (val < 0)
    {   
        sendByte('-');
        val = val * -1; 
    }   

    sendByte('0' + ((val / 1000000000) % 10));
    sendByte('0' + ((val / 100000000) % 10));
    sendByte('0' + ((val / 10000000) % 10));
    sendByte('0' + ((val / 1000000) % 10));
    sendByte('0' + ((val / 100000) % 10));
    sendByte('0' + ((val / 10000) % 10));
    sendByte('0' + ((val / 1000) % 10));
    sendByte('0' + ((val / 100) % 10));
    sendByte('0' + ((val / 10) % 10));
    sendByte('0' + (val % 10));
}

uint16_t getNumber(void)
{
    char thousands = '0';
    char hundreds = '0';
    char tens = '0';
    char ones = '0';
    char currentChar = '0';

    do
    {
        thousands = hundreds;
        hundreds = tens;
        tens = ones;
        ones = currentChar;
        currentChar = getByte();
        sendByte(currentChar);
        
    } while (currentChar != '\r');

    sendByte('\n');

    return (1000 * (thousands - '0') +
            100 * (hundreds - '0') +
            10 * (tens - '0') +
            (ones - '0'));
}
