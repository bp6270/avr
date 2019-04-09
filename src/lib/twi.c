#include "twi.h"

//==============================================================================

void init_twi(void)
{
    // TWI Frequency
    // F_CPU / [(16+2(TWBR)*(Prescaler Val)]
    // 8MHz / [16+2(2)*(1) = 400KHz
    TWBR = 2;

    // Enable TWI
    TWCR |= (1 << TWEN);
}

//==============================================================================

void twi_wait_until_ready(void)
{
    loop_until_bit_is_set(TWCR, TWINT);
}

//==============================================================================

void twi_start(void)
{
    TWCR = ((1 << TWINT) | (1 << TWEN) | (1 << TWSTA));
    twi_wait_until_ready();
}

//==============================================================================

void twi_stop(void)
{
    TWCR = ((1 << TWINT) | (1 << TWEN) | (1 << TWSTO));
}

//==============================================================================

void twi_send(uint8_t data)
{
    TWDR = data;
    TWCR = ((1 << TWINT) | (1 << TWEN));
    twi_wait_until_ready();
}

//==============================================================================

uint8_t twi_read_with_ack(void)
{
    TWCR = ((1 << TWINT) | (1 << TWEN) | (1 << TWEA));
    twi_wait_until_ready();

    return TWDR;
}

//==============================================================================

uint8_t twi_read_with_nack(void)
{
    TWCR = ((1 << TWINT) | (1 << TWEN));
    twi_wait_until_ready();

    return TWDR;
}

//==============================================================================

void twi_send_byte(uint8_t twi_addr, uint8_t reg_addr, uint8_t byte)
{
    twi_start();
    twi_send(twi_addr << 1);
    twi_send(reg_addr);
    twi_send(byte);
    twi_stop();
}

//==============================================================================

uint8_t twi_read_byte(uint8_t twi_addr, uint8_t reg_addr)
{
    twi_start();
    twi_send(twi_addr << 1);
    twi_send(reg_addr);
    twi_start();
    twi_send((twi_addr << 1) | 0x01);
    uint8_t byte = twi_read_with_nack();
    twi_stop();

    return byte;
}

//==============================================================================
