#include "twi.h"

void initTwi(void)
{
    // TWI Frequency
    // F_CPU / [(16+2(TWBR)*(Prescaler Val)]
    // 8MHz / [16+2(32)*(1) = 100KHz
    TWBR = 32;

    // Enable TWI
    TWCR |= (1 << TWEN);
}

void twiWaitUntilReady(void)
{
    loop_until_bit_is_set(TWCR, TWINT);
}

void twiStart(void)
{
    TWCR = ((1 << TWINT) | (1 << TWEN) | (1 << TWSTA));
    twiWaitUntilReady();
}

void twiStop(void)
{
    TWCR = ((1 << TWINT) | (1 << TWEN) | (1 << TWSTO));
}

void twiSend(uint8_t data)
{
    TWDR = data;
    TWCR = ((1 << TWINT) | (1 << TWEN));
    twiWaitUntilReady();
}

uint8_t twiReadWithAck(void)
{
    TWCR = ((1 << TWINT) | (1 << TWEN) | (1 << TWEA));
    twiWaitUntilReady();

    return (TWDR);
}

uint8_t twiReadWithNack(void)
{
    TWCR = ((1 << TWINT) | (1 << TWEN));
    twiWaitUntilReady();

    return (TWDR);
}

void twiSendByte(uint8_t i2cAddr, uint8_t regAddr, uint8_t byte)
{
    twiStart();
    twiSend(i2cAddr << 1);
    twiSend(regAddr);
    twiSend(byte);
    twiStop();
}

uint8_t twiReadByte(uint8_t i2cAddr, uint8_t regAddr)
{
    twiStart();
    twiSend(i2cAddr << 1);
    twiSend(regAddr);
    twiStart();
    twiSend((i2cAddr << 1) | 0x01);
    uint8_t byte = twiReadWithNack();
    twiStop();

    return byte;
}
