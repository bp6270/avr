#include <avr/io.h>
#include <avr/power.h>
#include <util/delay.h>

#include "USART.h"

# define PWM_PULSE_DURATION_MICRO 20000

static inline void initServo(void)
{
    // Fast PWM using ICR1 to carry TOP val
    TCCR1A |= (1 << WGM11);
    TCCR1B |= (1 << WGM12) | (1 << WGM13);

    // Pre-scaler set to CPU/8
    TCCR1B |= (1 << CS11);

    // Set ICR1 Val
    ICR1 = PWM_PULSE_DURATION_MICRO;

    // Clear output on compare match
    TCCR1A |= (1 << COM1A1);

    // Set OC1A/PB data direction to output
    DDRB = (1 << PB1);
}

int main(void)
{
    uint16_t pwmPulseLengthMicro;
    
    // Run CPU at 8MHz
    clock_prescale_set(clock_div_1);

    // Configure and enable PWM output
    initServo();

    // Configure and enable USART
    initUSART();

    // Greet
    printString("Welcome to PWM generation application! \r\n");

    while (1)
    {
        // Get input from user and set to output compare reg
        printString("\r\nEnter PWM pulse in microsecond:\r\n");
        pwmPulseLengthMicro = getNumber();
        OCR1A = pwmPulseLengthMicro;
        _delay_ms(3000);
    }

    return 0;
}
