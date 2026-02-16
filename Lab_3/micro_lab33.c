#define F_CPU 16000000UL
#include "avr/io.h"
#include <util/delay.h>

int main() {
    unsigned char DC_VALUE = 128;
    unsigned char values[] = {5, 26, 46, 66, 87, 107, 128, 148, 168, 189, 209, 230, 250}; //array of duty cycles
    unsigned int index = 6; //index at current duty cyle
    unsigned char mode = 0;
    uint16_t pot;
     
    // Set voltage reference to AVCC and select ADC0 channel (0000)
    ADMUX = (1 << REFS0);
    
    // Enable ADC and set prescaler to 128 for maximum resolution
    ADCSRA = (1 << ADEN) | (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);
    
    // initialize TMR1A in Fast PWM, 8-bit, non-inverting mode
    // and prescale = 8
    TCCR1A = (1<<WGM10) | (1<<COM1A1);
    TCCR1B = (1<<WGM12) | (1 << CS11);
    
    // initialize PORTB as output and PORTD as input
    DDRB |= 0b00111111;
    DDRD &= ~((1 << PD1) | (1 << PD2) | (1 << PD6) | (1 << PD7)); 

    while (1) {
        // if PD6 is pressed activate mode1
        if (!(PIND & (1 << PIND6))) mode = 1;
        // else if PD7 is pressed activate mode2
        else if (!(PIND & (1 << PIND7))) mode = 2;
        
        if (mode == 1) {
            if (!(PIND & (1 << PIND1))) { //check if PD1 is pressed
                if (index < 12) {
                    index++;
                    DC_VALUE = values[index]; //increase duty cycle
                }
                _delay_ms(200);
            }
            else if (!(PIND & (1 << PIND2))) { //check if PD2 is pressed
                if (index > 0) {
                    index--;
                    DC_VALUE = values[index]; //decrease duty cycle
                }
                _delay_ms(200);
            }
        }
        else if (mode == 2) {
            // start an ADC conversion and wait for it to finish
            // if ADSC is set, a conversion is in progress
            ADCSRA |= (1 << ADSC);
            while (ADCSRA & (1 << ADSC));
            // collect the conversion's 10-bit result
            pot = ADCL | (ADCH << 8);
            // and transform it to a Duty Cycle value for 8-bit Fast PWM
            DC_VALUE = (pot*255) / 1023;
        }
        //  the Duty Cycle value
        OCR1AL = DC_VALUE;
    }
    return 0;
}