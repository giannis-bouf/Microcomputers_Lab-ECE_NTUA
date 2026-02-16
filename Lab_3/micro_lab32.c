#define F_CPU 16000000UL
#include "avr/io.h"
#include <util/delay.h>

int main() {
    unsigned char DC_VALUE = 128;
    unsigned char values[] = {5, 26, 46, 66, 87, 107, 128, 148, 168, 189, 209, 230, 250}; //array of duty cycles
    unsigned int index = 6; //index at current duty cyle
    uint16_t adc_value1, adc_value2, adc_value; //adc conversion result
    float Vadc; //corresponding voltage
     
    // Set voltage reference to AVCC and select ADC1 channel
    ADMUX = (1 << REFS0) | (1 << MUX0);
    
    // Enable ADC and set prescaler to 128 for maximum resolution
    ADCSRA = (1 << ADEN) | (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);
    
     // initialize TMR1A in Fast PWM, 8-bit, non-inverting mode
    // and prescale = 8
    TCCR1A = (1<<WGM10) | (1<<COM1A1);
    TCCR1B = (1<<WGM12) | (1 << CS11);
    
    DDRD = 0xFF; //led output
    DDRB |= 0b00111111; //pwm output
    DDRC &= ~((1 << PC3) | (1 << PC4)); //input from PC3 and PC4

    PORTD = 0x00;
    while (1) {
        if (!(PINC & (1 << PINC3))) { //check if PC3 is pressed
         if (index < 12) {
             index++;
             DC_VALUE = values[index]; //increase duty cycle
         }
         _delay_ms(100);
        }
        else if (!(PINC & (1 << PINC4))) { //check if PC4 is pressed
            if (index > 0) {
             index--;
             DC_VALUE = values[index]; //decrease duty cycle
         }
        _delay_ms(100);
        }
        OCR1AL = DC_VALUE;
        
        //start ADC conversion
        ADCSRA |= (1 << ADSC);
    
        // Wait for conversion to complete
        while (ADCSRA & (1 << ADSC));
    
        // Combine ADCL and ADCH to get the 10-bit result
        adc_value1 = ADCL | (ADCH << 8); 
        
        //start ADC conversion
        ADCSRA |= (1 << ADSC);
    
        // Wait for conversion to complete
        while (ADCSRA & (1 << ADSC));
    
        // Combine ADCL and ADCH to get the 10-bit result
        adc_value2 = ADCL | (ADCH << 8); //ADCH? ADCL? ADCL | (ADCH << 8)?
        
        //get mean value of two different conversions for better accuracy
        adc_value = (adc_value1 + adc_value2)/2;
        
        Vadc = (adc_value*5.0)/1023.0; //scale the digital ADC to a voltage level
        
        // Display LED based on ADC value
        if (Vadc <= 0.625) {
             PORTD = (1 << PD0);
        } else if (Vadc <= 1.25) {
            PORTD = (1 << PD1);
        } else if (Vadc <= 1.875) {
            PORTD = (1 << PD2);
        } else if (Vadc <= 2.5) {
            PORTD = (1 << PD3);
        } else if (Vadc <= 3.125) {
            PORTD = (1 << PD4);
        } else if (Vadc <= 3.75) {
            PORTD = (1 << PD5);
        } else if (Vadc <= 4.375) {
            PORTD = (1 << PD6);
        } else {
            PORTD = (1 << PD7);
        }
        _delay_ms(100);
    }
    return 0;
}

