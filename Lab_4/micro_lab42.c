#define F_CPU 16000000UL
#include "avr/io.h"
#include <math.h>
#include <stdio.h>
#include <string.h>
#include <util/delay.h>

// swap macro
uint8_t swap(uint8_t x) {
    return (uint8_t)((x >> 4) | (x << 4));
}

void write_2_nibbles(uint8_t command) {
    // write MSB nibble
    PORTD = ((PIND & 0X0f) | (command & 0xf0)); 
    
    PORTD |= (1 << PD3);  // Enable pulse
    _delay_us(1);
    PORTD &= ~(1 << PD3); // Disable the enable pulse
    
    // write LSB nibble
    PORTD = ((PIND & 0X0f) | (swap(command) & 0xf0));
    
    PORTD |= (1 << PD3);
    _delay_us(1);
    PORTD &= ~(1 << PD3);
}

// send a command to lcd's controller (4-bit mode)
void lcd_command(uint8_t command) {
    PORTD &= ~(1 << PD2);  // LCD_RS = 0 => instruction
    write_2_nibbles(command);  // send instruction
    _delay_us(250);  // wait 250us
}

// send data to lcd's controller (4-bit mode)
void lcd_data(uint8_t data) {
    PORTD |= (1 << PD2);  // LCD_RS = 1 => data
    write_2_nibbles(data);  // send instruction
    _delay_us(250);  // wait 250us
}

void lcd_clear_display() {
    lcd_command(0x01);
    _delay_ms(5);
}

// lcd monitor set-up
void lcd_init() {
    _delay_ms(200);  // wait 200ms
    
    // set microcontroler in 4-bit mode
    
    for (int i=0; i<3; i++) {  // 3 times
        PORTD = 0x30;  // switch to 8-bit mode
        PORTD |= (1 << PD3);  // Enable pulse
        _delay_us(1);
        PORTD &= ~(1 << PD3); // Disable the enable pulse
        _delay_us(250);
    }
    
    PORTD = 0x20;  // switch to 4-bit mode
    PORTD |= (1 << PD3);  // Enable pulse
    _delay_us(1);
    PORTD &= ~(1 << PD3); // Disable the enable pulse
    _delay_us(250);
    
    lcd_command(0x28);   // 5x8 dots, 2 lines
    lcd_command(0x0c);   // display on, cursor-blinking off
    lcd_clear_display(); // clear display
    lcd_command(0x06);   // increase address, no display shift
}

int main() {
    char Vdisp[4];
    uint16_t adc_value;
    float Vin;
     
    // set voltage reference to AVCC and select ADC2 channel (0010)
    ADMUX = (1 << REFS0) | (1 << MUX1);
    // enable ADC and set prescaler to 128 for maximum resolution
    ADCSRA = (1 << ADEN) | (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);
    
    // initialize PORTD as output
    DDRD = 0xff; 
    
    lcd_init();
    _delay_ms(100);

    while (1) {
        lcd_clear_display();
        
        // start an ADC conversion and wait for it to finish
        // if ADSC is set, a conversion is in progress
        ADCSRA |= (1 << ADSC);
        while (ADCSRA & (1 << ADSC));
        // collect the conversion's 10-bit result
        adc_value = ADC;
        
        // scale the digital ADC to a voltage level with 2 decimal places
        Vin = (adc_value*5.0)/1023.0;
        
        // extracting individual digits
        int digit1 = (int)Vin;
        int digit2 = (int)((Vin - digit1) * 10);
        int digit3 = (int)((Vin * 100) - (digit1 * 100 + digit2 * 10));

        // convert digits to characters and store in the array
        Vdisp[0] = '0' + digit1;
        Vdisp[1] = '.';
        Vdisp[2] = '0' + digit2;
        Vdisp[3] = '0' + digit3;
    
        // display the result character-by-character
        for (int i = 0; i < 4; i++) {
            lcd_data((uint8_t)(Vdisp[i]));
        }
        // wait for the next second
        _delay_ms(1000);
    }
    return 0;
}