#define F_CPU 16000000UL
#include "avr/io.h"
#include <math.h>
#include <stdio.h>
#include <string.h>
#include <util/delay.h>

// messages to be displayed
char gas_msg[] = {'G', 'A', 'S', ' ', 'D', 'E', 'T', 'E', 'C', 'T', 'E', 'D'};
char clr_msg[] = {'C', 'L', 'E', 'A', 'R'};

// swap macro
uint8_t swap(uint8_t x) {
    return (uint8_t)((x >> 4) | (x << 4));
}

void write_2_nibbles(uint8_t command) {
    // write MSB nibble
    PORTD = ((PIND & 0X0f) | (command & 0xf0)); 
    
    PORTD |= (1 << PD3);  // enable pulse
    _delay_us(1);
    PORTD &= ~(1 << PD3); // disable the enable pulse
    
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

// display the messages
void gas_detected() {
    for (int i=0; i<12; i++) {
        lcd_data((uint8_t)gas_msg[i]);
    }
}

void gas_clear() {
    for (int i=0; i<5; i++) {
        lcd_data((uint8_t)clr_msg[i]);
    }
}

int main() {
    uint16_t adc_value;
    float CO;
    unsigned int normal_level = 0;
    unsigned int first_det=0, first_clr=0; 
     
    // set voltage reference to AVCC and select ADC2 channel (0010)
    ADMUX = (1 << REFS0) | (1 << MUX1);
    // enable ADC and set prescaler to 128 for maximum resolution
    ADCSRA = (1 << ADEN) | (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);
    
    // initialize PORTD and PORTB as output
    DDRD = 0xff; 
    DDRB = 0xff; 
    
    // initialize PORTB
    PORTB = 0;
    
    // set up lcd and wait 100ms
    lcd_init();
    _delay_ms(100);

    while (1) {
        // start an ADC conversion and wait for it to finish
        // if ADSC is set, a conversion is in progress
        ADCSRA |= (1 << ADSC);
        while (ADCSRA & (1 << ADSC));
        // collect the conversion's 10-bit result
        adc_value = ADC;
        
        // scale the digital ADC to a voltage level with 2 decimal places
        CO = (adc_value*90.0)/1023.0;
        // 70ppm is a intermediate to high value considering CO level
        // so we set 90ppm as max value
        
        // turn on the corresponding LEDs
        if (CO <= 15) { PORTB = 0b1; }
        else if (CO <= 30) { PORTB = 0b11; }
        else if (CO <= 45) { PORTB = 0b111; }
        else if (CO <= 60) { PORTB = 0b1111; }
        else if (CO <= 75) { PORTB = 0b11111; }
        else { PORTB = 0b111111; }
        
        // wait 50ms
        _delay_ms(50);
      
        
        // if levels are high
        if (CO >= 70) { 
            // turn off and on the LEDs every 50ms - 'blinking'
            PORTB = 0; 
            // display "GAS DETECTED" message on lcd
            if (first_det == 0) {
                lcd_clear_display();
                gas_detected(); normal_level = 10;
                // don't display the message again
                first_det = 1;
            }
        }
        // when CO levels come back to normal
        // for the next 10 iterations of while-loop (10*(~100ms) = 1s)
        // display "CLEAR" message on lcd
        else if (normal_level > 0) {
            first_det = 0;
            if (first_clr == 0) {
                lcd_clear_display();
                gas_clear();
                // don't display the message again
                first_clr = 1;
            }
            normal_level--; 
        }
        // else clear lcd
        else {
            lcd_clear_display();
            first_clr = 0;
        }
        
        // wait another 50ms (total of 100ms)
        _delay_ms(50);
        
    }
    return 0;
}