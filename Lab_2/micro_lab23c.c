
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

// flag to check when INT1 is activated
volatile uint8_t buttonPressed = 0;

int main() {
    // PD3 as input 
    DDRD &= ~(1 << 3);
    PORTD |= (1 << 3);
  
    // INT1 on rising edge
    EICRA = (1 << ISC10) | (1 << ISC11);
  
    // enable INT1
    EIMSK |= (1 << INT1);
  
    // enable global interrupts
    sei();
    
    // PORTB as output
    DDRB = 0xFF;

    while (1) {
        // check if PD3 is pressed
    	if (buttonPressed) {
	   		buttonPressed = 0;
      
       		// Turn on LEDs for 0.5 sec
   	   		PORTB = 0xFF;
	   		for (int i=0; i<500; i++) {
				_delay_ms(16);
				// if PD3 is pressed
				if(buttonPressed) break;
			}
	   		// start all over again
	   		if(buttonPressed) continue;
  
    		// turn off the LEDs except PB0 for 3 sec
    		PORTB = 0x01;
	   		for (int i=0; i<3000; i++) {
				_delay_ms(16);
				if(buttonPressed) break;
			}
			// turn off PB0 too
        	PORTB = 0x00;
        }
    }
}

// INT1 interrupt routine
ISR(INT1_vect) {
    // set flag
    buttonPressed = 1;
}


