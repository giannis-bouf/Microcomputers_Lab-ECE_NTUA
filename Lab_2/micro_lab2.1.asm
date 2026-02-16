.include "m328PBdef.inc"
    
.equ FOSC_MHZ=16	    ;MHz
.equ DEL_ms=500	    ;ms
.equ DEL_NU=FOSC_MHZ*DEL_ms

.org 0x0
rjmp init
.org 0x4
rjmp ISR1
    
init:    
;Init Stack Pointer
    ldi r24, LOW(RAMEND)
    out SPL, r24
    ldi r24, HIGH(RAMEND)
    out SPL, r24
    
    ldi r24, (1 << ISC11) | (0 << ISC10)
    sts EICRA, r24
    ldi r24, (1 << INT1)
    out EIMSK, r24
    
    ser r26
    out DDRB, r26		;init PORTB as output		
    out DDRC, r26		;init PC4-PC0 as output
    clr r16	    		;init interruption counter to 0
    out PORTC, r16
    
    sei
 
;main programm    
main:
    clr r26
loop:
    out PORTB, r26
    
    ldi r24, low(DEL_NU)
    ldi r25, high(DEL_NU) ;set delay
    rcall delay_mS
    
    inc r26
    
    cpi r26, 16
    breq main
    rjmp loop

;delay routine    
delay_mS:
    ldi r23, 249
loop_inn:
    dec r23
    nop
    brne loop_inn
    
    sbiw r24, 1
    brne delay_mS
    
    ret
    
;interrupt routine
ISR1:   
    ldi r24, (1 << INTF1)
    out EIFR, r24
    ldi r24, low(100*FOSC_MHZ)
    ldi r25, high(100*FOSC_MHZ)
    rcall delay_mS
    in r24, EIFR
    cpi r24, 0
    brne ISR1
    
    in r17, PIND		; Read the state of the port with the button
    sbrs r17, 6 			
    rjmp isr1_exit		; Button is pressed, exit the ISR

    inc r16	                ; increment counter
    cpi r16, 32			; check if the counter reached 32
    brne update_leds		; if not, update the LEDs
    clr r16			; reset the counter to zero 
    update_leds:
	out PORTC, r16
    isr1_exit:            
	reti
    