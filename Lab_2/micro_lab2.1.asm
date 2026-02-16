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

	; enable interrupt INT1 on falling edge
    ldi r24, (1 << ISC11) | (0 << ISC10)
    sts EICRA, r24
    ldi r24, (1 << INT1)
    out EIMSK, r24
    
    ser r26
    out DDRB, r26		; PORTB as output		
    out DDRC, r26		; PORTB as output
    clr r16	    		; interruption counter to 0
    out PORTC, r16
    
    sei					; enable global interrupts
 
; main programm    
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

; delay routine    
delay_mS:
    ldi r23, 249
loop_inn:
    dec r23
    nop
    brne loop_inn
    
    sbiw r24, 1
    brne delay_mS
    
    ret
    
; interrupt routine
ISR1:   
    ldi r24, (1 << INTF1)
    out EIFR, r24					; clear INTF1
    ldi r24, low(100*FOSC_MHZ)
    ldi r25, high(100*FOSC_MHZ)
    rcall delay_mS					; delay 100ms
    in r24, EIFR
    cpi r24, 0						; INTF1 = 1 -> there’s been a debounce 
    brne ISR1						; wait until there’s no debounce
    
    in r17, PIND
    sbrs r17, 6 					; if PD6 is pressed		
    rjmp isr1_exit					; then exit the interrupt routine

    inc r16	                		; increment counter
    cpi r16, 32						; check if the counter reached 32
    brne update_leds				; if not, update the LEDs
    clr r16							; reset the counter to zero 
    update_leds:
	out PORTC, r16
    isr1_exit:            
	reti
    
