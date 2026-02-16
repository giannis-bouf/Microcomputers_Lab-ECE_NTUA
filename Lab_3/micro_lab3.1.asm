.include "m328PBdef.inc"

values: .db 5, 26, 46, 66, 87, 107, 128, 148, 168, 189, 209, 230, 250
    
.def DC_VALUE = r18
	
init:
    ldi r24, low(RAMEND)
    out SPL, r24
    ldi r24, high(RAMEND)
    out SPH, r24
    
    ; initialize TMR1A for Fast PWM, 8-bit, prescaler = 8
    ldi r24, (1<<WGM10) | (1<<COM1A1)
    sts TCCR1A, r24
    ldi r24, (1<<WGM12) | (1<<CS11) 	; non-inverting mode
    sts TCCR1B, r24
    
    clr r24
    out DDRD, r24 				; set PORTD as input
    ser r24
    out DDRB, r24 				; set PORTB as output
    
    ; load the memory address of the array
    ldi ZL, low(values) 
    ldi ZH, high(values)
    adiw ZL, 6 					; offset = 6 (50% duty cycle - 128)
    lpm DC_VALUE, Z 				; initialize DC_VALUE = 128
main:
    ldi r24, low(16*100)
    ldi r25, high(16*100)			; set delay (100 ms)
    
    sbis PIND, 1 					; if PD1 is pressed
    rjmp increase 				; increase duty cycle
    sbis PIND, 2 					; if PD2 is pressed
    rjmp decrease 				; decrease duty cycle
output:    
    sts OCR1AL, DC_VALUE 			; pwm 
    rcall delay_mS 				; delay to avoid debounce
    rjmp main 					; repeat
    
increase:
    cpi DC_VALUE,250 				; duty cycle doesn't exceed 98%
    breq inc_exit
    adiw ZL, 1 					; next element of array
    lpm DC_VALUE, Z 				; update DC_VALUE
inc_exit:
    rjmp output
    
decrease:
    cpi DC_VALUE,5 				; duty cycle isn't bellow 2%
    breq dec_exit
    sbiw ZL, 1 					; previous element of array
    lpm DC_VALUE, Z 
dec_exit:
    rjmp output
    
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
