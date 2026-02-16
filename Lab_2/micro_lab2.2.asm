.include "m328PBdef.inc"
    
.equ FOSC_MHZ=16	    ;MHz
.equ DEL_ms=1000	    ;ms
.equ DEL_NU=FOSC_MHZ*DEL_ms
    
.def result = r16
.def mask = r18
.def bits = r19
 
.org 0x0
rjmp init
.org 0x2
rjmp ISR0
    
init:    
    ; Init Stack Pointer
    ldi r24, LOW(RAMEND)
    out SPL, r24
    ldi r24, HIGH(RAMEND)
    out SPL, r24

    ; enable INT0 on falling edge
    ldi r24, (1 << ISC01) | (0 << ISC00)
    sts EICRA, r24
    ldi r24, (1 << INT0)
    out EIMSK, r24
    
    ser r26		
    out DDRC, r26		; PORTC as output
    
    
    sei				    ; enable global interrupts
 
; main programm    
main:
    clr r26
loop:
    out PORTC, r26
    
    ldi r24, low(DEL_NU)
    ldi r25, high(DEL_NU)	; set delay
    rcall delay_mS
    
    inc r26
    
    cpi r26, 32
    breq main
    rjmp loop

; interrupt routine
ISR0:
    cli				        ; disable interrupts
    
    ldi result, 0		    ; initialize the result register
    in r17, PINB		    ; read PORTB
    com r17                 ; inverse logic input
    ldi mask, 0b011111		; initialize mask register
    ldi bits, 5			    ; bits to check
    
    and r17, mask		    ; isolate PB4-PB0 bits
int_loop:
    lsr r17			
    brcc check			    ; if LSB is 1
    inc result			    ; increment the result register
    lsl result			    ; and shift left to check next bit
check:    
    dec bits			    ; else if there are more bits
    brne int_loop		    ; repeat
isr0_exit:
    lsr result			    ; else shift the last "1" to LSB
    out PORTC, result		; and show at the output PORTC
    sei				        ; enable interrupts
    reti
    
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
