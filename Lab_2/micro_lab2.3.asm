.include "m328pbdef.inc"

.org 0x0 
rjmp reset
.org 0x4
rjmp ISR1
    
reset:
    ; init Stack Pointer
    ldi r24, LOW(RAMEND)
    out SPL, r24
    ldi r24, HIGH(RAMEND)
    out SPL, r24

    ; enable INT1 on falling edge
    ldi r24, (1 << ISC01) | (0 << ISC00)
    sts EICRA, r24
    ldi r24, (1 << INT1)
    out EIMSK, r24

    ser r21           
    out DDRB, r21                         ; PORTB as output
    sei                                   ; enable interrupts
main:    
    rjmp main

; interrupt routine
ISR1:   
    ser r26
    out PORTB, r26                ; turn on all LEDs
    call delay_half               ; delay 0.5 seconds
    ldi r26, 1
    out PORTB, r26                ; turn off the LEDs except PB0
    call delay_three              ; delay 3 seconds
    clr r26 
    out PORTB, r26                ; turn off PB0 too
    reti

    
delay_half:    
    ldi r24, low(16*500)
    ldi r25, high(16*500) 
    call delay_ms
    ret
    
delay_three:
    ldi r24, low(16*3000)
    ldi r25, high(16*3000) 
    call delay_ms
    ret
    
; delay routine    
delay_mS:
    ldi r23, 249
loop_inn:
    sei                        ; allow delay routine to be interrupted
                               ; start the interrupt routine all over again
    dec r23
    nop
    brne loop_inn
    
    sbiw r24, 1
    brne delay_mS
    
    ret


