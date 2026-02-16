.include "m328pbdef.inc"

.org 0x0 
rjmp reset
.org 0x4
rjmp ISR1
    
reset:
    ldi r24, LOW(RAMEND)
    out SPL, r24
    ldi r24, HIGH(RAMEND)
    out SPL, r24
    
    ldi r24, (1 << ISC01) | (0 << ISC00)
    sts EICRA, r24
    ldi r24, (1 << INT1)
    out EIMSK, r24

    ser r21           
    out DDRB, r21 ;output
    sei ;enable interrupts
main:    
    rjmp main
    
ISR1:   
    ser r26
    out PORTB, r26
    call delay_half
    ldi r26, 1
    out PORTB, r26
    call delay_three
    clr r26 
    out PORTB, r26
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
    
;delay routine    
delay_mS:
    ldi r23, 249
loop_inn:
    sei
    dec r23
    nop
    brne loop_inn
    
    sbiw r24, 1
    brne delay_mS
    
    ret


