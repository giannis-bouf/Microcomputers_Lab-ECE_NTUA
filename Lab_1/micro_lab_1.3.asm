.include "m328PBdef.inc"

 
init:
    ldi r25,low(RAMEND)
    out SPL,r25
    ldi r25,high(RAMEND)
    out SPH,r25  
    
    ser r26
    out DDRD,r26
    out PORTD,r26   ;PortD is output
    
    ldi r21, 0b00000001  ;initialize LED
    bst r21, 7           ;Store the value of the 7th bit (T flag) into the T Register
    out PORTD, r21
    call delayone
    jmp goleft    
      
    
goleft:
    lsl r21  ;rotate left
    bst r21, 7
    out PORTD, r21
    call delayone
    brtc goleft  ;if MSB not set continue
    call delaytwo
    jmp goright
     
goright:
    lsr r21  ;rotate right
    bst r21, 0
    out PORTD, r21
    call delayone
    brtc goright  ;if LSB not set continue
    call delaytwo
    jmp goleft
    
delaytwo:
    ldi r24, low(2000-1)
    ldi r25, high(2000-1)      
    call delay0
    ret
    
delayone: 
    ldi r24, low(1500-1)
    ldi r25, high(1500-1)      
    call delay0
    ret
    
delay0:
    ldi r23, 248
    nop
    nop
loop0_in:
    dec r23
    nop
    brne loop0_in
     
    ;total group delay 996 cycles
delay_inner:		    
    ldi	r23, 249	    ; (1 cycle)	
loop_inn:
    dec r23		    ; 1 cycle
    nop			    ; 1 cycle
    brne loop_inn	    ; 1 or 2 cycles
     
    sbiw r24 ,1		    ; 2 cycles
    brne delay_inner	    ; 1 or 2 cycles
 
    ret			    ;4 cycles



