.include "m328PBdef.inc"

.equ FOSC_MHZ=16	;MHz
.equ DEL_mS=10	;mS
.equ F1=FOSC_MHZ*DEL_mS
	
 reset:
    ldi r24,low(RAMEND)
    out SPL,r24
    ldi r24,high(RAMEND)
    out SPH,r24  
    
    ser r24
    out DDRD,r24
    out PORTD,r24
    
    clr r26
    
loop1:
    ldi r24, low(F1-1)	    ;first ms of delay will be implemented 
    ldi r25, high(F1-1)	    ;differently
    rcall delay0
    com r26
    out PORTD,r26
    rjmp loop1
    
    
delay0:
    ldi r23, 248	    ;so we can add two more "nop" instructions
    nop			    ;and decrease the value of r23
    nop			    ;(responsible for ms-loop) by 1
loop0_in:		    ;in order to have an exact delay
    dec r23
    nop
    brne loop0_in
    
delay_inner:		    
    ldi	r23, 249	    ; (1 cycle)	
loop_inn:
    dec r23		    ; 1 cycle
    nop			    ; 1 cycle
    brne loop_inn	    ; 1 or 2 cycles
     
    sbiw r24 ,1		    ; 2 cycles
    brne delay_inner	    ; 1 or 2 cycles
 
    ret			    ;4 cycles