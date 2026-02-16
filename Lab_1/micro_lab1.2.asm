.include "m328PBdef.inc"
    
.def A = r16
.def B = r17
.def C = r18
.def D = r19
.def T = r20
.def F0 = r21
.def F1 = r22
.def counter = r23
.def step = r24
	
init:
    ldi r25,low(RAMEND)
    out SPL,r25
    ldi r25,high(RAMEND)
    out SPH,r25  
    
    ldi A, 0x45
    ldi B, 0x23
    ldi C, 0x21
    ldi D, 0x01
    ldi step, 0x05
    ldi counter, 5
    
loop:
    clr T	    ;clear the temporary and the results' registers
    clr F0
    clr F1
    
    com A	    
    mov F0, A	    ;F0 = A'
    mov F1, A	    ;F1 = A'
    com A
    
    com B	    
    and F0, B	    ;F0 = A'*B'
    mov T, B	    ;T = B'
    com B
    
    com C
    and F0, C	    ;F0 = A'*B'*C'
    com C	    
    or F1, C	    ;F1 = (A' + C)
    
    or F0, D	    ;F0 = (A'*B'*C' + D)
    com D
    or T, D	    ;T = (B' + D')
    com D
    
    com F0	    ;F0 = F0'
    and F1, T	    ;F1 = (A' + C)*(B' + D')
    
    ldi T, 0x04	    ;T is used in order to use as little regs as possible
    inc A	    
    inc B
    inc B	    ;A and B are increased with "inc"
    add C, T	    ;C and D are increased with "add" 
    add D, step	    ;because that way less instructions are needed
    
    dec counter	
    brne loop