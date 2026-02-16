.include "m328PBdef.inc" ;ATmega328P microcontroller definition
    
.equ PD0=0
.equ PD1=1
.equ PD2=2
.equ PD3=3
.equ PD4=4
.equ PD5=5
.equ PD6=6
.equ PD7=7

.def quotient = r23
    
.org 0x00
rjmp main
.org 0x2A                  ; ADC interrupt vector
rjmp ADC_int
    
main:
 ldi r24, low(RAMEND)
 out SPL, r24
 ldi r24, high(RAMEND)
 out SPH, r24               ; init stack pointer

 ser r24
 out DDRD, r24              ; set PORTD as output

 ldi r24, 0b01000010        ; right adjusted, ADC2 channel, Avcc voltage ref.
 sts ADMUX, r24
 ldi r24, 0b10001111        ; ADEN = 1 (enable ADC circuit), ADIE =1 (enable interrupt), ADPS = 128
 sts ADCSRA, r24
    
 clr r24
 rcall lcd_init             ; initialize display
 ldi r24, low(100)
 ldi r25, high(100)         ; delay 100 mS
 rcall wait_msec
 sei
main1:
 rcall lcd_clear_display    ; clear display

 lds r24, ADCSRA    
 ori r24, (1<<ADSC)         ; set ADCS flag 
 sts ADCSRA, r24            ; start ADC conversion
 
 ldi r24, low(1000)
 ldi r25, high(1000)
 rcall wait_msec            ; delay 1 Sec

 rjmp main

ADC_int:
 ; adc conversion result (right-adjusted)
 lds r30, ADCL
 lds r31, ADCH
 
 ; Vadc = (5*ADC/1024) * 100 to get two decimals  
 
 ; calculate in r26:r27 -> 12*ADC/1024 = 3*ADC/256
 mov r26, r30
 mov r27, r31
 clr r17
 rcall multiplier           ; multiply ADC * 3
 ldi r17, 7                 ; divide by 256 - shift right 8 times
 
shift:
    lsr r27
    ror r26
    dec r17
    cpi r17,0
    breq exit
    rjmp shift
exit:
    
                             ; now we calculate 512*ADC/1024
 lsr r31  
 ror r30                     ; divide ADC / 2 - shift right one time 
 
; Vadc = 512*ADC/1024 - 12*ADC/1024 ;
 sub r30, r26  
 sbc r31, r27  
 
 ldi quotient, 0
 ldi r18, 100
 ldi r17, 0
 rcall divide                 ; divide Vadc by 100 to get whole part
                              ; fractional part in remainder r30
 
 ldi r24, '0'
 add r24, quotient
 rcall lcd_data               ; display data
 
 ldi r31,0
 ldi r18,10
 ldi quotient, 0
 rcall divide                 ; divide fractional part by 10 to get first decimal digit
                              ; second digit is the remainder r30
 
 ldi r24, '.'
 rcall lcd_data
 
 ldi r24, '0'
 add r24, quotient
 rcall lcd_data

 ldi r24, '0'
 add r24, r30
 rcall lcd_data
 
 reti
 
    
; multiply by 3 a 16-bit value in registers r26:r27
multiplier:
    ldi r28, 3
loop2:
    dec r28
    cpi r28,0
    breq exit3
    clc
    add r26,r26
    adc r27,r17
    rjmp loop2
exit3:
    ret
 
; divide a 16-bit value in registers r30:r31 
divide:
    cp r30,r18
    brsh loop
    cpi r31,0
    brne loop
    ldi quotient,0
    rjmp exit_now
loop: 
    clc
    sub r30, r18
    sbc r31, r17
    cpi r31, 0
    breq check
    inc quotient
    rjmp loop
check:
    cp r30, r18
    brcc exit1
    rjmp end
exit1:
    inc quotient
    rjmp loop
end:
    inc quotient
exit_now:
    ret
    

 lcd_init:
 ldi r24 ,low(200) 
 ldi r25 ,high(200) ; Wait 200 mSec
 rcall wait_msec 

 ldi r24 ,0x30 ; command to switch to 8 bit mode
 out PORTD ,r24 
 sbi PORTD ,PD3 ; Enable Pulse
 nop
 nop
 cbi PORTD ,PD3
 ldi r24 ,250 ;
 ldi r25 ,0 ; Wait 250uSec
 rcall wait_usec ;

 ldi r24 ,0x30 ; command to switch to 8 bit mode
 out PORTD ,r24 ;
 sbi PORTD ,PD3 ; Enable Pulse
 nop
 nop
 cbi PORTD ,PD3
 ldi r24 ,250 ;
 ldi r25 ,0 ; Wait 250uSec
 rcall wait_usec ;

 ldi r24 ,0x30 ; command to switch to 8 bit mode
 out PORTD ,r24 ;
 sbi PORTD ,PD3 ; Enable Pulse
 nop
 nop
 cbi PORTD ,PD3
 ldi r24 ,250 ;
 ldi r25 ,0 ; Wait 250uSec
 rcall wait_usec

 ldi r24 ,0x20 ; command to switch to 4 bit mode
 out PORTD ,r24
 sbi PORTD ,PD3 ; Enable Pulse
 nop
 nop
 cbi PORTD ,PD3
 ldi r24 ,250 ;
 ldi r25 ,0 ; Wait 250uSec
 rcall wait_usec

 ldi r24 ,0x28 ; 5x8 dots, 2 lines
 rcall lcd_command
 ldi r24 ,0x0c ; dislay on, cursor off
 rcall lcd_command
 
 rcall lcd_clear_display
 ldi r24 ,0x06 ; Increase address, no display shift
 rcall lcd_command ;
 ret
 
 lcd_clear_display:

 ldi r24 ,0x01 ; clear display command
 rcall lcd_command

 ldi r24 ,low(5) ;
 ldi r25 ,high(5) ; Wait 5 mSec
 rcall wait_msec ;
 ret
 
 lcd_command:
 cbi PORTD ,PD2 ; LCD_RS=0(PD2=0), Instruction
 rcall write_2_nibbles ; send Instruction
 ldi r24 ,250 ;
 ldi r25 ,0 ; Wait 250uSec
 rcall wait_usec
 ret
 
 lcd_data:
 sbi PORTD ,PD2 ; LCD_RS=1(PD2=1), Data
 rcall write_2_nibbles ; send data
 ldi r24 ,250 ;
 ldi r25 ,0 ; Wait 250uSec
 rcall wait_usec
 ret
 
 write_2_nibbles:
 push r24 ; save r24(LCD_Data)

 in r25 ,PIND ; read PIND

 andi r25 ,0x0f ;
 andi r24 ,0xf0 ; r24[3:0] Holds previus PORTD[3:0]
 add r24 ,r25 ; r24[7:4] <-- LCD_Data_High_Byte
 out PORTD ,r24 ;

 sbi PORTD ,PD3 ; Enable Pulse
 nop
 nop
 cbi PORTD ,PD3

 pop r24 ; Recover r24(LCD_Data)
 swap r24 ;
 andi r24 ,0xf0 ; r24[3:0] Holds previus PORTD[3:0]
 add r24 ,r25 ; r24[7:4] <-- LCD_Data_Low_Byte
 out PORTD ,r24

 sbi PORTD ,PD3 ; Enable Pulse
 nop
 nop
 cbi PORTD ,PD3

 ret
 
 wait_msec:
 push r24 ; 2 cycles
 push r25 ; 2 cycles
 ldi r24 , low(999) ; 1 cycle
 ldi r25 , high(999) ; 1 cycle
 rcall wait_usec ; 998.375 usec
 pop r25 ; 2 cycles
 pop r24 ; 2 cycles
 nop ; 1 cycle
 nop ; 1 cycle
 sbiw r24 , 1 ; 2 cycles
 brne wait_msec ; 1 or 2 cycles
 ret ; 4 cycles


wait_usec:
 sbiw r24 ,1 ; 2 cycles (2/16 usec)
 call delay_8cycles ; 4+8=12 cycles
 brne wait_usec ; 1 or 2 cycles
 ret
delay_8cycles:
 nop
 nop
 nop
 ret
