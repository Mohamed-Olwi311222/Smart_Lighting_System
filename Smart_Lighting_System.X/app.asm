; Authors: Mohamed Olwi, Sama Mohamed 
; Description:
; This program uses the ADC of the PIC18F4620 to monitor an LDR sensor.
; It turns an LED on/off based on whether the light level crosses a defined threshold.


; PIC18F4620 Configuration Bit Settings
; Assembly source line config statements
; CONFIG1H
  CONFIG  OSC = INTIO67         ; Oscillator Selection bits (Internal oscillator block, port function on RA6 and RA7)
  CONFIG  FCMEN = OFF           ; Fail-Safe Clock Monitor Enable bit (Fail-Safe Clock Monitor disabled)
  CONFIG  IESO = OFF            ; Internal/External Oscillator Switchover bit (Oscillator Switchover mode disabled)
; CONFIG2L
  CONFIG  PWRT = OFF            ; Power-up Timer Enable bit (PWRT disabled)
  CONFIG  BOREN = SBORDIS       ; Brown-out Reset Enable bits (Brown-out Reset enabled in hardware only (SBOREN is disabled))
  CONFIG  BORV = 3              ; Brown Out Reset Voltage bits (Minimum setting)
; CONFIG2H
  CONFIG  WDT = OFF             ; Watchdog Timer Enable bit (WDT disabled (control is placed on the SWDTEN bit))
  CONFIG  WDTPS = 32768         ; Watchdog Timer Postscale Select bits (1:32768)
; CONFIG3H
  CONFIG  CCP2MX = PORTC        ; CCP2 MUX bit (CCP2 input/output is multiplexed with RC1)
  CONFIG  PBADEN = OFF          ; PORTB A/D Enable bit (PORTB<4:0> pins are configured as digital I/O on Reset)
  CONFIG  LPT1OSC = OFF         ; Low-Power Timer1 Oscillator Enable bit (Timer1 configured for higher power operation)
  CONFIG  MCLRE = OFF           ; MCLR Pin Enable bit (RE3 input pin enabled; MCLR disabled)
; CONFIG4L
  CONFIG  STVREN = ON           ; Stack Full/Underflow Reset Enable bit (Stack full/underflow will cause Reset)
  CONFIG  LVP = OFF             ; Single-Supply ICSP Enable bit (Single-Supply ICSP disabled)
  CONFIG  XINST = OFF           ; Extended Instruction Set Enable bit (Instruction set extension and Indexed Addressing mode disabled (Legacy mode))
; CONFIG5L
  CONFIG  CP0 = OFF             ; Code Protection bit (Block 0 (000800-003FFFh) not code-protected)
  CONFIG  CP1 = OFF             ; Code Protection bit (Block 1 (004000-007FFFh) not code-protected)
  CONFIG  CP2 = OFF             ; Code Protection bit (Block 2 (008000-00BFFFh) not code-protected)
  CONFIG  CP3 = OFF             ; Code Protection bit (Block 3 (00C000-00FFFFh) not code-protected)
; CONFIG5H
  CONFIG  CPB = OFF             ; Boot Block Code Protection bit (Boot block (000000-0007FFh) not code-protected)
  CONFIG  CPD = OFF             ; Data EEPROM Code Protection bit (Data EEPROM not code-protected)
; CONFIG6L
  CONFIG  WRT0 = OFF            ; Write Protection bit (Block 0 (000800-003FFFh) not write-protected)
  CONFIG  WRT1 = OFF            ; Write Protection bit (Block 1 (004000-007FFFh) not write-protected)
  CONFIG  WRT2 = OFF            ; Write Protection bit (Block 2 (008000-00BFFFh) not write-protected)
  CONFIG  WRT3 = OFF            ; Write Protection bit (Block 3 (00C000-00FFFFh) not write-protected)
; CONFIG6H
  CONFIG  WRTC = OFF            ; Configuration Register Write Protection bit (Configuration registers (300000-3000FFh) not write-protected)
  CONFIG  WRTB = OFF            ; Boot Block Write Protection bit (Boot Block (000000-0007FFh) not write-protected)
  CONFIG  WRTD = OFF            ; Data EEPROM Write Protection bit (Data EEPROM not write-protected)
; CONFIG7L
  CONFIG  EBTR0 = OFF           ; Table Read Protection bit (Block 0 (000800-003FFFh) not protected from table reads executed in other blocks)
  CONFIG  EBTR1 = OFF           ; Table Read Protection bit (Block 1 (004000-007FFFh) not protected from table reads executed in other blocks)
  CONFIG  EBTR2 = OFF           ; Table Read Protection bit (Block 2 (008000-00BFFFh) not protected from table reads executed in other blocks)
  CONFIG  EBTR3 = OFF           ; Table Read Protection bit (Block 3 (00C000-00FFFFh) not protected from table reads executed in other blocks)
; CONFIG7H
  CONFIG  EBTRB = OFF           ; Boot Block Table Read Protection bit (Boot Block (000000-0007FFh) not protected from table reads executed in other blocks)

// config statements should precede project file includes.
PROCESSOR 18f4620
#include <pic18f4620.inc>
        ORG 0x0000                 ; Set the reset vector to address 0x0000
        GOTO Start                 ; Jump to the start of the code

;define delay variables
delay0 equ 0x450h
delay1 equ 0x451h
delay2 equ 0x452h


;define adc result variables
adc_res_low equ 0x452h
adc_res_high equ 0x453h
;define LDR sensor daylight threshold
ldr_daylight_threshold equ 0x454h
;define LDR flag for turning on the led on RC0
 threshold_flag equ 0x456h
;------------------------------
; Main Program
;------------------------------
Start:
;Configure Internal Clock to be 8 MHZ
    MOVLW 0x72h		    ; Sets IRCF (internal clock frequency), and SCS bits (clock source)
    MOVWF OSCCON	    ; This might trigger the error in Proteus

;Configure EUSART for debugging
    ;BAUDCON Register
    MOVLW 0x48H
    MOVWF BAUDCON
    ;RCSTA Register
    MOVLW 0x80H
    MOVWF RCSTA
    ;TXSTA Register
    MOVLW 0x26H
    MOVWF TXSTA
    ;SPBRG Register
    MOVLW 0xCFH
    MOVWF SPBRG

    call configure_interrupt
    call configure_adc
    call configure_led
    MOVLW 0x00H
    MOVWF STATUS		    ;Initialize the STATUS register with 0
    MOVLW 0xB8h			    ;load ldr_daylight_threshold with 133(daylight volatage)
    MOVWF ldr_daylight_threshold
    MOVLW 0x01h			    ;Initalize the threshold_flag with zero
    MOVWF threshold_flag
main_loop:
    call start_adc_conversion	    ;start the adc conversion
    call check_adc_conversion	    ;check the read value with the threshold
    BTFSS threshold_flag, 0	    ;Check the threshold_flag to turn off the led if it is set
    BCF LATC, 0			    ;Day
    BTFSC threshold_flag, 0         ;Check the threshold_flag to turn on the led if it is clear
    BSF LATC, 0                     ;Night
    BRA main_loop
    return
;------------------------------
; configure_led Subroutine
    ;Configure the Led connected on RC0
;------------------------------
configure_led:
    BCF TRISC, 0	;Make RC0 as Output
    BCF LATC, 0		;Make RC0 default is Logic Low
    return
;------------------------------
; configure_interrupt Subroutine
    ;Configure the external interrupt
;------------------------------
configure_interrupt:
    BSF RCON, 7		;Enable Priority Feature		    IPEN
    BSF INTCON, 7	;Enable global interrupt		    GIEH
    BSF INTCON, 6	;Enable peripheral interrupt		    PIEL
    BSF INTCON2, 0	;PORTB on change interrupt high priority    RBIP
    BSF INTCON, 3	;Enable PORTB on change interrupt	    RBIE
    return
;------------------------------
; configure_adc Subroutine
    ;configure the adc peripheral
;------------------------------   
configure_adc:
    ;ADCON0 Bits
    BCF ADCON0, 2	;Set the ADC channel to AN0 (CHS0:CHS3) 
    BSF ADCON0, 0	;Enable ADC (ADON bit)
    
    ;ADCON1 Bits
    ;Set PORT Configuration to make AN0 an analog input
    BCF ADCON1, 0	
    BSF ADCON1, 3
    
    ;ADCON2 Bits
    ;Set the ADC Conversion Clock
    BSF ADCON2, 0
    BSF ADCON2, 2
    ;Set the Acquisition time
    BSF ADCON2, 3
    BSF ADCON2, 5
    ;Set the result to be right justified
    BSF ADCON2, 7
    return 
;------------------------------
; start_adc_conversion Subroutine
    ;Start the ADC conversion
;------------------------------   
start_adc_conversion:
    BSF ADCON0, 1	;Start ADC conversion	
    return
;------------------------------
; start_adc_conversion Subroutine
    ;Read the ADC conversion
;------------------------------   
read_adc_conversion:
;Poll the GO/DONE bit until the ADC is idle
check:
    BTFSC ADCON0, 1	    ;Check GO/DONE bit, skip if clear
    BRA check
    ;Read adc conversion result
    MOVF ADRESL, w
    MOVWF adc_res_low
    MOVF ADRESH, w   
    MOVWF adc_res_high
    return
;------------------------------
; check_adc_conversion Subroutine
    ;Check the ADC conversion, will set threshold_flag if night
;------------------------------      
check_adc_conversion:
    ;COMPARE adc_res_low WITH ldr_daylight_threshold
    ;If the subtraction results in an overflow (i.e., the result is outside the 8-bit range)
    ;the CF is cleared. Otherwise, it remains 1.
    call read_adc_conversion
    MOVF adc_res_low, w			;Store adc_res_low to Working register
    SUBWF ldr_daylight_threshold, w	;Subtract W(adc_res_low) from ldr_daylight_threshold_temp
    BTFSC STATUS, 0			;if borrow flag is set, (adc_res_low > ldr_daylight_threshold)
    MOVLW 0x01h				;threshold_flag is set, day
    BTFSS STATUS, 0			;if borrow flag is clear, (adc_res_low < ldr_daylight_threshold)
    MOVLW 0x00h				;threshold_flag is clear, night
    MOVWF threshold_flag
    MOVF adc_res_low, W
    MOVWF TXREG
    return
;------------------------------
; Delay Subroutine
    ;Make a delay of 5ms
;------------------------------
Delay5ms:
    MOVLW 0xC8			    ;200 cycles => each cycle is 5 us
    MOVWF delay0
Loop5ms:
    NOP
    NOP
    DECFSZ delay0, f
    BRA Loop5ms
    RETURN
;------------------------------
; Delay100ms Subroutine
    ;Make a delay of 100ms
;------------------------------
Delay100ms:
    MOVLW 0x64
    MOVWF delay1
Loop100ms:
    call Delay5ms
    DECFSZ delay1, f
    BRA Loop100ms
    RETURN
;------------------------------
; Delay500ms Subroutine
    ;Make a delay of 500ms
;------------------------------
Delay500ms:
    MOVLW 0x05
    MOVWF delay2
Loop500ms:
    call Delay100ms
    DECFSZ delay2, f
    BRA Loop500ms
    RETURN
END Start                       ; End of program
