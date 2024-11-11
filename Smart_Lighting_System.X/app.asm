; 
    ; Simple LED Blink Program for PIC18F4620
; Author: Mohamed Olwi
; Description: Blink an LED on RB0 using a simple delay


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

;------------------------------
; Main Program
;------------------------------
Start:
    BCF TRISB, 0		   ; Set RB0 to be output
    BCF LATB, 0			   ; Output 0 v on RB0

MainLoop:
    BSF LATB, 0			   ; Output 5 v on RB0
    CALL Delay5ms		   ; Delay
    BCF LATB, 0			   ; Output 0 v on RB0
    CALL Delay5ms			
    GOTO MainLoop
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
    GOTO Loop5ms
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
    GOTO Loop100ms
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
    GOTO Loop500ms
    RETURN
END Start                       ; End of program
