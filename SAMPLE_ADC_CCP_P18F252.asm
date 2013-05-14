;******************************************************************************
;   This file is a basic template for assembly code for a PIC18F252. Copy     *
;   this file into your project directory and modify or add to it as needed.  *
;                                                                             *
;   Refer to the MPASM User's Guide for additional information on the         *
;   features of the assembler.                                                *
;                                                                             *
;   Refer to the PIC18FXX2 Data Sheet for additional information on the       *
;   architecture and instruction set.                                         *
;                                                                             *
;******************************************************************************
;                                                                             *
;    Filename:      sample_adc_ccp_p18f252.asm                                *
;    Date:          11.05.2013                                                *
;                   14.05.2013
;    File Version:                                                            *
;                                                                             *
;    Author:        burnout840@gmail.com                                      *
;    Company:                                                                 *
;                                                                             * 
;******************************************************************************
;                                                                             *
;    Files Required: P18F252.INC                                              *
;                                                                             *
;******************************************************************************

        LIST P=18F252           ;directive to define processor
        #include <P18F252.INC>  ;processor specific variable definitions

;******************************************************************************
;Configuration bits
;Microchip has changed the format for defining the configuration bits, please 
;see the .inc file for futher details on notation.  Below are a few examples.



;   Oscillator Selection:
    CONFIG      OSC = XT             ;XT

;******************************************************************************
;Variable definitions
; These variables are only needed if low priority interrupts are used. 
; More variables may be needed to store other special function registers used
; in the interrupt routines.

                CBLOCK  0x080
                WREG_TEMP       ;variable used for context saving 
                STATUS_TEMP     ;variable used for context saving
                BSR_TEMP        ;variable used for context saving
                ENDC

                REG_A equ 0x000

                SENSOR_L equ 0x001
                SENSOR_H equ 0x002

                POT1_L equ 0x003
                POT1_H equ 0x004

                POT2_L equ 0x005
                POT2_H equ 0x006

                REG_TEMP equ 0x007

                LED_1 equ 0

                ;CBLOCK 0x000
                ;EXAMPLE                ;example of a variable in access RAM
                ;ENDC

;******************************************************************************
;EEPROM data
; Data to be programmed into the Data EEPROM is defined here

                ORG     0xf00000

                DE      "Test Data",0,1,2,3,4,5

;******************************************************************************
;Reset vector
; This code will start executing when a reset occurs.

                ORG     0x0000

                GOTO MAIN               ;go to start of main code

;******************************************************************************
;High priority interrupt vector
; This code will start executing when a high priority interrupt occurs or
; when any interrupt occurs if interrupt priorities are not enabled.

                ORG     0x0008

                BRA     HIGHINT         ;go to high priority interrupt routine

;******************************************************************************
;Low priority interrupt vector and routine
; This code will start executing when a low priority interrupt occurs.
; This code can be removed if low priority interrupts are not used.

                ORG     0x0018

                MOVFF   STATUS,STATUS_TEMP      ;save STATUS register
                MOVFF   WREG,WREG_TEMP          ;save working register
                MOVFF   BSR,BSR_TEMP            ;save BSR register

;       *** low priority interrupt code goes here ***


                MOVFF   BSR_TEMP,BSR            ;restore BSR register
                MOVFF   WREG_TEMP,WREG          ;restore working register
                MOVFF   STATUS_TEMP,STATUS      ;restore STATUS register
                RETFIE

;******************************************************************************
;High priority interrupt routine
; The high priority interrupt code is placed here to avoid conflicting with
; the low priority interrupt vector.

HIGHINT:

;       *** high priority interrupt code goes here ***


                RETFIE  FAST

;******************************************************************************
;Start of main program
; The main program code is placed here.

MAIN:

;************ НАСТРОЙКА ПОРТОВ ************************************************

 CLRF PORTA                     ; Настройка порта PORTA
 CLRF LATA                      ;
 MOVLW 0x0B                     ; RA<1:0> RA<3> вход
 MOVWF TRISA                    ; RA<2> RA<4:6> выход

 CLRF PORTB                     ; Настройка порта PORTB
 CLRF LATB                      ;
 MOVLW 0xCF                     ; RB<3:0> RB<7:6> вход
 MOVWF TRISB                    ; RB<5:4> выход

 CLRF PORTC                     ; Настройка PORTC
 CLRF LATC                      ;
 MOVLW 0xCC                     ; RC<3> RC<1> RC<7:6> вход
 MOVWF TRISC                    ; RC<0> RC<2> RC<5:4> выход

;************ НАСТРОЙКА МОДУЛЯ CPP1 КАК ШИМ ***********************************

 MOVLW 0xFF                     ; Устанавливаем период ШИМ в регистре PR2
 MOVWF PR2                      ;
 MOVLW 0x80                     ; Устанавливаем длительность импульса в регистрах CCPR1L и CCP1CON<5:4>
 MOVWF CCPR1L                   ;
 BSF CCP1CON,5                  ;
 BSF CCP1CON,4                  ;
 BCF TRISC,2                    ; Настраиваем вывод CCP1 как выход, сбросив бит TRISC<2>
 MOVLW 0x7F                     ; Настраиваем предделитель и включаем TMR2 в регистре T2CON
 MOVWF T2CON
 ;BSF CCP1CON,3                 ; Включаем CCP1 в режиме ШИМ
 ;BSF CCP1CON,2

;************ НАСТРОЙКА МОДУЛЯ АЦП ********************************************

 MOVLW 0xC4                     ; AN0, AN1, AN3 - входной канал; 6 старших бит ADRESH читаются как ноль
 MOVWF ADCON1                   ; Внутренний RC генератор

;************ ОСНОВНОЕ КОЛЬЦО ПРОГРАММЫ ***************************************

MAIN_RING:                      ;

 MOVLW 0xC1                     ; Выполняем АЦП преобразование для AN0
 MOVWF ADCON0                   ;
 MOVLW 0x80                     ; Задежка для кондера
 MOVWF REG_A                    ;
 DECFSZ REG_A,1,0               ;
 BRA $-2                        ;
 BSF ADCON0, GO                 ;
 BTFSC ADCON0, GO, 0            ; Ожидаем окончания преобразования
 BRA $-2                        ;
 MOVF ADRESL,0,0                ; Запоминаем результат
 MOVWF SENSOR_L,0               ;
 MOVF ADRESH,0,0                ;
 MOVWF SENSOR_H,0               ;

 MOVLW 0xC9                     ; Выполняем АЦП преобразование для AN1
 MOVWF ADCON0                   ;
 MOVLW 0x80                     ;
 MOVWF REG_A                    ;
 DECFSZ REG_A,1,0               ;
 BRA $-2                        ;
 BSF ADCON0, GO                 ;
 BTFSC ADCON0, GO, 0            ;
 BRA $-2                        ;
 MOVF ADRESL,0,0                ;
 MOVWF POT1_L,0                 ;
 MOVF ADRESH,0,0                ;
 MOVWF POT1_H,0                 ;

 MOVLW 0xD9                     ; Выполняем АЦП преобразование для AN3
 MOVWF ADCON0                   ;
 MOVLW 0x80                     ;
 MOVWF REG_A                    ;
 DECFSZ REG_A,1,0               ;
 BRA $-2                        ;
 BSF ADCON0, GO                 ;
 BTFSC ADCON0, GO, 0            ;
 BRA $-2                        ;
 MOVF ADRESL,0,0                ;
 MOVWF POT2_L,0                 ;
 MOVF ADRESH,0,0                ;
 MOVWF POT2_H,0                 ;

 MOVF POT1_H,0                  ; Если уровень напряжения датчика больше либо равен
 CPFSLT SENSOR_H,0              ; напряжению регултора, то включаем ШИМ и светодиод
 BRA $+4                        ; Если меньше - делаем обратное
 BRA UPDATE_CONF_CCP            ;
 MOVF POT1_L,0                  ;
 CPFSLT SENSOR_L,0              ;
 BRA $+4                        ;
 BRA $+0x10                     ;
 BSF PORTC, LED_1               ; Включить светодиод 1
 BSF CCP1CON,3                  ; Включить ШИМ
 BSF CCP1CON,2                  ; 
 BRA $+8                        ;
 BCF PORTC, LED_1               ; Выключить светодиод 1
 BCF CCP1CON,3                  ; Выключить ШИМ
 BCF CCP1CON,2                  ;

UPDATE_CONF_CCP:                ; Обновление параметров ШИМ
 MOVLW 0x02                     ;
 CPFSGT POT2_H,0                ;
 BRA MAIN_RING                  ;
 MOVLW 0x79                     ;
 CPFSGT POT2_L,0                ;
 BRA MAIN_RING                  ;

 MOVLW 0x80                     ;
 SUBWF POT2_L,0,0               ;
 MOVWF REG_TEMP,0               ;
 RLNCF REG_TEMP,0,0             ;
 MOVWF CCPR1L,0                 ;

GOTO MAIN_RING                  ; Вернуться в начало основного кольца

;******************************************************************************
;End of program

                END
