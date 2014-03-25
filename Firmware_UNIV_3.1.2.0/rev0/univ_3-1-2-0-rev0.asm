;==============================================================================
;   HAPCAN - Home Automation Project Firmware (http://hapcan.com)
;   Copyright (C) 2014 hapcan.com
;
;   This program is free software: you can redistribute it and/or modify
;   it under the terms of the GNU General Public License as published by
;   the Free Software Foundation, either version 3 of the License, or
;   (at your option) any later version.
;
;   This program is distributed in the hope that it will be useful,
;   but WITHOUT ANY WARRANTY; without even the implied warranty of
;   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
;   GNU General Public License for more details.
;
;   You should have received a copy of the GNU General Public License
;   along with this program.  If not, see <http://www.gnu.org/licenses/>.
;==============================================================================
;   Filename:              univ_3-1-2-0.asm
;   Associated diagram:    univ_3-1-2-x.sch
;   Author:                Jacek Siwilo                          
;   Note:                  6 channel touch button & temp module
;==============================================================================
;   Revision History
;   Rev:  Date:     Details:
;   0     02.2014   Original version
;==============================================================================
;===  FIRMWARE DEFINITIONS  =================================================== 
;==============================================================================
    #define    ATYPE    .1                            ;application type [0-255]
    #define    AVERS    .2                         ;application version [0-255]
    #define    FVERS    .0                            ;firmware version [0-255]

    #define    FREV     .0                         ;firmware revision [0-65536]
;==============================================================================
;===  NEEDED FILES  ===========================================================
;==============================================================================
    LIST P=18F26K80                              ;directive to define processor
    #include <P18F26K80.INC>           ;processor specific variable definitions
    #include "univ_3-1-2-0-rev0.inc"                         ;project variables
    #include "univ3-fake_bootloader-rev2.inc"    ;fake bootloader for debugging
INCLUDEDFILES   code    
    #include "univ3-routines-rev3.inc"                     ;UNIV 3 CPU routines
    #include "univ3-mTouch-rev0.inc"                ;UNIV 3 CPU mTouch routines
    #include "univ3-1-wire-rev0.inc"                          ;1-wire functions
;==============================================================================
;===  CONFIG  DATA  ===========================================================
;==============================================================================
EEPROM      code                                                ;default config
    org 0xF00008
    DE      0x09,0x09,0x09,0x09,0x09,0x09,0x00  ;buttons send only on & off msg
    org 0xF00014
    DE      0xFF,0x08,0x00,0x01,0x40,0x02,0x00,0x00
                 ;|use last saved thermostat value on power up
                           ;|thermostat last saved value = 20deg
                                     ;|histeresis = 0.5deg
                                          ;|read temperature offset = 0deg
    org 0xF00050           ;touch button thresholds: press = 100 & release = 50
    DE      0x00,0x64,0x00,0x32,0x00,0x64,0x00,0x32,0x00,0x64,0x00,0x32
    DE      0x00,0x64,0x00,0x32,0x00,0x64,0x00,0x32,0x00,0x64,0x00,0x32
;==============================================================================
;===  FIRMWARE CHECKSUM  ======================================================
;==============================================================================
FIRMCHKSM   code    0x001000
    DB      0x5C, 0x49, 0x3C, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF
;==============================================================================
;===  FIRMWARE ID  ============================================================
;==============================================================================
FIRMID      code    0x001010
    DB      0x30, 0x00, 0x03,ATYPE,AVERS,FVERS,FREV>>8,FREV
;            |     |     |     |     |     |     |_____|_____ firmware revision
;            |     |     |     |     |     |__________________ firmware version
;            |     |     |     |     |_____________________ application version
;            |     |     |     |______________________________ application type
;            |     |     |________________________________ hardware version '3'
;            |_____|______________________________________ hardware type 'UNIV'
;==============================================================================
;===  MOVED VECTORS  ==========================================================
;==============================================================================
;PROGRAM RESET VECTOR
FIRMRESET   code    0x1020
        goto    Main
;PROGRAM HIGH PRIORITY INTERRUPT VECTOR
FIRMHIGHINT code    0x1030
        call    HighInterrupt
        retfie
;PROGRAM LOW PRIORITY INTERRUPT VECTOR
FIRMLOWINT  code    0x1040
        call    LowInterrupt
        retfie

;==============================================================================
;===  FIRMWARE STARTS  ========================================================
;==============================================================================
FIRMSTART   code    0x001050
;------------------------------------------------------------------------------
;---  LOW PRIORITY INTERRUPT  -------------------------------------------------
;------------------------------------------------------------------------------
LowInterrupt
        movff   STATUS,STATUS_LOW           ;save STATUS register
        movff   WREG,WREG_LOW               ;save working register
        movff   BSR,BSR_LOW                 ;save BSR register
        movff   FSR0L,FSR0L_LOW             ;save other registers used in high int
        movff   FSR0H,FSR0H_LOW
        movff   FSR1L,FSR1L_LOW
        movff   FSR1H,FSR1H_LOW

    ;main firmware ready flag
        movlb   0x1
        btfss   FIRMREADY,0
        bra     ExitLowInterrupt            ;main firmware is not ready yet
    ;CAN buffer
        movlb   0x1
        btfsc   CANFULL,0                   ;check if CAN received anything
        call    CANInterrupt                ;proceed with CAN interrupt

ExitLowInterrupt
        movff   BSR_LOW,BSR                 ;restore BSR register
        movff   WREG_LOW,WREG               ;restore working register
        movff   STATUS_LOW,STATUS           ;restore STATUS register
        movff   FSR0L_LOW,FSR0L             ;restore other registers used in high int
        movff   FSR0H_LOW,FSR0H
        movff   FSR1L_LOW,FSR1L
        movff   FSR1H_LOW,FSR1H
    return

;------------------------------------------------------------------------------
;---  HIGH PRIORITY INTERRUPT  ------------------------------------------------
;------------------------------------------------------------------------------
HighInterrupt
        movff   STATUS,STATUS_HIGH          ;save STATUS register
        movff   WREG,WREG_HIGH              ;save working register
        movff   BSR,BSR_HIGH                ;save BSR register
        movff   FSR0L,FSR0L_HIGH            ;save other registers used in high int
        movff   FSR0H,FSR0H_HIGH
        movff   FSR1L,FSR1L_HIGH
        movff   FSR1H,FSR1H_HIGH

    ;main firmware ready flag
        movlb   0x1
        btfss   FIRMREADY,0
        bra     ExitHighInterrupt           ;main firmware is not ready yet
    ;Timer0
        btfsc   INTCON,TMR0IF               ;Timer0 interrupt? (1000ms)
        rcall   Timer0Interrupt
    ;Timer2    
        btfsc   PIR1,TMR2IF                    ;Timer2 interrupt? (20ms)
        rcall   Timer2Interrupt

ExitHighInterrupt
        movff   BSR_HIGH,BSR                ;restore BSR register
        movff   WREG_HIGH,WREG              ;restore working register
        movff   STATUS_HIGH,STATUS          ;restore STATUS register
        movff   FSR0L_HIGH,FSR0L            ;restore other registers used in high int
        movff   FSR0H_HIGH,FSR0H
        movff   FSR1L_HIGH,FSR1L
        movff   FSR1H_HIGH,FSR1H
    return

;------------------------------------------------------------------------------
; Routine:          CAN INTERRUPT
;------------------------------------------------------------------------------
; Overview:         Checks CAN message for response and RTR and saves to FIFO
;------------------------------------------------------------------------------
CANInterrupt
        banksel CANFRAME2
        btfsc   CANFRAME2,0                 ;response message?
    return                                  ;yes, so ignore it and exit
        btfsc   CANFRAME2,1                 ;RTR (Remote Transmit Request)?
    return                                  ;yes, so ignore it and exit
        call    Copy_RXB_RXFIFOIN           ;copies received message to CAN RX FIFO input buffer
        call    WriteToCanRxFIFO            ;saves message to FIFO
    return

;------------------------------------------------------------------------------
; Routine:          TIMER 0 INTERRUPT
;------------------------------------------------------------------------------
; Overview:         1000ms periodical interrupt
;------------------------------------------------------------------------------
Timer0Interrupt:
        call    Timer0Initialization8MHz    ;restart 1000ms Timer   
        call    UpdateUpTime                ;counts time from restart
        call    UpdateTransmitTimer         ;increment transmit timer (seconds after last transmission)
        banksel TIMER0_1000ms
        setf    TIMER0_1000ms               ;timer 0 interrupt occurred flag
    return

;------------------------------------------------------------------------------
; Routine:          TIMER 2 INTERRUPT
;------------------------------------------------------------------------------
; Overview:         20ms periodical interrupt
;------------------------------------------------------------------------------
Timer2Interrupt
        rcall   Timer2Initialization        ;restart timer
        rcall   SetLeds                     ;update LEDs states
        rcall   CnvrtTime                   ;counts temp conversion time
        banksel TIMER2_20ms
        setf    TIMER2_20ms                 ;timer 2 interrupt occurred flag
    return
;-------------------------------
Timer2Initialization
        movlb   0xF
        bcf     PMD1,TMR2MD                 ;enable timer 2
        movlw   0x3F          
        movwf   TMR2                        ;set 20ms (19.999500)
        movlw   b'01001111'                 ;start timer, prescaler=16, postscaler=10
        movwf   T2CON
        bsf     IPR1,TMR2IP                 ;high priority for interrupt
        bcf     PIR1,TMR2IF                 ;clear timer's flag
        bsf     PIE1,TMR2IE                 ;interrupt on
    return
;-------------------------------
SetLeds
        ;move right led status
        movlw   b'11000000'                 ;clear only LED outputs
        andwf   LATC
        movlw   b'00111111'                 ;take only 6 bits of LED states register
        andwf   DiodesA,W
        iorwf   LATC                        ;move new LED states
    return
;-------------------------------
CnvrtTime
        setf    WREG
        xorwf   CNVRTFLAG,W                 ;if flag = FF do not change
        bz      $ + 6
        tstfsz  CNVRTFLAG                   ;decrement conversion in progress flag
        decf    CNVRTFLAG
    return


;==============================================================================
;===  MAIN PROGRAM  ===========================================================
;==============================================================================
Main:
    ;disable global interrupts for startup
        call    DisAllInt                   ;disable all interrupts 
    ;firmware initialization
        rcall   PortInitialization          ;prepare processor ports
        call    GeneralInitialization       ;read eeprom config, clear other registers
        call    FIFOInitialization          ;prepare FIFO buffers
        call    Timer0Initialization32MHz   ;Timer 0 initialization for 1s periodical interrupt 
        call    Timer2Initialization        ;Timer 2 initialization for 20ms periodical interrupt
        call    ButtonPowerUpValues         ;button on power up values
        call    mTouch_Initialization       ;touch button initialization & calibration
        call    TempPowerUpValues           ;temperature on power up values
    ;firmware ready
        movlb   0x1
        bsf     FIRMREADY,0                 ;set flag "firmware started and ready for interrupts"
    ;enable global interrupts
        call    EnAllInt                    ;enable all interrupts

;-------------------------------
Loop:                                       ;main loop
        clrwdt                              ;clear Watchdog timer
        call    ReceiveProcedure            ;check if any msg in RX FIFO and if so - process the msg
        call    TransmitProcedure           ;check if any msg in TX FIFO and if so - transmit it
        rcall   OnceA20ms                   ;do routines only after 20ms interrupt 
        rcall   OnceA1000ms                 ;do routines only after 1000ms interrupt
    bra     Loop

;-------------------------------
OnceA20ms                                   ;procedures executed once per 1000ms (flag set in interrupt)
        banksel TIMER2_20ms
        tstfsz  TIMER2_20ms                 ;flag set?
        bra     $ + 4
    return                                  ;no, so exit
        call    ReadmTouchButtons           ;do mTuch procedures
        call    RecognizeButtons            ;recognize what button is pressed and for what period
        banksel TIMER2_20ms
        clrf    TIMER2_20ms
    return
;-------------------------------
OnceA1000ms                                 ;procedures executed once per 1000ms (flag set in interrupt)
        banksel TIMER0_1000ms
        tstfsz  TIMER0_1000ms               ;flag set?
        bra     $ + 4
    return                                  ;no, so exit
        call    ReadTemperature             ;read ds sensor
        call    CnvrtTemperature            ;do conversion in DS sensor
        call    SaveSateToEeprom            ;save thermostat value into eeprom memory when needed
        call    mTouch_UpdateBaseline       ;update surrunding
        call    UpdateHealthRegs            ;saves health maximums to eeprom
        banksel TIMER0_1000ms
        clrf    TIMER0_1000ms
    return


;==============================================================================
;===  FIRMWARE ROUTINES  ======================================================
;==============================================================================
;------------------------------------------------------------------------------
; Routine:          PORT INITIALIZATION
;------------------------------------------------------------------------------
; Overview:         It sets processor pins. All unused pins should be set as
;                   outputs and driven low
;------------------------------------------------------------------------------
PortInitialization                          ;default all pins are set as analog (portA,B) or digital (portB,C) inputs 
    ;PORT A
        banksel ANCON0                      ;select memory bank
        ;0-digital, 1-analog input
        movlw   b'11100011'                 ;(x,x,x,AN4,AN3,AN2,AN1-boot_mode,AN0-volt)
        movwf   ANCON0
        ;output level
        clrf    LATA                        ;all low
        ;0-output, 1-input
        movlw   b'00000011'                 ;all output except AN0 & AN1
        movwf   TRISA       
    ;PORT B
        ;0-digital, 1-analog input
        movlw   b'11111000'                 ;(x,x,x,x,x,AN10,AN9,AN8)
        movwf   ANCON1
        ;output level
        clrf    LATB                        ;all low
        ;0-output, 1-input
        movlw   b'00001000'                 ;all output except CANRX
        movwf   TRISB
    ;PORT C
        ;output level
        movlw   b'10000000'                 ;all low except active guard for mTouch button
        movwf   LATC   
        ;0-output, 1-input
        movlw   b'00000000'                 ;all output
        movwf   TRISC
    return

;------------------------------------------------------------------------------
; Routine:          NODE STATUS
;------------------------------------------------------------------------------
; Overview:         It prepares status messages when status request was
;                   received
;------------------------------------------------------------------------------
NodeStatusRequest

;-Macro Buttons--------------------
ButtonStatus: MACRO ButNr,ButReg,ButVal,DiodeReg ;macro sends status for chosen button
        banksel TXFIFOIN0
        movlw   ButNr                       ;button x
        movwf   TXFIFOIN6
        ;button
        clrf    TXFIFOIN7                   ;0x00 - released
        movlw   ButVal                      ;one button pressed?
        xorwf   ButReg,W                    ;is ButReg = ButVal?
        bnz     $ + 4                       ;no, so skip next line
        setf    TXFIFOIN7                   ;0xFF - pressed
        movlw   0x3F                        ;all buttons pressed?
        xorwf   ButReg,W                    ;is ButReg = 0x3F?
        bnz     $ + 4                       ;no, so skip next line         
        setf    TXFIFOIN7                   ;0xFF - pressed
        ;diode
        setf    TXFIFOIN8                   ;0xFF - diode on
        btfss   DiodeReg,ButNr-1
        clrf    TXFIFOIN8                   ;0x00 - diode off
        setf    TXFIFOIN9                   ;unused
        setf    TXFIFOIN10                  ;unused
        setf    TXFIFOIN11                  ;unused
        rcall   SendButtonStatus
    ENDM
;-Macro Touch Buttons Parameters---
TouchStatus: MACRO ButNr,SampH,SampL,BaseH,BaseL,TrimBits ;macro sends status for chosen touch button
        banksel TXFIFOIN0
        movlw   ButNr                       ;button x
        movwf   TXFIFOIN6
        movff   SampH,TXFIFOIN7             ;SampleH
        movff   SampL,TXFIFOIN8             ;SampleL
        movff   BaseH,TXFIFOIN9             ;BaselineH
        movff   BaseL,TXFIFOIN10            ;BaselineL
        movff   TrimBits,TXFIFOIN11         ;Trimbits
        rcall   SendButtonStatus
    ENDM
;-Buttons--------------------
        ButtonStatus   0x01,ButtonsA,0x01,DiodesA  ;button 1, call macro, /macro_arg: Button_No, Button_REGISTER, Button_register_VALUE, Diode_REGISTER/
        ButtonStatus   0x02,ButtonsA,0x02,DiodesA  ;button 2
        ButtonStatus   0x03,ButtonsA,0x04,DiodesA  ;button 3
        ButtonStatus   0x04,ButtonsA,0x08,DiodesA  ;button 4
        ButtonStatus   0x05,ButtonsA,0x10,DiodesA  ;button 5
        ButtonStatus   0x06,ButtonsA,0x20,DiodesA  ;button 6
;-Touch Buttons Parameters---
        TouchStatus   0x21,mT_SAMPCHAN2H,mT_SAMPCHAN2L,mT_BASECHAN2H,mT_BASECHAN2L,mT_TRIMAN2      ;button 1, call macro, /macro_arg: ButNr,SAMPH,SAMPL,BASEH,BASEL,TRIMBITS/
        TouchStatus   0x22,mT_SAMPCHAN3H,mT_SAMPCHAN3L,mT_BASECHAN3H,mT_BASECHAN3L,mT_TRIMAN3      ;button 2
        TouchStatus   0x23,mT_SAMPCHAN4H,mT_SAMPCHAN4L,mT_BASECHAN4H,mT_BASECHAN4L,mT_TRIMAN4      ;button 3
        TouchStatus   0x24,mT_SAMPCHAN9H,mT_SAMPCHAN9L,mT_BASECHAN9H,mT_BASECHAN9L,mT_TRIMAN9      ;button 4
        TouchStatus   0x25,mT_SAMPCHAN10H,mT_SAMPCHAN10L,mT_BASECHAN10H,mT_BASECHAN10L,mT_TRIMAN10 ;button 5
        TouchStatus   0x26,mT_SAMPCHAN8H,mT_SAMPCHAN8L,mT_BASECHAN8H,mT_BASECHAN8L,mT_TRIMAN8      ;button 6
        bra     TempStatus
;------------
SendButtonStatus
        movlw   0x30                        ;set frame type
        movwf   TXFIFOIN0
        movlw   0x10
        movwf   TXFIFOIN1
        bsf     TXFIFOIN1,0                 ;response bit
        movff   NODENR,TXFIFOIN2            ;node id
        movff   GROUPNR,TXFIFOIN3
        setf    TXFIFOIN4                   ;unused
        setf    TXFIFOIN5                   ;unused
        call    WriteToCanTxFIFO
    return

;-Temperature----------------
TempStatus
        banksel TXFIFOIN0
        tstfsz  ERRORREG                    ;CheckIfError
    bra        SendDSError
        movlw   0x11                        ;"sensor 1"
        movwf   TXFIFOIN6                
        movf    TEMPMSB,W                   ;current temperature
        movwf   TXFIFOIN7
        movf    TEMPLSB,W
        movwf   TXFIFOIN8
        movff   THERMMSB,TXFIFOIN9          ;set thermostat temp
        movff   THERMLSB,TXFIFOIN10
        movff   HISTERE,TXFIFOIN11          ;histeresis
        rcall   SendTempStatus
ThermostatStatus:                           ;sends status after instruction
        movlw   0x12                        ;"thermostat 1"
        movwf   TXFIFOIN6
        movff   STHERMOS,TXFIFOIN7          ;thermostat status
        setf    TXFIFOIN8                   ;unused
        setf    TXFIFOIN9                   ;unused
        setf    TXFIFOIN10                  ;unused
        setf    TXFIFOIN11                  ;unused
        rcall   SendTempStatus
    return
SendDSError:
        movlw   0xF0                        ;0xF0 - ERROR FRAME
        movwf   TXFIFOIN6
        movff   ERRORREG,TXFIFOIN7          ;load error flag
        setf    TXFIFOIN8                   ;unused
        setf    TXFIFOIN9                   ;unused
        setf    TXFIFOIN10                  ;unused
        setf    TXFIFOIN11                  ;unused
        rcall   SendTempStatus
    return
SendTempStatus
        banksel TXFIFOIN0
        movlw   0x30                        ;set frame type
        movwf   TXFIFOIN0
        movlw   0x40
        movwf   TXFIFOIN1
        bsf     TXFIFOIN1,0                 ;set response bit
        movff   NODENR,TXFIFOIN2            ;node id
        movff   GROUPNR,TXFIFOIN3
        setf    TXFIFOIN4
        setf    TXFIFOIN5
        ;TXFIFOIN6 - TXFIFOIN11 are already changed
        call    WriteToCanTxFIFO
    return

;------------------------------------------------------------------------------
; Routine:            DO INSTRUCTION
;------------------------------------------------------------------------------
; Overview:            Executes instruction immediately or sets timer for later
;                   execution
;------------------------------------------------------------------------------
DoInstructionRequest
        banksel INSTR1                      ;allow only known values
        movlw   0x06                        ;INSTR less than?
        cpfslt  INSTR1
    bra     ExitDoInstructionRequest        ;no, so exit
        movf    INSTR1,W                    ;recognize instruction
        call    ComputedGoto                ;modify PC according to WREG value
        bra     Instr00                     ;instruction 00
        bra     Instr01                     ;instruction 01
        bra     Instr02                     ;instruction 02
        bra     Instr03                     ;instruction 03
        bra     Instr04                     ;instruction 04
        bra     Instr05                     ;instruction 05
ExitDoInstructionRequest
    return 

;-------------------------------
;Instruction execution
Instr00                                     ;turn off diode
        movf    INSTR2,W                    ;get mask of channels to change
        comf    WREG                        ;modify mask
        andwf   DiodesA,F
        bra     ExitDoInstructionRequest
;---------------
Instr01                                     ;turn on diode
        movf    INSTR2,W                    ;get mask of channels to change 
        iorwf   DiodesA,F
        bra     ExitDoInstructionRequest
;---------------
Instr02                                     ;toggle
        movf    INSTR2,W                    ;get mask of channels to change
        xorwf   DiodesA,F
        bra     ExitDoInstructionRequest
;---------------
Instr03                                     ;set THERMOSTAT to INSTR2,INSTR3
        movff   INSTR2,THERMMSB
        movff   INSTR3,THERMLSB
        rcall   EepromToSave
        call    TempStatus                  ;give quick response and send value of new thermos
        bra     ExitDoInstructionRequest
;---------------
Instr04                                     ;decrement THERM INSTR2 times (each time = 0.0625deg)
        movlw   0xFC                        ;min 0xFC90 = -55deg
        xorwf   THERMMSB,W
        bnz     $ + .8                      ;not yet
        movlw   0x90
        xorwf   THERMLSB,W
        bz      $ + .12                     ;it's max
        decf    THERMLSB                    ;decrement thermostat LSB
        bc      $ + 4                       ;branch if not borrow b=!c     
        decf    THERMMSB                    ;and borrow if underflowed
        decfsz  INSTR2
        bra     Instr04
        rcall   EepromToSave
        call    TempStatus                  ;give quick response and send value of new thermos
        bra     ExitDoInstructionRequest
;---------------
Instr05                                     ;increment THERM INSTR2 times (each time = 0.0625deg)
        movlw   0x07                        ;max 0x07D0 = 125deg
        xorwf   THERMMSB,W
        bnz     $ + .8                      ;not yet
        movlw   0xD0
        xorwf   THERMLSB,W
        bz      $ + .10                     ;it's max
        infsnz  THERMLSB                    ;increment termostat LSB
        incf    THERMMSB                    ;and MSB if LSB overflowed
        decfsz  INSTR2
        bra     Instr05
        rcall   EepromToSave
        call    TempStatus                    ;give quick response and send value of new thermos
        bra     ExitDoInstructionRequest

;---------------
EepromToSave                                ;indicate that save to eeprom nedded
        banksel EEPROMTIMER
        movlw   0x06                        ;wait 6s before saving to eeprom
        movwf   EEPROMTIMER
    return

;------------------------------------------------------------------------------
; Routine:          DO INSTRUCTION LATER
;------------------------------------------------------------------------------
; Overview:         It saves instruction for particular channel for later
;                   execution
;------------------------------------------------------------------------------
DoInstructionLater
    return

;==============================================================================
;                   mTOUCH PROCEDURES
;==============================================================================
;------------------------------------------------------------------------------
; Routine:          mTOUCH READ BUTTONS
;------------------------------------------------------------------------------
; Overview:         Read touch buttons 
;------------------------------------------------------------------------------
ReadmTouchButtons
        rcall   ReadmTouchButtonsAN2
        rcall   ReadmTouchButtonsAN3
        rcall   ReadmTouchButtonsAN4
        rcall   ReadmTouchButtonsAN8
        rcall   ReadmTouchButtonsAN9
        rcall   ReadmTouchButtonsAN10
    return

ReadmTouchButtonsAN2
        banksel mT_TRIMAN2
        tstfsz  mT_TRIMAN2                  ;don't read if calibration failed
        bra     $ + 4
        return
        movff   mT_TRIMAN2,CTMUICON         ;set calibrated current source
        call    mTouch_GetAvrgSampleChAN2   ;get sample
        call    mTouch_DecodeButtonStatusAN2 ;update button status
        banksel mT_BUTTONSTATUS
        bcf     ButtonsA,0                  ;match analog channels to button number AN2 -> button1
        btfsc   mT_BUTTONSTATUS,2
        bsf     ButtonsA,0
    return

ReadmTouchButtonsAN3
        banksel mT_TRIMAN3
        tstfsz  mT_TRIMAN3
        bra     $ + 4
        return
        movff   mT_TRIMAN3,CTMUICON
        call    mTouch_GetAvrgSampleChAN3
        call    mTouch_DecodeButtonStatusAN3
        banksel mT_BUTTONSTATUS
        bcf     ButtonsA,1                  ;AN3 -> button2
        btfsc   mT_BUTTONSTATUS,3
        bsf     ButtonsA,1
    return

ReadmTouchButtonsAN4
        banksel mT_TRIMAN4
        tstfsz  mT_TRIMAN4
        bra     $ + 4
        return
        movff   mT_TRIMAN4,CTMUICON
        call    mTouch_GetAvrgSampleChAN4
        call    mTouch_DecodeButtonStatusAN4
        banksel mT_BUTTONSTATUS
        bcf     ButtonsA,2                  ;AN4 -> button3
        btfsc   mT_BUTTONSTATUS,4
        bsf     ButtonsA,2
    return

ReadmTouchButtonsAN8
        banksel mT_TRIMAN8
        tstfsz  mT_TRIMAN8
        bra     $ + 4
        return
        movff   mT_TRIMAN8,CTMUICON
        call    mTouch_GetAvrgSampleChAN8
        call    mTouch_DecodeButtonStatusAN8
        banksel mT_BUTTONSTATUS
        bcf     ButtonsA,5                  ;AN8 -> button6
        btfsc   mT_BUTTONSTATUS,5
        bsf     ButtonsA,5
    return

ReadmTouchButtonsAN9
        banksel mT_TRIMAN9
        tstfsz  mT_TRIMAN9
        bra     $ + 4
        return
        movff   mT_TRIMAN9,CTMUICON
        call    mTouch_GetAvrgSampleChAN9
        call    mTouch_DecodeButtonStatusAN9
        banksel mT_BUTTONSTATUS
        bcf     ButtonsA,3                  ;AN9 -> button4
        btfsc   mT_BUTTONSTATUS,6
        bsf     ButtonsA,3
    return

ReadmTouchButtonsAN10
        banksel mT_TRIMAN10
        tstfsz  mT_TRIMAN10
        bra     $ + 4
        return 
        movff   mT_TRIMAN10,CTMUICON
        call    mTouch_GetAvrgSampleChAN10
        call    mTouch_DecodeButtonStatusAN10
        banksel mT_BUTTONSTATUS
        bcf     ButtonsA,4                  ;AN10 -> button5
        btfsc   mT_BUTTONSTATUS,7
        bsf     ButtonsA,4
    return


;==============================================================================
;                   BUTTON PROCEDURES
;==============================================================================
;------------------------------------------------------------------------------
; Routine:          BUTTON POWER UP VALUES
;------------------------------------------------------------------------------
; Overview:         Sets registers at power up 
;------------------------------------------------------------------------------
ButtonPowerUpValues
        clrf    ButtonsA                    ;buttons status as released
        clrf    DiodesA                     ;LEDs off              
        movlw   .6                          ;clear counters            
        lfsr    FSR0,BUT1Cnt
        clrf    POSTINC0
        decfsz  WREG
        bra     $ - 4
    return

;------------------------------------------------------------------------------
; Routine:          RECOGNIZE BUTTONS
;------------------------------------------------------------------------------
; Overview:         Recognizes which button is pressed and for how long.
;                   Routine also sends button message to the CAN bus. 
;------------------------------------------------------------------------------
RecognizeButtons
        call    Button1_ON
        call    Button1_OFF
        call    Button2_ON
        call    Button2_OFF
        call    Button3_ON
        call    Button3_OFF
        call    Button4_ON
        call    Button4_OFF
        call    Button5_ON
        call    Button5_OFF
        call    Button6_ON
        call    Button6_OFF
        call    Button7_ON
        call    Button7_OFF
    return

;----------------------------
Button_IncCnt:MACRO ButCnt            ;increment but don't overflow button counter
        incfsz  ButCnt
        bra     $ + 4
        decf    ButCnt
    ENDM
;------------
Button_Pressed:MACRO ButNr,ButCnt,DiodeReg,DiodeBit  ;counter equal 2 (40ms-button pressed)
        banksel BUTCnfg                     ;turn on in config?
        btfss   BUTCnfg+ButNr-1,0
    bra $ + .30                             ;no - go to macro end
        movlw   .2                          ;counter = 2?
        cpfseq  ButCnt
    bra $ + .24                             ;no - go to macro end
        banksel TXFIFOIN0
        movlw   ButNr                       ;set button number in msg to be sent
        movwf   TXFIFOIN6
        movlw   0xFF    
        movwf   TXFIFOIN7                   ;button code 0xFF - pressed          
        setf    WREG                        ;diode state 0xFF - on
        btfss   DiodeReg,DiodeBit
        clrf    WREG                        ;0x00 - off
        movwf   TXFIFOIN8
        call    TransmitButton
    ENDM
;------------
Button_400ms:MACRO ButNr,ButCnt,DiodeReg,DiodeBit   ;counter equal 20 (400ms-button pressed)
        banksel BUTCnfg                     ;turn on in config?
        btfss   BUTCnfg+ButNr-1,1
    bra $ + .30                             ;no - go to macro end
        movlw   .20                         ;counter =20?
        cpfseq  ButCnt                      ;skip if so
    bra $ + .24                             ;no - go to macro end
        banksel TXFIFOIN0
        movlw   ButNr                       ;set button number in msg to be sent
        movwf   TXFIFOIN6
        movlw   0xFE    
        movwf   TXFIFOIN7                   ;button code 0xFE - pressed for 400ms
        setf    WREG                        ;diode state 0xFF - on
        btfss   DiodeReg,DiodeBit
        clrf    WREG                        ;0x00 - off
        movwf   TXFIFOIN8
        call    TransmitButton
    ENDM
;------------
Button_4s:MACRO ButNr,ButCnt,DiodeReg,DiodeBit      ;counter equal 200 (4s-button pressed)
        banksel BUTCnfg                     ;turn on in config?
        btfss   BUTCnfg+ButNr-1,2
    bra $ + .30                             ;no - go to macro end
        movlw   .200                        ;counter =200?
        cpfseq  ButCnt                      ;skip if so
    bra $ + .24                             ;no - go to macro end
        banksel TXFIFOIN0
        movlw   ButNr                       ;set button number in msg to be sent
        movwf   TXFIFOIN6
        movlw   0xFD    
        movwf   TXFIFOIN7                   ;button code 0xFD - pressed for 4s
        setf    WREG                        ;diode state 0xFF - on
        btfss   DiodeReg,DiodeBit
        clrf    WREG                        ;0x00 - off
        movwf   TXFIFOIN8
        call    TransmitButton
    ENDM
;------------
Button_Released:MACRO ButNr,ButCnt,DiodeReg,DiodeBit    ;counter >2 (released after 20ms)
        banksel BUTCnfg                     ;turn on in config?
        btfss   BUTCnfg+ButNr-1,3
    bra $ + .32                             ;no - go to macro end
        movlw   .2                          ;counter >=2?
        cpfslt  ButCnt                      ;to send msg counter must be at least 2 to make sure "pressed msg" was send
        bra     $ + 4                       ;if counter <2 means button was in the same state, so do not send msg
    bra $ + .24                             ;no - go to macro end
        banksel TXFIFOIN0
        movlw   ButNr                       ;set button number in msg to be sent
        movwf   TXFIFOIN6
    movlw   0x00    
        movwf   TXFIFOIN7                   ;button code 0x00 - released          
        setf    WREG                        ;diode state 0xFF - on
        btfss   DiodeReg,DiodeBit
        clrf    WREG                        ;0x00 - off
        movwf   TXFIFOIN8
        call    TransmitButton
    ENDM
;------------
Button_60_400ms:MACRO ButNr,ButCnt,DiodeReg,DiodeBit    ;2 < counter < 20 (release before 400ms)
        banksel BUTCnfg                     ;turn on in config?
        btfss   BUTCnfg+ButNr-1,4
    bra $ + .38                             ;no - go to macro end
        movlw   .2                          ;counter >=2?
        cpfslt  ButCnt                      ;to send msg counter must be at least 2 to make sure "pressed msg" was send
        bra     $ + 4                       ;if counter <2 means button was in the same state, so do not send msg
    bra $ + .30                             ;no - go to macro end
        movlw   .20                         ;counter <20?
        cpfslt  ButCnt                      ;skip if so
    bra $ + .24                             ;no - go to macro end
        banksel TXFIFOIN0
        movlw   ButNr                       ;set button number in msg to be sent
        movwf   TXFIFOIN6
        movlw   0xFC    
        movwf   TXFIFOIN7                   ;button code 0xFC - released within 400ms
        setf    WREG                        ;diode state 0xFF - on
        btfss   DiodeReg,DiodeBit
        clrf    WREG                        ;0x00 - off
        movwf   TXFIFOIN8
        call    TransmitButton
    ENDM
;------------
Button_400_4s:MACRO ButNr,ButCnt,DiodeReg,DiodeBit  ;20 < counter < 200 (released between 400ms and 4s)
        banksel BUTCnfg                     ;turn on in config?
        btfss   BUTCnfg+ButNr-1,5
    bra $ + .38                             ;no - go to macro end
        movlw   .20                         ;counter >=20?
        cpfslt  ButCnt                      ;to send msg counter must be at least 20
        bra     $ + 4                       ;if not do not send msg
    bra $ + .30                             ;no - go to macro end
        movlw   .200                        ;counter <200?
        cpfslt  ButCnt                      ;skip if so
    bra $ + .24                             ;no - go to macro end
        banksel TXFIFOIN0
        movlw   ButNr                       ;set button number in msg to be sent
        movwf   TXFIFOIN6
        movlw   0xFB    
        movwf   TXFIFOIN7                   ;button code 0xFB - released within 4s
        setf    WREG                        ;diode state 0xFF - on
        btfss   DiodeReg,DiodeBit
        clrf    WREG                        ;0x00 - off
        movwf   TXFIFOIN8
        call    TransmitButton
    ENDM
;------------
Button_4s_infin:MACRO ButNr,ButCnt,DiodeReg,DiodeBit ;200 < counter < infinity (released after 4s)
        banksel BUTCnfg                     ;turn on in config?
        btfss   BUTCnfg+ButNr-1,6
    bra $ + .32                             ;no - go to macro end
        movlw   .200                        ;counter >=200?
        cpfslt  ButCnt                      ;to send msg counter must be at least 200
        bra     $ + 4                       ;if not do not send msg
    bra $ + .24                             ;no - go to macro end
        banksel TXFIFOIN0
        movlw   ButNr                       ;set button number in msg to be sent
        movwf   TXFIFOIN6
        movlw   0xFA    
        movwf   TXFIFOIN7                   ;button code 0xFA - released after 4s
        setf    WREG                        ;diode state 0xFF - on
        btfss   DiodeReg,DiodeBit
        clrf    WREG                        ;0x00 - off
        movwf   TXFIFOIN8
        call    TransmitButton
    ENDM
;------------
TransmitButton
        banksel TXFIFOIN0
        movlw   0x30                        ;set frame type
        movwf   TXFIFOIN0
        movlw   0x10
        movwf   TXFIFOIN1
        movff   NODENR,TXFIFOIN2            ;node id
        movff   GROUPNR,TXFIFOIN3
        setf    TXFIFOIN4                   ;unused
        setf    TXFIFOIN5                   ;unused
        ;(TXFIFOIN6 -TXFIFOIN8) are already changed in macro
        setf    TXFIFOIN9                   ;unused
        setf    TXFIFOIN10                  ;unused
        setf    TXFIFOIN11                  ;unused
        call    WriteToCanTxFIFO
    ;node can respond to its own message
        bcf     INTCON,GIEL                 ;disable low priority intr to make sure RXFIFO buffer is not overwritten
        call    Copy_TXFIFOIN_RXFIFOIN
        call    WriteToCanRxFIFO
        bsf     INTCON,GIEL                 ;enable back interrupt
    return

;------------
ButtonON:MACRO ButNr,ButCnt,DiodeReg,DiodeBit       ;do all needed routines when button is pressed
        Button_IncCnt   ButCnt                                 ;increment button counter      /macro_arg: Button_COUNTER/
        Button_Pressed  ButNr,ButCnt,DiodeReg,DiodeBit         ;button pressed?               /macro_arg: Button_No, Button_COUNTER, Diode_REGISTER, Diode_PIN/
        Button_400ms    ButNr,ButCnt,DiodeReg,DiodeBit         ;button held for 400ms?        /macro_arg: Button_No, Button_COUNTER, Diode_REGISTER, Diode_PIN/
        Button_4s       ButNr,ButCnt,DiodeReg,DiodeBit         ;button held for 4s?           /macro_arg: Button_No, Button_COUNTER, Diode_REGISTER, Diode_PIN/
    ENDM
;------------
ButtonOFF:MACRO ButNr,ButCnt,DiodeReg,DiodeBit       ;do all needed routines when button is released
        Button_Released ButNr,ButCnt,DiodeReg,DiodeBit         ;button released?              /macro_arg: Button_No, Button_COUNTER, Diode_REGISTER, Diode_PIN/
        Button_60_400ms ButNr,ButCnt,DiodeReg,DiodeBit         ;button released within 400ms? /macro_arg: Button_No, Button_COUNTER, Diode_REGISTER, Diode_PIN/
        Button_400_4s   ButNr,ButCnt,DiodeReg,DiodeBit         ;button released within 4s?    /macro_arg: Button_No, Button_COUNTER, Diode_REGISTER, Diode_PIN/
        Button_4s_infin ButNr,ButCnt,DiodeReg,DiodeBit         ;button released after 4s?     /macro_arg: Button_No, Button_COUNTER, Diode_REGISTER, Diode_PIN/
        clrf    ButCnt                                         ;reset counter
    ENDM

;----------button 1----------
Button1_ON:
        movlw   0x01                                ;only this button on?
        xorwf   ButtonsA,W
        bz      $ + 4
    return                                          ;no
        ButtonON   .1,BUT1Cnt,DiodesA,0             ;all routines for button ON /macro_arg: Button_No, Button_COUNTER, Diode_REGISTER, Diode_PIN/
    return
;------------
Button1_OFF:
        movlw   0x01                                ;button off?
        xorwf   ButtonsA,W
        bnz     $ + 4
    return                                          ;no
        ButtonOFF  .1,BUT1Cnt,DiodesA,0             ;all routines for button OFF /macro_arg: Button_No, Button_COUNTER, Diode_REGISTER, Diode_PIN/
    return
;----------button 2----------
Button2_ON:
        movlw   0x02
        xorwf   ButtonsA,W  
        bz      $ + 4               
    return                                       
        ButtonON   .2,BUT2Cnt,DiodesA,1        
    return
;------------
Button2_OFF:
        movlw   0x02
        xorwf   ButtonsA,W
        bnz     $ + 4                     
    return                                        
        ButtonOFF  .2,BUT2Cnt,DiodesA,1             
    return
;----------button 3----------
Button3_ON:
        movlw   0x04
        xorwf   ButtonsA,W        
        bz      $ + 4
    return                                     
        ButtonON   .3,BUT3Cnt,DiodesA,2            
    return
;------------
Button3_OFF:
        movlw   0x04
        xorwf   ButtonsA,W
        bnz     $ + 4                          
    return                                       
        ButtonOFF  .3,BUT3Cnt,DiodesA,2        
    return
;----------button 4----------
Button4_ON:
        movlw   0x08
        xorwf   ButtonsA,W  
        bz      $ + 4
    return
        ButtonON   .4,BUT4Cnt,DiodesA,3
    return
;------------
Button4_OFF:
        movlw   0x08
        xorwf   ButtonsA,W
        bnz     $ + 4  
    return
        ButtonOFF  .4,BUT4Cnt,DiodesA,3
    return
;----------button 5----------
Button5_ON:
        movlw   0x10
        xorwf   ButtonsA,W 
        bz      $ + 4 
    return
        ButtonON   .5,BUT5Cnt,DiodesA,4
    return
;------------
Button5_OFF:
        movlw   0x10
        xorwf   ButtonsA,W
        bnz     $ + 4  
    return
        ButtonOFF  .5,BUT5Cnt,DiodesA,4
    return
;----------button 6----------
Button6_ON:
        movlw   0x20
        xorwf   ButtonsA,W 
        bz      $ + 4  
    return
        ButtonON   .6,BUT6Cnt,DiodesA,5
    return
;------------
Button6_OFF:
        movlw   0x20
        xorwf   ButtonsA,W
        bnz     $ + 4  
    return
        ButtonOFF  .6,BUT6Cnt,DiodesA,5
    return
;----------button 7---------- (all buttons at the same time)
Button7_ON:
        movlw   0x3F
        xorwf   ButtonsA,W  
        bz      $ + 4  
    return
        ButtonON   .7,BUT7Cnt,DiodesA,6
    return
;------------
Button7_OFF:
        movlw   0x3F
        xorwf   ButtonsA,W  
        bnz     $ + 4  
    return
        ButtonOFF  .7,BUT7Cnt,DiodesA,6
    return

;==============================================================================
;                   TEMPERATURE PROCEDURES
;==============================================================================
;------------------------------------------------------------------------------
; Routine:          TEMPERATURE POWER UP VALUES
;------------------------------------------------------------------------------
; Overview:         Sets registers at power up 
;------------------------------------------------------------------------------
TempPowerUpValues                 
    ;config from eeprom
        ;thermostat value
        banksel THERMMSB
        movff   CONFIG13,THERMMSB           ;set switching temperature
        movff   CONFIG14,THERMLSB
        movlw   0x08                        ;if THERM = 0x0800 then take last saved
        xorwf   THERMMSB,W                  
        bnz     $ + .16
        movlw   0x00
        xorwf   THERMLSB,W                  
        bnz     $ + .10
        movff   CONFIG15,THERMMSB           ;last saved switching temperature
        movff   CONFIG16,THERMLSB
        ;histeresis
        banksel HISTEMSB
        movff   CONFIG17,HISTERE            ;<7:2>.<1:0> (2 bits after decimal point)
        movff   CONFIG17,HISTELSB        
        clrf    HISTEMSB                  
        bcf     STATUS,C                    ;rotate left to match decimal point of DS temperature
        rlcf    HISTELSB 
        rlcf    HISTEMSB
        rlcf    HISTELSB
        rlcf    HISTEMSB                    ;<MSB><LSB> = <000000hh><hhhh.hh00> h-histeresis from CONFIG, .- decimal point
        ;temperature offset
        movff   CONFIG18,OFFSETMSB          ;set temperature offset
        movff   CONFIG19,OFFSETLSB
        ;other values
        banksel TEMPLSB
        movlw   0x08                        ;previous temperature 0x0800
        movwf   STEMPLSB
        clrf    STEMPMSB
        movlw   0x80                        ;thermostat status not set yet
        movwf   STHERMOS
        setf    CNVRTFLAG                   ;will force to do temp A/D convertion first
    return
;------------------------------------------------------------------------------
; Routine:          TEMPERATURE A/D CONVERT 
;------------------------------------------------------------------------------
; Overview:         It does 1-wire initialization and if there is no error it
;                   sends "do temperature measurement" order to DS sensor
;------------------------------------------------------------------------------
CnvrtTemperature
        ;1-wire initialization
        call    DisAllInt                   ;disable interrupts
        call    InitiateDS                  ;initiate DS
        call    EnAllInt                    ;enable interrupts
        tstfsz  ERRORREG
    return                                  ;error on 1-wire bus indicated in ERRORREG
        ;convert request
        call    DisAllInt                   ;disable interrupts
        call    DSReset                     ;Send Reset Pulse and read for Presence Detect Pulse
        movlw   SKPROM                      ;Send Skip ROM Command (0xCC)
        call    DSTXByte                    
        movlw   0x44                        ;Send Convert T (0x44)
        call    DSTXByte
        call    EnAllInt                    ;enable interrupts
        ;strong  pull up on
        bsf     PORTDQ,DQ                   ;high level on DS
        bcf     TRISDQ,DQ                   ;pin as output
        ;flag
        movlw   .40                         ;wait 40x20ms=800ms to convert
        movwf   CNVRTFLAG                   ;conversion in progress flag
    return
;----------------------------
InitiateDS:
        ;presence
        call    DSReset                     ;Send Reset Pulse and read for Presence Detect Pulse
        btfsc   PDBYTE,0                    ;1 = Presence Detect Detected
        bra     CheckDSROM
        call    NoDevice
        bra     ExitInitiateDS
CheckDSROM                                  ;checks if there is only one device on network
        call    CheckROM
        tstfsz  WREG
        bra     $ + 4
        bra     ReadDSROM
        call    TooManyDevices
        bra     ExitInitiateDS
ReadDSROM                                   ;check CRC
        call    DSReset
        call    ReadROM
        tstfsz  WREG                        ;returned 1 means wrong CRC
        bra     $ +4
        bra     CheckDS
        call    WrongCRC
        bra     ExitInitiateDS        
CheckDS
        movlw   0x22                        ;DS1822
        xorwf   ROMCODE0,W
        bz      NoDSError
        movlw   0x28                        ;DS18B20
        xorwf   ROMCODE0,W
        bz      NoDSError
        call    WrongDevices
        bra     ExitInitiateDS
NoDSError
        clrf    ERRORREG                    ;no error
ExitInitiateDS
    return

;------------------------------------------------------------------------------
; Routine:            READ TEMPERATURE
;------------------------------------------------------------------------------
; Overview:            It reads temperature value from DS
;------------------------------------------------------------------------------
ReadTemperature
        tstfsz  CNVRTFLAG                   ;conversion in progress if flag!=0
        bra     ExitReadTemperature
        setf    WREG                        ;get temp done already?
        xorwf   CNVRTFLAG,W
        bz      ExitReadTemperature         ;yes, so do not do it
        ;request and read temperature
        call    DisAllInt                   ;disable interrupts
        rcall   GetTemp                     ;get temp
        call    EnAllInt                    ;enable interrupts
        tstfsz  ERRORREG
    return 
        setf    CNVRTFLAG                   ;make flag =FF meaning GetTemp done
        rcall   AddTempOffset               ;add temperature offset
        rcall   CompareTherm                ;compare thermostat value to temperature and send msg if needed
        rcall   CompareTemp                 ;compare temperature to previous one and send msg if needed
ExitReadTemperature
    return

;----------------------------
GetTemp
        ;temp request
        call    DSReset                     ;Send Reset Pulse and read for Presence Detect Pulse      
        movlw   SKPROM                      ;Send Skip ROM Command (0xCC)
        call    DSTXByte
        movlw   0xBE                        ;ReadScratchpap (0xBE)
        call    DSTXByte
        ;read temp
        clrf    CRCREG                      ;init CRC
        call    DSRXByte                    ;read LSB
        call    CalcCrc
        movf    IOBYTE,W
        movwf   TEMPLSB                 
        call    DSRXByte                    ;read MSB
        call    CalcCrc
        movf    IOBYTE,W
        movwf   TEMPMSB
        call    DSRXByte                    ;read TH
        call    CalcCrc
        call    DSRXByte                    ;read TL
        call    CalcCrc
        call    DSRXByte                    ;read CFG
        call    CalcCrc
        call    DSRXByte                    ;read RES
        call    CalcCrc
        call    DSRXByte                    ;read RES
        call    CalcCrc
        call    DSRXByte                    ;read RES
        call    CalcCrc
        call    DSRXByte                    ;read CRC
        call    CalcCrc
        tstfsz  CRCREG                      ;test if CRC correct
        bra     GotTempIsBad
        clrf    ERRORREG                    ;no error
    return                                  ;reading ok
GotTempIsBad
        call    WrongCRC
    return                                  ;error when reading DS

;----------------------------
AddTempOffset                               ;add temperature offset to current temperature
        movf    OFFSETLSB,W
        addwf   TEMPLSB,F
        movf    OFFSETMSB,W
        addwfc  TEMPMSB,F
    return

;----------------------------
CompareTherm                                ;compare temp to thermostat and send msg if needed
        banksel TEMPLSB
        ;check difference between current temperature and thermostat
        movf    TEMPLSB,W                   ;DIFF(TTEMP) = THERM - TEMP (difference = thermostat - current)     
        subwf   THERMLSB,W
        movwf   TTEMPLSB
        movf    TEMPMSB,W
        subwfb  THERMMSB,W
        movwf   TTEMPMSB
        ;result negative?
        bnn     $ + .10                     
        comf    TTEMPLSB                    ;yes, so change to positive
        comf    TTEMPMSB
        infsnz  TTEMPLSB
        incf    TTEMPMSB
        ;compare the difference with histeresis
        movf    HISTELSB,W                  ;TTEMP = DIFF(TTEMP) - HISTE (temporary temperature = difference - histeresis) 
        subwf   TTEMPLSB,F 
        movf    HISTEMSB,W
        subwfb  TTEMPMSB,F
    bn      ExitCompareTherm                ;current temp inside histeresis, so exit
        ;temp above or below thermostat
        movf    TEMPLSB,W                   ;THERM - TEMP
        subwf   THERMLSB,W
        movf    TEMPMSB,W
        subwfb  THERMMSB,W
        bnn     TxTLTherm                   ;TEMP < THERM, so send TL
        bn      TxTHTherm                   ;TEMP > THERM, so send TH
TxTLTherm:
        movlw   0x00                        ;check if TL was sent
        xorwf   STHERMOS,W
        bz      ExitCompareTherm            ;yes, so exit
        clrf    STHERMOS
        clrf    TXFIFOIN7
        bra     TxTherm
TxTHTherm:
        movlw   0xFF                        ;check if TH was sent
        xorwf   STHERMOS,W
        bz      ExitCompareTherm            ;yes, so exit
        setf    STHERMOS
        setf    TXFIFOIN7
        bra     TxTherm
TxTherm:
        movlw   0x12                        ;load data to transmit buffer
        movwf   TXFIFOIN6                   ;0x12 - "thermostat 1"
        setf    TXFIFOIN8
        setf    TXFIFOIN9                   ;unused
        setf    TXFIFOIN10                  ;unused
        setf    TXFIFOIN11                  ;unused
        call    TransmitTemp
ExitCompareTherm
    return

;----------------------------
CompareTemp                                 ;compare temp to previous one and send msg if different
        banksel TEMPLSB
        ;TTEMP = STEMP - TEMP (temporary temperature = previous - current)
        movf    TEMPLSB,W      
        subwf   STEMPLSB,W
        movwf   TTEMPLSB
        movf    TEMPMSB,W
        subwfb  STEMPMSB,W
        movwf   TTEMPMSB
        ;result negative?
        bnn     $ + .10                     
        comf    TTEMPLSB                    ;yes, so change to positive
        comf    TTEMPMSB
        infsnz  TTEMPLSB
        incf    TTEMPMSB
        ;check if diference between current and previous temperature is greater or equal 0.5deg
        tstfsz  TTEMPMSB,W
        bra     TxTemp                      ;temp difference greater than 1deg, so send msg        
        movlw   0x08                        ;difference greater or equal than 0.5deg?
        cpfslt  TTEMPLSB
        bra     TxTemp                      ;yes, so send msg        
    bra        ExitCompareTemp

TxTemp
        banksel TXFIFOIN0
        movlw   0x11                        ;load data to transmit buffer
        movwf   TXFIFOIN6                   ;0x11 - "temperature 1"
        movff   TEMPMSB,TXFIFOIN7
        movff   TEMPLSB,TXFIFOIN8
        movff   STHERMOS,TXFIFOIN9          ;set thermostat temp
        movff   HISTERE,TXFIFOIN10
        setf    TXFIFOIN11
        call    TransmitTemp
        movff   TEMPMSB,STEMPMSB            ;save MSB as previous value
        movff   TEMPLSB,STEMPLSB            ;save LSB as previous value
ExitCompareTemp
    return

;----------------------------
TransmitTemp
        banksel TXFIFOIN0
        movlw   0x30                        ;set frame type
        movwf   TXFIFOIN0
        movlw   0x40
        movwf   TXFIFOIN1
        movff   NODENR,TXFIFOIN2            ;node id
        movff   GROUPNR,TXFIFOIN3
        setf    TXFIFOIN4
        setf    TXFIFOIN5
        ;(TXFIFOIN6 -TXFIFOIN11) are already changed parent procedure
        call    WriteToCanTxFIFO
    ;node can respond to its own message
        bcf     INTCON,GIEL                 ;disable low priority intr to make sure RXFIFO buffer is not overwritten
        call    Copy_TXFIFOIN_RXFIFOIN
        call    WriteToCanRxFIFO
        bsf     INTCON,GIEL                 ;enable back interrupt
    return

;------------------------------------------------------------------------------
; Routine:            1-wire ERRORS
;------------------------------------------------------------------------------
; Overview:            It sets error type
;------------------------------------------------------------------------------
NoDevice:                                   ;no device on bus
        movlw   0x01
        movwf   ERRORREG
    return
;------------
TooManyDevices:                             ;too many devices on bus or wrong device (without 64bit rom)
        movlw   0x02                    
        movwf   ERRORREG
    return
;------------
WrongDevices:                               ;wrong device
        movlw    0x03
        movwf    ERRORREG
    return
;------------
WrongCRC:                                   ;CRC problem
        movlw    0x04
        movwf    ERRORREG
    return

;------------------------------------------------------------------------------
; Routine:            SAVE THERMOSTAT TO EEPROM
;------------------------------------------------------------------------------
; Overview:            It saves current thermostat value into EEPROM memory
;------------------------------------------------------------------------------
SaveSateToEeprom            
        banksel EEPROMTIMER
        ;wait 6s before saving
        tstfsz  EEPROMTIMER
        bra     $ + 4
        bra     ExitSaveStateToEeprom
        decfsz  EEPROMTIMER
        bra     ExitSaveStateToEeprom
        ;save to eeprom
        clrf    EEADRH                      ;point at high address
        movlw   low CONFIG15                ;point at low address    
        movwf   EEADR
        movf    THERMMSB,W                  ;set data
        call    EepromSaveWREG
        incf    EEADR                       ;point at next address
        movf    THERMLSB,W                  ;set data
        call    EepromSaveWREG
ExitSaveStateToEeprom
    return



;==============================================================================
;===  END  OF  PROGRAM  =======================================================
;==============================================================================
    END