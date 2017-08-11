;==============================================================================
;   HAPCAN - Home Automation Project Firmware (http://hapcan.com)
;   Copyright (C) 2017 hapcan.com
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
;   Filename:              univ_3-1-2-1.asm
;   Associated diagram:    univ_3-1-2-x.sch
;   Author:                Jacek Siwilo                          
;   Note:                  6 channel touch button & temp module
;==============================================================================
;   Revision History
;   Rev:  Date:     Details:
;   0     08.2017   Initial version
;==============================================================================
;===  FIRMWARE DEFINITIONS  =================================================== 
;==============================================================================
    #define    ATYPE    .1                            ;application type [0-255]
    #define    AVERS    .2                         ;application version [0-255]
    #define    FVERS    .1                            ;firmware version [0-255]

    #define    FREV     .0                         ;firmware revision [0-65536]
;==============================================================================
;===  NEEDED FILES  ===========================================================
;==============================================================================
    LIST P=18F26K80                              ;directive to define processor
    #include <P18F26K80.INC>           ;processor specific variable definitions
    #include "univ_3-1-2-1-rev0.inc"                         ;project variables
INCLUDEDFILES   code    
    #include "univ3-routines-rev7.inc"                     ;UNIV 3 CPU routines
    #include "univ3-mTouch-rev1.inc"                ;UNIV 3 CPU mTouch routines
    #include "univ3-1-wire-rev0.inc"                          ;1-wire functions

;==============================================================================
;===  FIRMWARE CHECKSUM  ======================================================
;==============================================================================
FIRMCHKSM   code    0x001000
    DB      0x58, 0xF2, 0xAC, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF
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
        rcall   DeactivateTouchButtons      ;count deactivation time and blink leds
        banksel TIMER0_1000ms
        setf    TIMER0_1000ms               ;timer 0 interrupt occurred flag
    return
;-------------------------------
DeactivateTouchButtons
        banksel mT_DEACTIVTIMER
        tstfsz  mT_DEACTIVTIMER
        bra     $ + .4
    return                                  ;just exit
        decfsz  mT_DEACTIVTIMER
        bra     $ + .8
        movff   mT_LEDSHADOW,DiodesA        ;restore previous led statuses
    return                                  
        comf    DiodesA                     ;blink diodes if deactivation
    return
;------------------------------------------------------------------------------
; Routine:          TIMER 2 INTERRUPT
;------------------------------------------------------------------------------
; Overview:         20ms periodical interrupt
;------------------------------------------------------------------------------
Timer2Interrupt
        rcall   Timer2Initialization8MHz    ;restart timer
        rcall   SetLeds                     ;update LEDs states
        rcall   CnvrtTime                   ;counts temp conversion time
        banksel TIMER2_20ms
        setf    TIMER2_20ms                 ;timer 2 interrupt occurred flag
    return
;-------------------------------
Timer2Initialization8MHz
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
        call    Timer0Initialization8MHz    ;Timer 0 initialization for 1s periodical interrupt 
        call    Timer2Initialization8MHz    ;Timer 2 initialization for 20ms periodical interrupt
        call    ButtonPowerUpValues         ;button on power up values
        call    mTouch_Initialization       ;touch button initialization & calibration
        call    ThermometerPowerUpValues    ;thermometer on power up values
    ;firmware ready
        banksel FIRMREADY
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
        call    ReadmTouchButtons           ;do mTouch procedures
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
        call    ThermometerProcedures       ;read ds sensor
        call    SaveStatesToEeprom          ;save values into eeprom memory when needed
        call    mTouch_UpdateBaseline       ;update surrounding
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
        ;button
        rcall   ButtonStatus
        ;thermometer
        banksel E_ENABLEDPER
        btfss   E_ENABLEDPER,ENABLETEMP     ;is temperature module enabled?
    return                                  ;no
        tstfsz  ERRORREG                    ;check if error
        bra     ThermometerError
        banksel E_ENABLEDPER
        btfsc   E_ENABLEDPER,ENABLETEMP     ;is temperature module enabled?
        rcall   TemperatureStatus
        banksel E_ENABLEDPER
        btfsc   E_ENABLEDPER,ENABLETHERM    ;is thermostat module enabled?
        rcall   ThermostatStatus
        banksel E_ENABLEDPER
        btfsc   E_ENABLEDPER,ENABLECTRL     ;is controller module enabled?
        rcall   ControllerStatus
    return    

;-Macro Buttons--------------------
ButtonStatusM: MACRO ButNr,ButReg,ButVal,DiodeReg ;macro sends status for chosen button
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
TouchStatusM: MACRO ButNr,SampH,SampL,BaseH,BaseL,TrimBits ;macro sends status for chosen touch button
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
;------------
ButtonStatus
;-Buttons--------------------
        ButtonStatusM   0x01,ButtonsA,0x01,DiodesA  ;button 1, call macro, /macro_arg: Button_No, Button_REGISTER, Button_register_VALUE, Diode_REGISTER/
        ButtonStatusM   0x02,ButtonsA,0x02,DiodesA  ;button 2
        ButtonStatusM   0x03,ButtonsA,0x04,DiodesA  ;button 3
        ButtonStatusM   0x04,ButtonsA,0x08,DiodesA  ;button 4
        ButtonStatusM   0x05,ButtonsA,0x10,DiodesA  ;button 5
        ButtonStatusM   0x06,ButtonsA,0x20,DiodesA  ;button 6
;-Touch Buttons Parameters---
        TouchStatusM   0x21,mT_SAMPCHAN2H,mT_SAMPCHAN2L,mT_BASECHAN2H,mT_BASECHAN2L,mT_TRIMAN2      ;button 1, call macro, /macro_arg: ButNr,SAMPH,SAMPL,BASEH,BASEL,TRIMBITS/
        TouchStatusM   0x22,mT_SAMPCHAN3H,mT_SAMPCHAN3L,mT_BASECHAN3H,mT_BASECHAN3L,mT_TRIMAN3      ;button 2
        TouchStatusM   0x23,mT_SAMPCHAN4H,mT_SAMPCHAN4L,mT_BASECHAN4H,mT_BASECHAN4L,mT_TRIMAN4      ;button 3
        TouchStatusM   0x24,mT_SAMPCHAN9H,mT_SAMPCHAN9L,mT_BASECHAN9H,mT_BASECHAN9L,mT_TRIMAN9      ;button 4
        TouchStatusM   0x25,mT_SAMPCHAN10H,mT_SAMPCHAN10L,mT_BASECHAN10H,mT_BASECHAN10L,mT_TRIMAN10 ;button 5
        TouchStatusM   0x26,mT_SAMPCHAN8H,mT_SAMPCHAN8L,mT_BASECHAN8H,mT_BASECHAN8L,mT_TRIMAN8      ;button 6
    return
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

;------thermometer-----------
TemperatureStatus
        banksel TXFIFOIN0
        movlw   0x11                        ;"sensor 1"
        movwf   TXFIFOIN6                
        movff   TEMPMSB,TXFIFOIN7           ;current temperature
        movff   TEMPLSB,TXFIFOIN8
        movff   SETPOINTMSB,TXFIFOIN9       ;setpoint temp
        movff   SETPOINTLSB,TXFIFOIN10
        movff   E_HYSTERE,TXFIFOIN11        ;hysteresis
        rcall   SendTempStatus
    return
;------------
ThermostatStatus
        banksel TXFIFOIN0
        movlw   0x12                        ;"thermostat 1"
        movwf   TXFIFOIN6
        movff   THERMOS,TXFIFOIN7           ;thermostat status
        setf    TXFIFOIN8                   ;unused
        setf    TXFIFOIN9                   ;unused
        setf    TXFIFOIN10                  ;unused
        clrf    TXFIFOIN11                  ;thermostat off
        btfsc   MODSTATE,MODTHERM           ;is thermostat on?
        setf    TXFIFOIN11                  ;yes
        rcall   SendTempStatus
    return
;------------
ControllerStatus
        banksel TXFIFOIN0
        movlw   0x13                        ;"controller 1"
        movwf   TXFIFOIN6                
        movff   TCHEATPWMS,TXFIFOIN7        ;heating pwm state
        movff   TCHEATPWMV,TXFIFOIN8        ;heating pwm value
        movff   TCCOOLPWMS,TXFIFOIN9        ;cooling pwm state
        movff   TCCOOLPWMV,TXFIFOIN10       ;cooling pwm value
        clrf    TXFIFOIN11                  ;controller off
        btfsc   MODSTATE,MODPWMCTRL         ;is PWM controller on?
        setf    TXFIFOIN11                  ;yes
        rcall   SendTempStatus
    return
;------------
ThermometerError
        banksel TXFIFOIN0
        movlw   0xF0                        ;0xF0 - ERROR FRAME
        movwf   TXFIFOIN6
        movff   ERRORREG,TXFIFOIN7          ;load error flag
        setf    TXFIFOIN8                   ;unused
        setf    TXFIFOIN9                   ;unused
        setf    TXFIFOIN10                  ;unused
        setf    TXFIFOIN11                  ;unused
        rcall   SendTempStatus
    return
;------------
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
; Routine:          DO INSTRUCTION
;------------------------------------------------------------------------------
; Overview:         Executes instruction immediately or sets timer for later
;                   execution
;------------------------------------------------------------------------------
DoInstructionRequest
        banksel INSTR1                      ;allow only known values
        movlw   0x0A                        ;INSTR less than?
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
        bra     Instr06                     ;instruction 06
        bra     Instr07                     ;instruction 07
        bra     Instr08                     ;instruction 08
        bra     Instr09                     ;instruction 09
ExitDoInstructionRequest
    return 


;-------------------------------
;Instruction execution
Instr00                                     ;turn off diode
        movf    INSTR2,W                    ;get mask of channels to change
        comf    WREG                        ;modify mask
        andwf   DiodesA,F
        bra     ExitDoInstructionRequest
Instr01                                     ;turn on diode
        movf    INSTR2,W                    ;get mask of channels to change 
        iorwf   DiodesA,F
        bra     ExitDoInstructionRequest
Instr02                                     ;toggle
        movf    INSTR2,W                    ;get mask of channels to change
        xorwf   DiodesA,F
        bra     ExitDoInstructionRequest
Instr03                                     ;SETPOINT from INSTR2,INSTR3
        movff   INSTR2,SETPOINTMSB
        movff   INSTR3,SETPOINTLSB
        rcall   EepromToSave
        ;send new setpoint
        tstfsz  ERRORREG                    ;error?
        bra     $ + .4                      ;yes
        bra     $ + .8
        call    ErrorTransmit               ;error, so exit
        bra     ExitDoInstructionNow
        call    TMTransmit                  ;send temperature
        bra     ExitDoInstructionNow
Instr04                                     ;decrement SETPOINT INSTR2 times (each time = 0.0625deg)
        movlw   0xFC                        ;min 0xFC90 = -55deg
        xorwf   SETPOINTMSB,W
        bnz     $ + .8                      ;not yet
        movlw   0x90
        xorwf   SETPOINTLSB,W
        bz      $ + .12                     ;it's max
        decf    SETPOINTLSB                 ;decrement setpoint LSB
        bc      $ + 4                       ;branch if not borrow b=!c     
        decf    SETPOINTMSB                 ;and borrow if underflow
        decfsz  INSTR2
        bra     Instr04
        rcall   EepromToSave        
        ;send new setpoint
        tstfsz  ERRORREG                    ;error?
        bra     $ + .4                      ;yes
        bra     $ + .8
        call    ErrorTransmit               ;error, so exit
        bra     ExitDoInstructionNow
        call    TMTransmit                  ;send temperature
        bra     ExitDoInstructionNow
Instr05                                     ;increment SETPOINT INSTR2 times (each time = 0.0625deg)
        movlw   0x07                        ;max 0x07D0 = 125deg
        xorwf   SETPOINTMSB,W
        bnz     $ + .8                      ;not yet
        movlw   0xD0
        xorwf   SETPOINTLSB,W
        bz      $ + .10                     ;it's max
        infsnz  SETPOINTLSB                 ;increment setpoint LSB
        incf    SETPOINTMSB                 ;and MSB if LSB overflowed
        decfsz  INSTR2
        bra     Instr05
        rcall   EepromToSave
        ;send new setpoint
        tstfsz  ERRORREG                    ;error?
        bra     $ + .4                      ;yes
        bra     $ + .8
        call    ErrorTransmit               ;error, so exit
        bra     ExitDoInstructionNow
        call    TMTransmit                  ;send temperature
        bra     ExitDoInstructionNow
Instr06                                     ;turn off module
        movf    INSTR2,W                    ;get mask of channels to change
        comf    WREG                        ;modify mask
        andwf   MODSTATENEW,F
        bra     ExitDoInstructionNow 
Instr07                                     ;turn on module
        movf    INSTR2,W                    ;get mask of channels to change
        iorwf   MODSTATENEW,F
        bra     ExitDoInstructionNow 
Instr08                                     ;toggle
        movf    INSTR2,W                    ;get mask of channels to change
        xorwf   MODSTATENEW,F
        bra     ExitDoInstructionNow 
Instr09                                     ;deactivate touch buttons for some period
        movff   INSTR2,mT_DEACTIVTIMER      ;get deactivation time
        movff   DiodesA,mT_LEDSHADOW        ;get current led statuses
        bra     ExitDoInstructionNow
ExitDoInstructionNow
        setf    INSTR1                      ;clear instruction
    return
;------------                
EepromToSave                                ;indicate that save to eeprom needed
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
        banksel mT_DEACTIVTIMER
        tstfsz  mT_DEACTIVTIMER
    return                                  ;exit because buttons are deactivated
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
;                   THERMOMETER PROCEDURES
;==============================================================================


;==============================================================================
;                   TEMPERATURE
;==============================================================================
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
        call    ReEnAllInt                    ;enable interrupts
        tstfsz  ERRORREG
    return                                  ;error on 1-wire bus indicated in ERRORREG
        ;convert request
        call    DisAllInt                   ;disable interrupts
        call    DSReset                     ;Send Reset Pulse and read for Presence Detect Pulse
        movlw   SKPROM                      ;Send Skip ROM Command (0xCC)
        call    DSTXByte                    
        movlw   0x44                        ;Send Convert T (0x44)
        call    DSTXByte
        call    ReEnAllInt                    ;enable interrupts
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
; Routine:          1-wire ERRORS
;------------------------------------------------------------------------------
; Overview:         It sets error type
;------------------------------------------------------------------------------
NoDevice:                                   ;no device on bus
        movlw   0x01
        movwf   ERRORREG
    return
TooManyDevices:                             ;too many devices on bus or wrong device (without 64bit rom)
        movlw   0x02                    
        movwf   ERRORREG
    return
WrongDevices:                               ;wrong device type
        movlw   0x03
        movwf   ERRORREG
    return
WrongCRC:                                   ;CRC problem
        movlw   0x04
        movwf   ERRORREG
    return

;------------------------------------------------------------------------------
; Routine:          READ TEMPERATURE
;------------------------------------------------------------------------------
; Overview:         It reads temperature value from DS
;------------------------------------------------------------------------------
ReadTemperature
        call    DisAllInt                   ;disable interrupts
        rcall   GetTemp                     ;get temp from DS
        call    ReEnAllInt                    ;enable interrupts
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

;------------------------------------------------------------------------------
; Routine:          ADD TEMPERATURE OFFSET
;------------------------------------------------------------------------------
; Overview:         Adds offset to temperature read from DS sensor
;------------------------------------------------------------------------------
AddTempOffset
        movf    OFFSETLSB,W
        addwf   TEMPLSB,F
        movf    OFFSETMSB,W
        addwfc  TEMPMSB,F
    return

;------------------------------------------------------------------------------
; Routine:          COMPARE TEMPERATURE
;------------------------------------------------------------------------------
; Overview:         It compares temperature to previous one and sends message 
;                   if different
;------------------------------------------------------------------------------
CompareTemp
        banksel TEMPLSB
        ;check difference between current and previous temperature
        movf    TEMPLSB,W                   ;TTEMP = TEMPPRE - TEMP (temporary register = previous - current)
        subwf   TEMPLSBPRE,W
        movwf   TTEMPLSB
        movf    TEMPMSB,W
        subwfb  TEMPMSBPRE,W
        movwf   TTEMPMSB
        ;result negative?
        bnn     $ + .10                     
        comf    TTEMPLSB                    ;yes, so change to positive
        comf    TTEMPMSB
        infsnz  TTEMPLSB
        incf    TTEMPMSB
        ;check if difference between current and previous temperature is greater or equal to defined minimum change
        banksel E_TEMPCHANGE
        movf    E_TEMPCHANGE,W              ;TTEMP = DIFF(TTEMP) - E_TEMPCHANGE (temporary register = difference - temp_change) 
        subwf   TTEMPLSB,F 
        clrf    WREG
        subwfb  TTEMPMSB,F
    bn      ExitCompareTemp                 ;temp difference is below the minimum change, so exit
        tstfsz  E_TEMPCHANGE                ;do not send if E_TEMPCHANGE = 0
        bra     TMTransmit
ExitCompareTemp
    return

;------------------------------------------------------------------------------
; Routine:          TRANSMIT TEMPERATURE
;------------------------------------------------------------------------------
; Overview:         It transmits temperature messages 0x11
;------------------------------------------------------------------------------
TMTransmit
        banksel TXFIFOIN0
        movlw   0x11                        ;load data to transmit buffer
        movwf   TXFIFOIN6                   ;0x11 - "temperature 1"
        movff   TEMPMSB,TXFIFOIN7
        movff   TEMPLSB,TXFIFOIN8
        movff   SETPOINTMSB,TXFIFOIN9       ;temperature setpoint
        movff   SETPOINTLSB,TXFIFOIN10
        movff   E_HYSTERE,TXFIFOIN11        ;hysteresis
        call    TransmitThermometer
        movff   TEMPMSB,TEMPMSBPRE          ;save MSB as previous value
        movff   TEMPLSB,TEMPLSBPRE          ;save LSB as previous value
TMTransmitLoadTimer
        ;load periodical counter
        banksel E_TMCOUNTER
        movlw   .60                         ;period [s] = 60*E_TMCOUNTER
        mulwf   E_TMCOUNTER
        movff   PRODH,TMCOUNTERH
        movff   PRODL,TMCOUNTERL
    return

;==============================================================================
;                   THERMOSTAT
;==============================================================================
;------------------------------------------------------------------------------
; Routine:          COMPARE THERMOSTAT
;------------------------------------------------------------------------------
; Overview:         It compares temperature to setpoint and sends message if
;                   needed
;------------------------------------------------------------------------------
CompareTherm
        banksel TEMPLSB
        ;check difference between current temperature and setpoint
        movf    TEMPLSB,W                   ;DIFF(TTEMP) = SETPOINT - TEMP (difference = setpoint - current)     
        subwf   SETPOINTLSB,W
        movwf   TTEMPLSB
        movf    TEMPMSB,W
        subwfb  SETPOINTMSB,W
        movwf   TTEMPMSB
        ;result negative?
        bnn     $ + .10                     
        comf    TTEMPLSB                    ;yes, so change to positive
        comf    TTEMPMSB
        infsnz  TTEMPLSB
        incf    TTEMPMSB
        ;compare the difference with hysteresis
        movf    HYSTELSB,W                  ;TTEMP = DIFF(TTEMP) - HYSTE (temporary temperature = difference - hysteresis) 
        subwf   TTEMPLSB,F 
        movf    HYSTEMSB,W
        subwfb  TTEMPMSB,F
    bn      ExitCompareTherm                ;current temp inside hysteresis, so exit
        ;temp above or below setpoint
        movf    TEMPLSB,W                   ;SETPOINT - TEMP
        subwf   SETPOINTLSB,W
        movf    TEMPMSB,W
        subwfb  SETPOINTMSB,W
        bnn     TxTLTherm                   ;TEMP < SETPOINT, so send TL
        bn      TxTHTherm                   ;TEMP > SETPOINT, so send TH
TxTLTherm
        movlw   0x00                        ;check if TL was sent
        xorwf   THERMOS,W
        bz      ExitCompareTherm            ;yes, so exit
        clrf    THERMOS
        bra     TxTherm
TxTHTherm
        movlw   0xFF                        ;check if TH was sent
        xorwf   THERMOS,W
        bz      ExitCompareTherm            ;yes, so exit
        setf    THERMOS
        bra     TxTherm
TxTherm
        rcall   THTransmit

ExitCompareTherm
    return                                

;------------------------------------------------------------------------------
; Routine:          TRANSMIT THERMOSTAT FRAME
;------------------------------------------------------------------------------
THTransmit
        banksel TXFIFOIN6
        movlw   0x12                        ;load data to transmit buffer
        movwf   TXFIFOIN6                   ;0x12 - "thermostat 1"
        movff   THERMOS,TXFIFOIN7           ;thermostat state
        setf    TXFIFOIN8                   ;unused
        setf    TXFIFOIN9                   ;unused
        setf    TXFIFOIN10                  ;unused
        clrf    TXFIFOIN11                  ;thermostat off
        btfsc   MODSTATE,MODTHERM           ;is thermostat on?
        setf    TXFIFOIN11                  ;yes
        call    TransmitThermometer
THTransmitLoadTimer
        ;load periodical counter
        banksel E_THCOUNTER
        movlw   .60                         ;period [s] = 60*E_THCOUNTER
        mulwf   E_THCOUNTER
        movff   PRODH,THCOUNTERH
        movff   PRODL,THCOUNTERL
    return

;==============================================================================
;                   TEMPERATURE CONTROLLER
;==============================================================================
;------------------------------------------------------------------------------
; Routine:          COMPARE TEMPERATURE TO CONTROLLER
;------------------------------------------------------------------------------
; Overview:         It compares controller settings to current temperature and
;                   does PWM regulation
;------------------------------------------------------------------------------
CompareContr
        banksel TCERRLSB
        ;calculate controller error (difference between setpoint and current temperature)
        movf    TEMPLSB,W                   ;TCERR = SETPOINT - TEMP (error = setpoint - current temp)     
        subwf   SETPOINTLSB,W
        movwf   TCERRLSB
        movf    TEMPMSB,W
        subwfb  SETPOINTMSB,W
        movwf   TCERRMSB
        ;define controller state
        iorwf   TCERRLSB,W                  ;ior TCERRLSB with TCERRMSB
        btfsc   STATUS,Z                    ;are they zero?
        bra     TCErrorZero                 ;yes - controller goes to idle state
        btfss   TCERRMSB,7                  ;error positive?
        bra     TCErrorPositive             ;yes - set heating values
        bra     TCErrorNegative             ;no - set cooling values
ExitCompareContr
        rcall   TCPwmControl                ;takes care of turning On & OFF the PWM
    return

;----------------------------
TCErrorZero                                 ;idle state
    ;test if controller idle state was sent already
        movf    TCERRLSBPRE
        iorwf   TCERRMSBPRE
        btfsc   STATUS,Z                    ;both regs equal zero?
    bra     ExitCompareContr                ;yes, so state was sent
    ;send controller idle state
        clrf    TCHEATPWMS                  ;heating pwm state "OFF"
        clrf    TCHEATPWMV                  ;heating pwm value
        clrf    TCCOOLPWMS                  ;cooling pwm state "OFF"
        clrf    TCCOOLPWMV                  ;cooling pwm value
        bcf     TCFLAGS,HeatState           ;disable heating
        bcf     TCFLAGS,CoolState           ;disable cooling
        rcall   TxPwmCtrl
    ;save current error
        movff   TCERRMSB,TCERRMSBPRE        ;save MSB error as previous value
        movff   TCERRLSB,TCERRLSBPRE        ;save LSB error as previous value
    bra     ExitCompareContr

;--------------
TCErrorPositive                             ;heating
    ;test if controller state changed
        rcall   TCGetDeltaError             ;get delta value and test against controller sensitivity
        tstfsz  WREG                        ;delta equal or greater than sensitivity?
    bra     ExitCompareContr                ;delta lesser than sensitivity, so exit
    ;read table
        movlw   upper PWMHEATTAB1           ;access beginning of table
        movwf   TBLPTRU
        movlw   high PWMHEATTAB1
        movwf   TBLPTRH
        movlw   low PWMHEATTAB1
        movwf   TBLPTRL
        ;reduce error to match table size
        tstfsz  TCERRMSB                    ;error greater than controller table size?
        bra     $ + 8                       ;yes, so take last value from table
        movlw   .32      
        cpfsgt  TCERRLSB        
        bra     $ + 6                       ;no
        movlw   .32                         ;reduce error to maximum table size
        bra     $ + 4
        movf    TCERRLSB,W                  ;put error value
        addwf   TBLPTRL,F
        decf    TBLPTRL,F                   ;error in table starts from 0,0625 not 0, so decrement table value
        tblrd*                              ;read table
        movff   TABLAT,TCHEATPWMV           ;move value from table
    ;set controller heating state
        clrf    TCHEATPWMS
        tstfsz  TCHEATPWMV                  ;pwm duty cycle value = zero?, so pwm state must be 0x00
        setf    TCHEATPWMS                  ;heating pwm state "ON"
        clrf    TCCOOLPWMS                  ;cooling pwm state
        clrf    TCCOOLPWMV                  ;cooling pwm value
        bcf     TCFLAGS,CoolState           ;disable cooling
        bsf     TCFLAGS,HeatState           ;indicate heating
        rcall   TCPwmPeriodEnd              ;reload PWM according to new values and send message
    ;save current error
        movff   TCERRMSB,TCERRMSBPRE        ;save MSB error as previous value
        movff   TCERRLSB,TCERRLSBPRE        ;save LSB error as previous value
    bra     ExitCompareContr

;--------------
TCErrorNegative                             ;cooling
    ;test if controller state changed
        rcall   TCGetDeltaError             ;get delta value and test against controller sensitivity
        tstfsz  WREG                        ;delta equal or greater than sensitivity?
    bra     ExitCompareContr                ;delta lesser than sensitivity, so exit
    ;change error to positive
        comf    TCERRLSB,W
        movwf   TCWORKLSB                    
        comf    TCERRMSB,W
        movwf   TCWORKMSB
        infsnz  TCWORKLSB
        incf    TCWORKMSB
    ;read table
        movlw   upper PWMCOOLTAB1           ;access beginning of table
        movwf   TBLPTRU
        movlw   high PWMCOOLTAB1
        movwf   TBLPTRH
        movlw   low PWMCOOLTAB1
        movwf   TBLPTRL
        ;reduce error to match table size
        tstfsz  TCWORKMSB                   ;error greater than controller table?
        bra     $ + 8                       ;yes, so take last value from table
        movlw   .32      
        cpfsgt  TCWORKLSB        
        bra     $ + 6                       ;no
        movlw   .32                         ;reduce error to maximum table size
        bra     $ + 4
        movf    TCWORKLSB,W                 ;put error value
        addwf   TBLPTRL,F
        decf    TBLPTRL,F                   ;error in table starts from 0,0625 not 0, so decrement table value
        tblrd*                              ;read table
        movff   TABLAT,TCCOOLPWMV           ;move value from table
    ;set controller cooling state
        clrf    TCCOOLPWMS
        tstfsz  TCCOOLPWMV                  ;pwm duty cycle value = zero?, so pwm state must be 0x00
        setf    TCCOOLPWMS                  ;cooling pwm state "ON"
        clrf    TCHEATPWMS                  ;heating pwm state
        clrf    TCHEATPWMV                  ;heating pwm value
        bcf     TCFLAGS,HeatState           ;disable heating
        bsf     TCFLAGS,CoolState           ;indicate cooling
        rcall   TCPwmPeriodEnd              ;reload PWM according to new values and send message
    ;save current error
        movff   TCERRMSB,TCERRMSBPRE        ;save MSB error as previous value
        movff   TCERRLSB,TCERRLSBPRE        ;save LSB error as previous value
    bra     ExitCompareContr

;----------------------------
TCGetDeltaError                             ;get delta value (difference between previous and current error) and test against controller sensitivity
    ;calculate delta error
        movf    TCERRLSB,W                  ;TCDERR = STCERR - TCERR (delta error = previous error - current error)     
        subwf   TCERRLSBPRE,W
        movwf   TCDERRLSB
        movf    TCERRMSB,W
        subwfb  TCERRMSBPRE,W
        movwf   TCDERRMSB
        ;result negative?
        bnn     $ + .10 
        comf    TCDERRLSB                   ;yes, so change to positive
        comf    TCDERRMSB
        infsnz  TCDERRLSB
        incf    TCDERRMSB
    ;test against sensitivity               ;ONLY NEGATIVE RESULT MEANS "DELTA WITHIN SENSITIVITY"
                                            ;in fact TCSENS=0 means 0.0625deg etc, so TCSENS(=0)-TCDERR(=1)(meaning 0.0625) gives negative result
        movf    TCDERRLSB,W                 ;result = TCSENS - TCDERR (result =  sensitivity - delta error)
        subwf   TCSENS,W
        movlw   0x00                        ;msb byte of TCSENS = 0
        subfwb  TCDERRMSB,W
        btfss   WREG,7                      ;result positive?
    retlw   0x01                            ;yes - delta is lesser than sensitivity, so exit with error
    retlw   0x00                            ;no - delta is equal or greater than sensitivity, so exit without error

;------------------------------------------------------------------------------
TCPwmControl
        btfsc   TCFLAGS,PwmOnEnd
        rcall   TCPwmOnEnd
        btfsc   TCFLAGS,PwmPeriodEnd
        rcall   TCPwmPeriodEnd   
    return

;----------------------------
;End of PWM ON event
TCPwmOnEnd
        bcf     TCFLAGS,PwmOnEnd            ;clear flag
        btfsc   TCFLAGS,HeatState           ;heat state?
        bra     TCPwmOnEndHeat              ;yes
        btfsc   TCFLAGS,CoolState           ;cool state?
        bra     TCPwmOnEndCool              ;yes
    return
TCPwmOnEndHeat
        movlw   0xFF                        ;PWM = 255
        xorwf   TCHEATPWMV,W
        bnz     $ + 4
    return                                  ;yes, so do not send "PWM 0" state
        clrf    TCHEATPWMS                  ;HEAT PWM state = 0
        rcall   TxPwmCtrl                   ;send controller msg
    return
TCPwmOnEndCool 
        movlw   0xFF                        ;PWM = 255
        xorwf   TCCOOLPWMV,W
        bnz     $ + 4
    return                                  ;yes, so do not send "PWM 0" state
        clrf    TCCOOLPWMS                  ;COOL PWM state = 0
        rcall   TxPwmCtrl                   ;send controller msg
    return
;--------------
;End of PWM Period event
TCPwmPeriodEnd
        bcf     TCFLAGS,PwmPeriodEnd        ;clear flag
        rcall   TCPwmPeriodLoad             ;reload period time
        btfsc   TCFLAGS,HeatState           ;heat state?
        bra     TCPwmPeriodEndHeat          ;yes
        btfsc   TCFLAGS,CoolState           ;cool state?
        bra     TCPwmPeriodEndCool          ;yes
    return
TCPwmPeriodEndHeat
        movf    TCHEATPWMV,W                ;choose PWM reg and load PWM ON time
        rcall   TCPwmOnLoad
        movf    TCPWMONLSB,W                ;the PWM ON time is = 0?
        iorwf   TCPWMONMSB,W
        bz      $ + 6                       ;yes, so do not change state nor send msg
        setf    TCHEATPWMS                  ;HEAT PWM state = 1
        rcall   TxPwmCtrl                   ;send controller msg
    return
TCPwmPeriodEndCool
        movf    TCCOOLPWMV,W                ;choose PWM reg and load PWM ON time
        rcall   TCPwmOnLoad
        movf    TCPWMONLSB,W                ;the PWM ON time is = 0?
        iorwf   TCPWMONMSB,W
        bz      $ + 6                       ;yes, so do not change state nor send msg
        setf    TCCOOLPWMS                  ;COOL PWM state = 1
        rcall   TxPwmCtrl                   ;send controller msg
    return

;----------------------------
TCPwmPeriodLoad                             ;loads PWM period in seconds    
        banksel TCPWMPERIOD                 ;PWM period [s] = 60*TCPWMPERIOD+60, "+60" because TCPWMPERIOD=0 means 60s
        movlw   .60                         ;
        mulwf   TCPWMPERIOD
        movff   PRODH,TCPWMPEMSB
        movff   PRODL,TCPWMPELSB
        addwf   TCPWMPELSB                  ;+60
        btfsc   STATUS,C                    ;overflow?
        incf    TCPWMPEMSB                  ;yes
    return
;--------------
TCPwmOnLoad                                 ;loads PWM ON time in seconds
;!!!!!! PWM reg TCHEATPWMV or TCCOOLPWMV must be moved to WREG before calling this routine
        banksel TCHEATPWMV                  ;PWM ON time [s] = PwmPeriod*DutyCycle = (60*TCPWMPERIOD+60)*TCHEATPWM/256 =
        clrf    TCPWMONMSB                  ;                = 60*(TCHEATPWM*TCPWMPERIOD+TCHEATPWM)/256
        clrf    TCPWMONLSB
        mulwf   TCPWMPERIOD                 ;TCPWMPERIOD*TCHEATPWM
        movff   PRODH,TCWORKMSB
        movff   PRODL,TCWORKLSB     
        addwf   TCWORKLSB                   ;TCHEATPWM*TCPWMPERIOD+TCHEATPWM
        btfsc   STATUS,C
        incf    TCWORKMSB
        movlw   .60                         ;60*LSB(TCHEATPWM*TCPWMPERIOD+TCHEATPWM)
        mulwf   TCWORKLSB
        movff   PRODH,TCPWMONMSB
        movff   PRODL,TCPWMONLSB
        clrf    TCWORKLSB 
        movlw   .60                         ;60*MSB(TCHEATPWM*TCPWMPERIOD+TCHEATPWM)
        mulwf   TCWORKMSB
        movf    PRODL,W
        addwf   TCPWMONMSB,F
        movf    PRODH,W
        addwfc  TCWORKLSB,F
        movff   TCPWMONMSB,TCPWMONLSB       ;/256
        movff   TCWORKLSB,TCPWMONMSB
    return    
;----------------------------
TxPwmCtrl
        ;was it sent already?
        movf    TCHEATPWMVPRE,W
        xorwf   TCHEATPWMV,W
        bnz     TxPwmCtrlNow
        movf    TCCOOLPWMVPRE,W
        xorwf   TCCOOLPWMV,W
        bnz     TxPwmCtrlNow
        movf    TCHEATPWMSPRE,W
        xorwf   TCHEATPWMS,W
        bnz     TxPwmCtrlNow
        movf    TCCOOLPWMSPRE,W
        xorwf   TCCOOLPWMS,W
        bnz     TxPwmCtrlNow
    return                                  ;no, so send it
TxPwmCtrlNow
        movff   TCHEATPWMV,TCHEATPWMVPRE    ;update previous value registers
        movff   TCCOOLPWMV,TCCOOLPWMVPRE
        movff   TCHEATPWMS,TCHEATPWMSPRE
        movff   TCCOOLPWMS,TCCOOLPWMSPRE
        rcall   TCTransmit
    return  

;------------------------------------------------------------------------------
; Routine:          TRANSMIT CONTROLLER FRAME
;------------------------------------------------------------------------------
; Overview:         It transmits controller messages 0x13
;------------------------------------------------------------------------------
TCTransmit                                  ;temperature controller transmit
        banksel TXFIFOIN0
        movlw   0x13                        ;load data to transmit buffer
        movwf   TXFIFOIN6                   ;0x13 - "temperature controller"
        movff   TCHEATPWMS,TXFIFOIN7        ;heating pwm state
        movff   TCHEATPWMV,TXFIFOIN8        ;heating pwm value
        movff   TCCOOLPWMS,TXFIFOIN9        ;cooling pwm state
        movff   TCCOOLPWMV,TXFIFOIN10       ;cooling pwm value
        clrf    TXFIFOIN11                  ;controller off
        btfsc   MODSTATE,MODPWMCTRL         ;is PWM controller on?
        setf    TXFIFOIN11                  ;yes
        call    TransmitThermometer
TCTransmitLoadTimer
        ;load periodical counter
        banksel E_TCCOUNTER
        movlw   .60                         ;period [s] = 60*E_TCCOUNTER
        mulwf   E_TCCOUNTER
        movff   PRODH,TCCOUNTERH
        movff   PRODL,TCCOUNTERL
    return

;==============================================================================
;                   OTHER THERMOMETER PROCEDURES
;==============================================================================
;------------------------------------------------------------------------------
; Routine:          THERMOMETER PROCEDURES
;------------------------------------------------------------------------------
; Overview:         It does all thermometer procedures
;------------------------------------------------------------------------------
ThermometerProcedures
        banksel E_ENABLEDPER
        btfss   E_ENABLEDPER,ENABLETEMP     ;is temperature module enabled?
        bra     ExitThermometerProcedures   ;no
    ;read DS sensor
        tstfsz  ERRORREG
        bra     ExitThermometerProcedures   ;1-wire error
        tstfsz  CNVRTFLAG,W                 ;DS sensor A/D conversion done?
        bra     ExitThermometerProcedures   ;not yet   
        rcall   ReadTemperature             ;read temperature from DS sensor
        tstfsz  ERRORREG
        bra     ExitThermometerProcedures   ;1-wire error
    ;do all with read temperature
        rcall   AddTempOffset               ;add temperature offset
        rcall   CompareTemp                 ;compare temperature to previous one and send msg if needed
        ;thermostat
        banksel E_ENABLEDPER
        btfsc   E_ENABLEDPER,ENABLETHERM    ;is thermostat module enabled?
        rcall   CompareTherm                ;compare thermostat settings to new temperature and send msg if needed
        ;controller
        banksel E_ENABLEDPER
        btfsc   E_ENABLEDPER,ENABLECTRL     ;is controller module enabled?
        rcall   CompareContr                ;compare controller settings to new temperature
        ;other
        rcall   SetThermometerNewState      ;sets new state if requested and sends message
        rcall   SendPeriodically            ;send temp, thermostat & controller messages periodically
ExitThermometerProcedures
        rcall   CnvrtTemperature            ;do temperature conversion in DS sensor
    return

;------------------------------------------------------------------------------
; Routine:          THERMOMETER POWER UP VALUES
;------------------------------------------------------------------------------
; Overview:         Sets registers at power up 
;------------------------------------------------------------------------------
ThermometerPowerUpValues
    ;TEMPERATURE
        ;set temperature offset
        movff   E_OFFSETMSB,OFFSETMSB       ;set temperature offset
        movff   E_OFFSETLSB,OFFSETLSB
        ;temerature setpoint
        banksel SETPOINTMSB
        movff   E_SETPOINTMSB,SETPOINTMSB   ;set switching temperature
        movff   E_SETPOINTLSB,SETPOINTLSB
        movlw   0x08                        ;if SETPOINT = 0x0800 then take last saved
        xorwf   SETPOINTMSB,W                  
        bnz     $ + .16
        movlw   0x00
        xorwf   SETPOINTLSB,W                  
        bnz     $ + .10
        movff   E_SETPOINTSAVEDMSB,SETPOINTMSB  ;last saved switching temperature
        movff   E_SETPOINTSAVEDLSB,SETPOINTLSB
        ;other
        call    TMTransmitLoadTimer         ;load timer for periodical message
        banksel TEMPLSBPRE
        movlw   0x08                        ;previous temperature 0x0800
        movwf   TEMPLSBPRE                    
        clrf    TEMPMSBPRE
        setf    CNVRTFLAG                   ;will force to do temp A/D conversion first
        banksel mT_DEACTIVTIMER             ;buttons must be active
        clrf    mT_DEACTIVTIMER
    ;THERMOSTAT MODULE
        ;power up states
        banksel E_MODSTATESOURCE
        movff   E_MODSTATESET,MODSTATENEW   ;take "set power up states"
        comf    E_MODSTATESOURCE,W          ;take bits that will be taken from "last saved" - these bits are zeros now in WREG
        andwf   MODSTATENEW,F               ;clear bits that will be taken from "last saved"
        movf    E_MODSTATESOURCE,W          ;take bits that will be taken from "last saved" - these bits are ones now in WREG
        andwf   E_MODSTATESAVED,W           ;remove unwanted bits
        iorwf   MODSTATENEW,F               ;take bits from last saved
        movff   MODSTATENEW,MODSTATE
        ;hysteresis
        banksel HYSTEMSB
        movff   E_HYSTERE,HYSTELSB          ;0-255 -> 0.0625deg-16.000deg        
        clrf    HYSTEMSB     
        infsnz  HYSTELSB                    ;zero means 0.0625deg, so increment hysteresis register
        incf    HYSTEMSB
        ;other values
        call    THTransmitLoadTimer         ;load timer for periodical message
        movlw   0x80                        ;thermostat status not set yet
        movwf   THERMOS
    ;TEMPERATURE CONTROLLER
        banksel TCHEATPWMV
        movff   E_TCSENS,TCSENS             ;temperature controller sensitivity 
        movff   E_TCPWMPERIOD,TCPWMPERIOD   ;temperature controller PWM period
        call    TCTransmitLoadTimer         ;load timer for periodical message
        movlw   0x80                        ;set PWM previous regs to force sending message at reboot
        movwf   TCHEATPWMSPRE
        clrf    TCHEATPWMV                  ;clear PWM regs
        clrf    TCCOOLPWMV
        clrf    TCHEATPWMS
        clrf    TCCOOLPWMS
        clrf    TCPWMPELSB
        clrf    TCPWMPEMSB
        clrf    TCPWMONLSB
        clrf    TCPWMONMSB
        clrf    TCERRLSBPRE                 ;saved temperature controller error bytes
        clrf    TCERRMSBPRE
        clrf    TCFLAGS                     ;state flags
    return

;------------------------------------------------------------------------------
; Routine:          SET THERMOMETER STATE
;------------------------------------------------------------------------------
; Overview:         It sets thermometer new state if was requested and sends
;                   message to the bus.
;------------------------------------------------------------------------------
SetThermometerNewState
        banksel E_ENABLEDPER
        btfsc   E_ENABLEDPER,ENABLETHERM    ;is thermostat module enabled?
        rcall   SetThermostat
        banksel E_ENABLEDPER
        btfsc   E_ENABLEDPER,ENABLECTRL     ;is controller module enabled?
        rcall   SetController
ExitSetThermometeNewState
    return

;----------------------------
SetThermostat
        movf    MODSTATENEW,W               ;new state
        xorwf   MODSTATE,W                  ;actual state
        btfss   WREG,MODTHERM               ;changed?
    return                                  ;no
        bcf     MODSTATE,MODTHERM           ;yes, so update it
        btfsc   MODSTATENEW,MODTHERM
        bsf     MODSTATE,MODTHERM
        call    THTransmit                  ;and send message
        call    EepromToSave                ;save new state
    return
SetController
        movf    MODSTATENEW,W               ;new state
        xorwf   MODSTATE,W                  ;actual state
        btfss   WREG,MODPWMCTRL             ;changed?
    return                                  ;no
        bcf     MODSTATE,MODPWMCTRL         ;yes, so update it
        btfsc   MODSTATENEW,MODPWMCTRL
        bsf     MODSTATE,MODPWMCTRL
        call    TCTransmit                  ;and send message
        call    EepromToSave
    return

;------------------------------------------------------------------------------
; Routine:          SEND PERIODICALLY
;------------------------------------------------------------------------------
; Overview:         It sends temperature, thermostat & controller messages
;                   periodically
;------------------------------------------------------------------------------
SendPeriodically
        rcall   DecTempCounter              ;decrement temperature periodical message counter
        banksel E_ENABLEDPER
        btfsc   E_ENABLEDPER,ENABLETHERM    ;is thermostat module enabled?
        rcall   DecThermCounter             ;decrement thermostat periodical message counter
        banksel E_ENABLEDPER
        btfsc   E_ENABLEDPER,ENABLECTRL     ;is controller module enabled?
        rcall   DecCtrlCounter              ;decrement controller periodical message counter
    return

;----------------------------
DecTempCounter
        movf    TMCOUNTERH,W                ;counter already zero?
        iorwf   TMCOUNTERL,W
        bnz     $ + .4
    return                                  ;yes, so exit
        clrf    WREG                        ;decrement timer
        decf    TMCOUNTERL
        subwfb  TMCOUNTERH
        movf    TMCOUNTERH,W                ;counter is zero?
        iorwf   TMCOUNTERL,W
        bz      $ + .4
    return                                  ;no, so exit
        tstfsz  ERRORREG                    ;error?
        bra     $ + .4                      ;yes
        bra     $ + .6
        rcall   ErrorTransmit               ;error, so exit
    return
        rcall   TMTransmit                  ;send temperature
    return
;--------------
DecThermCounter
        movf    THCOUNTERH,W
        iorwf   THCOUNTERL,W
        bnz     $ + .4
    return
        clrf    WREG
        decf    THCOUNTERL
        subwfb  THCOUNTERH
        movf    THCOUNTERH,W
        iorwf   THCOUNTERL,W
        bz      $ + .4
    return
        tstfsz  ERRORREG
        bra     $ + .4
        bra     $ + .6
        rcall   ErrorTransmit
    return
        rcall   THTransmit                  ;send thermostat
    return
;--------------
DecCtrlCounter
        movf    TCCOUNTERH,W
        iorwf   TCCOUNTERL,W
        bnz     $ + .4
    return
        clrf    WREG
        decf    TCCOUNTERL
        subwfb  TCCOUNTERH
        movf    TCCOUNTERH,W
        iorwf   TCCOUNTERL,W
        bz      $ + .4
    return
        tstfsz  ERRORREG
        bra     $ + .4
        bra     $ + .6
        rcall   ErrorTransmit
    return
        rcall   TCTransmit                  ;send controller
    return

;------------------------------------------------------------------------------
; Routine:          TRANSMIT THERMOMETER
;------------------------------------------------------------------------------
; Overview:         It saves message to transmit FIFO buffer
;------------------------------------------------------------------------------
TransmitThermometer
        banksel TXFIFOIN0
        movlw   0x30                        ;set frame type
        movwf   TXFIFOIN0
        movlw   0x40
        movwf   TXFIFOIN1
        movff   NODENR,TXFIFOIN2            ;node id
        movff   GROUPNR,TXFIFOIN3
        setf    TXFIFOIN4
        setf    TXFIFOIN5
        ;(TXFIFOIN6 -TXFIFOIN11) are already changed in parent procedure
        call    WriteToCanTxFIFO
    ;node can respond to its own message
        bcf     INTCON,GIEL                 ;disable low priority intr to make sure RXFIFO buffer is not overwritten
        call    Copy_TXFIFOIN_RXFIFOIN
        call    WriteToCanRxFIFO
        bsf     INTCON,GIEL                 ;enable back interrupt
    return

;------------------------------------------------------------------------------
; Routine:          TRANSMIT ERROR FRAME
;------------------------------------------------------------------------------
; Overview:         It transmits error message 0xF0
;------------------------------------------------------------------------------
ErrorTransmit
        ;error 0xF0 frame
        banksel TXFIFOIN0
        movlw   0xF0                        
        movwf   TXFIFOIN6
        movff   ERRORREG,TXFIFOIN7          ;load error flag
        setf    TXFIFOIN8                   ;unused
        setf    TXFIFOIN9                   ;unused
        setf    TXFIFOIN10                  ;unused
        setf    TXFIFOIN11                  ;unused
        call    TransmitThermometer
    return

;------------------------------------------------------------------------------
; Routine:          SAVE SETPOINT TO EEPROM
;------------------------------------------------------------------------------
; Overview:         It saves current setpoint value into EEPROM memory
;------------------------------------------------------------------------------
SaveStatesToEeprom           
        banksel EEPROMTIMER
    ;wait 6s before saving
        tstfsz  EEPROMTIMER
        decfsz  EEPROMTIMER
        bra     ExitSaveStateToEeprom
    ;save to eeprom
        banksel E_SETPOINTSAVEDMSB
        clrf    EEADRH                      ;point at high address
        ;setpoint MSB
        movf    E_SETPOINTSAVEDMSB,W        ;values the same?
        xorwf   SETPOINTMSB,W
        bz      $ + .16                     ;yes, so don't save  
        movff   SETPOINTMSB,E_SETPOINTSAVEDMSB        
        movlw   low E_SETPOINTSAVEDMSB      ;point at eeprom low address    
        movwf   EEADR
        movf    SETPOINTMSB,W               ;set data
        call    EepromSaveWREG
        ;setpoint LSB
        movf    E_SETPOINTSAVEDLSB,W        ;values the same?
        xorwf   SETPOINTLSB,W
        bz      $ + .16                     ;yes, so don't save  
        movff   SETPOINTLSB,E_SETPOINTSAVEDLSB   
        movlw   low E_SETPOINTSAVEDLSB      ;point at eeprom low address    
        movwf   EEADR                       ;point at next address
        movf    SETPOINTLSB,W               ;set data
        call    EepromSaveWREG
        ;save thermometer states
        movf    E_MODSTATESAVED,W           ;values the same?
        xorwf   MODSTATE,W
        bz      $ + .16                     ;yes, so don't save  
        movff   MODSTATE,E_MODSTATESAVED
        movlw   low E_MODSTATESAVED         ;point at eeprom low address  
        movwf   EEADR
        movf    MODSTATE,W                  ;set data
        call    EepromSaveWREG
ExitSaveStateToEeprom
    return

;==============================================================================
;===  END  OF  PROGRAM  =======================================================
;==============================================================================
    END