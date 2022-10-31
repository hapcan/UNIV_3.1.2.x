    LIST
;==============================================================================
;   MIT License
;
;   Copyright (c) 2022 HAPCAN Home Automation Project (http://hapcan.com)
;
;   Permission is hereby granted, free of charge, to any person obtaining a copy
;   of this software and associated documentation files (the "Software"), to deal
;   in the Software without restriction, including without limitation the rights
;   to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
;   copies of the Software, and to permit persons to whom the Software is
;   furnished to do so, subject to the following conditions:
;
;   The above copyright notice and this permission notice shall be included in all
;   copies or substantial portions of the Software.
;
;   THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
;   IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
;   FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
;   AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
;   LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
;   OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
;   SOFTWARE.
;==============================================================================
;   Filename:           univ3_mTouch-revX.inc
;   Associated diagram: none
;   Author:             Jacek Siwilo   
;   Description         UNIV 3 CPU mTouch routines optimized for 8MHz clock
;==============================================================================                   
;   Revision History
;   Rev:  Date:     Details:
;   0     02.2014   Initial version
;   1     08.2017   ReEnAllInt instead of EnAllInt procedure
;==============================================================================

;------------------------------------------------------------------------------
; Local Definitions
;------------------------------------------------------------------------------
;These registers must be located in any ONE BANK
;mT_SAMPCHAN2H      - AN2 average sample 
;mT_SAMPCHAN2L
;mT_SAMPCHAN3H      - AN3 average sample 
;mT_SAMPCHAN3L  
;mT_SAMPCHAN4H      - AN4 average sample  
;mT_SAMPCHAN4L  
;mT_SAMPCHAN8H      - AN8 average sample 
;mT_SAMPCHAN8L    
;mT_SAMPCHAN9H      - AN9 average sample 
;mT_SAMPCHAN9L    
;mT_SAMPCHAN10H     - AN10 average sample 
;mT_SAMPCHAN10L   
;mT_BASECHAN2H      - AN2 base line value
;mT_BASECHAN2L    
;mT_BASECHAN3H      - AN3 base line value
;mT_BASECHAN3L      
;mT_BASECHAN4H      - AN4 base line value
;mT_BASECHAN4L      
;mT_BASECHAN8H      - AN8 base line value
;mT_BASECHAN8L      
;mT_BASECHAN9H      - AN9 base line value
;mT_BASECHAN9L      
;mT_BASECHAN10H     - AN10 base line value
;mT_BASECHAN10L     
;mT_TRIMAN2         - AN2 current source trim bits
;mT_TRIMAN3         - AN3 current source trim bits
;mT_TRIMAN4         - AN4 current source trim bits
;mT_TRIMAN8         - AN8 current source trim bits
;mT_TRIMAN9         - AN9 current source trim bits
;mT_TRIMAN10        - AN10 current source trim bits
;mT_WORKSAMPH       - working sample value
;mT_WORKSAMPL      
;mT_WORKBASEH       - working baseline
;mT_WORKBASEL      
;mT_WORKONTHRESH    - working threshold ON value
;mT_WORKONTHRESL   
;mT_WORKOFFTHRESH   - working threshold OFF value
;mT_WORKOFFTHRESL  
;mT_WORKTHRESH      - working threshold value
;mT_WORKTHRESL     
;mT_BUTTONSTATUS    - decoded buttons status
;mT_CNTR1           - working counter
;mT_SAMPCNTR        - A/D sample counter
;mT_SAMPH           - A/D sample high byte
;mT_SAMPL           - A/D sample low byte
;mT_TRIMBITS        - working trim bits reg

;These registers can be located anywere - they contain configuration
;mT_ONTHRESAN2H     - AN2 press threshold
;mT_ONTHRESAN2L     
;mT_OFFTHRESAN2H    - AN2 release threshold
;mT_OFFTHRESAN2L    
;mT_ONTHRESAN3H     - AN3 press threshold
;mT_ONTHRESAN3L     
;mT_OFFTHRESAN3H    - AN3 release threshold
;mT_OFFTHRESAN3L   
;mT_ONTHRESAN4H     - AN4 press threshold
;mT_ONTHRESAN4L    
;mT_OFFTHRESAN4H    - AN4 release threshold
;mT_OFFTHRESAN4L   
;mT_ONTHRESAN8H     - AN8 press threshold
;mT_ONTHRESAN8L    
;mT_OFFTHRESAN8H    - AN8 release threshold
;mT_OFFTHRESAN8L   
;mT_ONTHRESAN9H     - AN9 press threshold
;mT_ONTHRESAN9L    
;mT_OFFTHRESAN9H    - AN9 release threshold
;mT_OFFTHRESAN9L   
;mT_ONTHRESAN10H    - AN10 press threshold
;mT_ONTHRESAN10L   
;mT_OFFTHRESAN10H   - AN10 release threshold
;mT_OFFTHRESAN10L  

;==============================================================================
;==============================================================================
;       m T O U C H   I N I T I A L I Z A T I O N
;==============================================================================
;==============================================================================

;------------------------------------------------------------------------------
; Routine:          mTOUCH INITIALIZATION
;------------------------------------------------------------------------------
; Overview:         Inits CTMU & A/D, calibrates current and sets average value
;                   for each channel
;    Input:         none
;   Output:         none
;------------------------------------------------------------------------------
mTouch_Initialization   
        banksel  mT_BUTTONSTATUS
        clrf     mT_BUTTONSTATUS            ;buttons released
        ;A/D
        banksel PMD1
        bcf     PMD1,ADCMD                  ;enable power to A/D module
        movlw   b'10011001'                 ;result right justified, A/D clock Tad= Tosc*8= 1us, acquisition time Tacq= 6Tad
        movwf   ADCON2
        movlw   b'00000000'                 ;Vref-=Vss, Vref+=Vdd, analog "-" =Vss
        movwf   ADCON1
        movlw   b'00000001'                 ;power up converter
        movwf   ADCON0
        ;CTMU
        banksel PMD1
        bcf     PMD1,CTMUMD                 ;enable power to CTMU module
        movlw   b'10000000'                 ;enable CTMU,run in idle mode,disable edge delay,edges blocked,no edge sequence,analog output not grounded,disable trigger
        movwf   CTMUCONH
        movlw   b'10010000'                 ;edge will be set manualy, so clear all edges settings
        movwf   CTMUCONL
        movlw   b'00000011'                 ;clear trim bits, current 55uA
        movwf   CTMUICON
        ;current source calibration
        rcall   mTouch_CurrentCalibration   ;set current for each channel
        ;set base line value for each channel
        movff   mT_SAMPCHAN2H,mT_BASECHAN2H
        movff   mT_SAMPCHAN2L,mT_BASECHAN2L
        movff   mT_SAMPCHAN3H,mT_BASECHAN3H
        movff   mT_SAMPCHAN3L,mT_BASECHAN3L
        movff   mT_SAMPCHAN4H,mT_BASECHAN4H
        movff   mT_SAMPCHAN4L,mT_BASECHAN4L
        movff   mT_SAMPCHAN8H,mT_BASECHAN8H
        movff   mT_SAMPCHAN8L,mT_BASECHAN8L
        movff   mT_SAMPCHAN9H,mT_BASECHAN9H
        movff   mT_SAMPCHAN9L,mT_BASECHAN9L
        movff   mT_SAMPCHAN10H,mT_BASECHAN10H
        movff   mT_SAMPCHAN10L,mT_BASECHAN10L
    return

;------------------------------------------------------------------------------
; Routine:          mTOUCH CURRENT CALIBRATION
;------------------------------------------------------------------------------
; Overview:         Sets trim bits for each channel to get 80% of max voltage
;    Input:         none
;   Output:         mT_TRIMANx registers with calibrated trim bits
;                   or mT_TRIMANx = 0 if calibration fails
;------------------------------------------------------------------------------
mTouch_CurrentCalibration
        rcall   mTouch_CurrentCalibrChAN2
        rcall   mTouch_CurrentCalibrChAN3
        rcall   mTouch_CurrentCalibrChAN4
        rcall   mTouch_CurrentCalibrChAN8
        rcall   mTouch_CurrentCalibrChAN9
        rcall   mTouch_CurrentCalibrChAN10
    return

;-- Channel AN2 ----------------
mTouch_CurrentCalibrChAN2
        rcall   mTouch_CurrentCalibrStart   ;prepare trim bits and clear loop counter
mTouch_CurrentCalibrChAN2Loop
        rcall   mTouch_CurrentCalibrIncTrimBits ;take trim bits next value
        call    mTouch_GetAvrgSampleChAN2   ;get average sample
        banksel mT_SAMPCHAN2H
        movlw   .12                         ;average sample is lesser than 3072 (80% of 4095)?
        cpfslt  mT_SAMPCHAN2H
        bra     $ + .10                     ;no, so exit loop
        decfsz  mT_CNTR1
        bra     mTouch_CurrentCalibrChAN2Loop
        clrf    mT_TRIMAN2                  ;all trim bits looped with no success, so clear trimbits register
    retlw   0x01                            ;return with error
        movff   CTMUICON,mT_TRIMAN2         ;get trim bits for this channel
    retlw   0x00 
;-- Channel AN3 ----------------
mTouch_CurrentCalibrChAN3
        rcall   mTouch_CurrentCalibrStart
mTouch_CurrentCalibrChAN3Loop
        rcall   mTouch_CurrentCalibrIncTrimBits
        call    mTouch_GetAvrgSampleChAN3
        banksel mT_SAMPCHAN3H
        movlw   .12
        cpfslt  mT_SAMPCHAN3H
        bra     $ + .10
        decfsz  mT_CNTR1
        bra     mTouch_CurrentCalibrChAN3Loop
        clrf    mT_TRIMAN3
    retlw   0x01
        movff   CTMUICON,mT_TRIMAN3
    retlw   0x00 
;-- Channel AN4 ----------------
mTouch_CurrentCalibrChAN4
        rcall   mTouch_CurrentCalibrStart
mTouch_CurrentCalibrChAN4Loop
        rcall   mTouch_CurrentCalibrIncTrimBits
        call    mTouch_GetAvrgSampleChAN4
        banksel mT_SAMPCHAN4H
        movlw   .12
        cpfslt  mT_SAMPCHAN4H
        bra     $ + .10
        decfsz  mT_CNTR1
        bra     mTouch_CurrentCalibrChAN4Loop
        clrf    mT_TRIMAN4
    retlw   0x01
        movff   CTMUICON,mT_TRIMAN4
    retlw   0x00 
;-- Channel AN8 ----------------
mTouch_CurrentCalibrChAN8
        rcall   mTouch_CurrentCalibrStart
mTouch_CurrentCalibrChAN8Loop
        rcall   mTouch_CurrentCalibrIncTrimBits
        call    mTouch_GetAvrgSampleChAN8
        banksel mT_SAMPCHAN8H
        movlw   .12
        cpfslt  mT_SAMPCHAN8H
        bra     $ + .10
        decfsz  mT_CNTR1
        bra     mTouch_CurrentCalibrChAN8Loop
        clrf    mT_TRIMAN8
    retlw   0x01
        movff   CTMUICON,mT_TRIMAN8
    retlw   0x00 
;-- Channel AN9 ----------------
mTouch_CurrentCalibrChAN9
        rcall   mTouch_CurrentCalibrStart
mTouch_CurrentCalibrChAN9Loop
        rcall   mTouch_CurrentCalibrIncTrimBits
        call    mTouch_GetAvrgSampleChAN9
        banksel mT_SAMPCHAN9H
        movlw   .12
        cpfslt  mT_SAMPCHAN9H
        bra     $ + .10
        decfsz  mT_CNTR1
        bra     mTouch_CurrentCalibrChAN9Loop
        clrf    mT_TRIMAN9
    retlw   0x01
        movff   CTMUICON,mT_TRIMAN9
    retlw   0x00 
;-- Channel AN10 ---------------
mTouch_CurrentCalibrChAN10
        rcall   mTouch_CurrentCalibrStart
mTouch_CurrentCalibrChAN10Loop
        rcall   mTouch_CurrentCalibrIncTrimBits
        call    mTouch_GetAvrgSampleChAN10
        banksel mT_SAMPCHAN10H
        movlw   .12
        cpfslt  mT_SAMPCHAN10H
        bra     $ + .10
        decfsz  mT_CNTR1
        bra     mTouch_CurrentCalibrChAN10Loop
        clrf    mT_TRIMAN10
    retlw   0x01 
        movff   CTMUICON,mT_TRIMAN10
    retlw   0x00

;-------------------------------
mTouch_CurrentCalibrStart
        banksel mT_TRIMBITS
        movlw   b'11100000'                 ;start trim bits from .-31 (-32 here because incf in few steps)                 
        movwf   mT_TRIMBITS
        movlw   .63                         ;check 63 values up to -32+63 = 31
        movwf   mT_CNTR1
    return
;-------------------------------
mTouch_CurrentCalibrIncTrimBits
        banksel mT_TRIMBITS
        incf    mT_TRIMBITS                 ;increment value
        bsf     STATUS,C                    ;rotate left with two 1s (IRNG bits = 55uA)
        rlcf    mT_TRIMBITS,W
        bsf     STATUS,C
        rlcf    WREG,W
        banksel CTMUICON
        movwf   CTMUICON 
    return

;==============================================================================
;==============================================================================
;       m T O U C H   R O U T I N E S
;==============================================================================
;==============================================================================

;------------------------------------------------------------------------------
; Routine:          mTOUCH GET AVERAGE SAMPLE
;------------------------------------------------------------------------------
; Overview:         Takes a few A/D samples and calculates average value for
;                   each channel.
;    Input:         none
;   Output:         SAMPCHANx registers with average value
;------------------------------------------------------------------------------
;-- Channel AN2 ----------------
mTouch_GetAvrgSampleChAN2
        rcall   mTouch_GetAvrgSampleStart   ;prepare sample counter and clear sample regs
        rcall   mTouch_GetRawSampleChAN2    ;read sample
        rcall   mTouch_GetAvrgSampleAddSamp ;add sample
        banksel mT_SAMPCNTR
        decfsz  mT_SAMPCNTR                 ;all samples taken?
        bra     $ - .8                      ;not yet
        bcf     LATA,LATA2                  ;normaly keep sensor as low output
        bcf     TRISA,TRISA2
        rcall   mTouch_GetAvrgSampleCalcAvrg ;calculate average value
        movff   mT_SAMPH,mT_SAMPCHAN2H      ;save it
        movff   mT_SAMPL,mT_SAMPCHAN2L
    return
;-- Channel AN3 ----------------
mTouch_GetAvrgSampleChAN3
        rcall   mTouch_GetAvrgSampleStart
        rcall   mTouch_GetRawSampleChAN3
        rcall   mTouch_GetAvrgSampleAddSamp
        banksel mT_SAMPCNTR
        decfsz  mT_SAMPCNTR
        bra     $ - .8
        bcf     LATA,LATA3
        bcf     TRISA,TRISA3
        rcall   mTouch_GetAvrgSampleCalcAvrg
        movff   mT_SAMPH,mT_SAMPCHAN3H
        movff   mT_SAMPL,mT_SAMPCHAN3L
    return
;-- Channel AN4 ----------------
mTouch_GetAvrgSampleChAN4
        rcall   mTouch_GetAvrgSampleStart
        rcall   mTouch_GetRawSampleChAN4
        rcall   mTouch_GetAvrgSampleAddSamp
        banksel mT_SAMPCNTR
        decfsz  mT_SAMPCNTR
        bra     $ - .8
        bcf     LATA,LATA5
        bcf     TRISA,TRISA5
        rcall   mTouch_GetAvrgSampleCalcAvrg
        movff   mT_SAMPH,mT_SAMPCHAN4H
        movff   mT_SAMPL,mT_SAMPCHAN4L
    return
;-- Channel AN8 ----------------
mTouch_GetAvrgSampleChAN8
        rcall   mTouch_GetAvrgSampleStart
        rcall   mTouch_GetRawSampleChAN8
        rcall   mTouch_GetAvrgSampleAddSamp
        banksel mT_SAMPCNTR
        decfsz  mT_SAMPCNTR
        bra     $ - .8
        bcf     LATB,LATB1
        bcf     TRISB,TRISB1
        rcall   mTouch_GetAvrgSampleCalcAvrg
        movff   mT_SAMPH,mT_SAMPCHAN8H
        movff   mT_SAMPL,mT_SAMPCHAN8L
    return
;-- Channel AN9 ----------------
mTouch_GetAvrgSampleChAN9
        rcall   mTouch_GetAvrgSampleStart
        rcall   mTouch_GetRawSampleChAN9
        rcall   mTouch_GetAvrgSampleAddSamp
        banksel mT_SAMPCNTR
        decfsz  mT_SAMPCNTR
        bra     $ - .8
        bcf     LATB,LATB4
        bcf     TRISB,TRISB4
        rcall   mTouch_GetAvrgSampleCalcAvrg
        movff   mT_SAMPH,mT_SAMPCHAN9H
        movff   mT_SAMPL,mT_SAMPCHAN9L
    return
;-- Channel AN10 ---------------
mTouch_GetAvrgSampleChAN10
        rcall   mTouch_GetAvrgSampleStart
        rcall   mTouch_GetRawSampleChAN10
        rcall   mTouch_GetAvrgSampleAddSamp
        banksel mT_SAMPCNTR
        decfsz  mT_SAMPCNTR
        bra     $ - .8
        bcf     LATB,LATB0
        bcf     TRISB,TRISB0
        rcall   mTouch_GetAvrgSampleCalcAvrg
        movff   mT_SAMPH,mT_SAMPCHAN10H
        movff   mT_SAMPL,mT_SAMPCHAN10L
    return

;-------------------------------
mTouch_GetAvrgSampleStart                   ;clears counter and total sample reg
        banksel mT_SAMPCNTR
        movlw   .16                         ;get 16 samples
        movwf   mT_SAMPCNTR
        clrf    mT_SAMPH                    ;clear sample registers
        clrf    mT_SAMPL
    return
;-------------------------------
mTouch_GetAvrgSampleAddSamp                 ;adds sample to total sample reg
        banksel mT_SAMPL
        movf    ADRESL,W         
        addwf   mT_SAMPL
        movf    ADRESH,W
        addwfc  mT_SAMPH
    return
;-------------------------------
mTouch_GetAvrgSampleCalcAvrg                ;calculates average value = mT_SAMP/16
        banksel mT_SAMPL
        bcf     STATUS,C
        rrcf    mT_SAMPH
        rrcf    mT_SAMPL
        bcf     STATUS,C
        rrcf    mT_SAMPH
        rrcf    mT_SAMPL
        bcf     STATUS,C
        rrcf    mT_SAMPH
        rrcf    mT_SAMPL
        bcf     STATUS,C
        rrcf    mT_SAMPH
        rrcf    mT_SAMPL
    return

;------------------------------------------------------------------------------
; Routine:          mTOUCH GET SAMPLE
;------------------------------------------------------------------------------
; Overview:         Takes one A/D sample for particular channel.
;    Input:         none
;   Output:         Sample in ADRESH,ADERSK regs.
;------------------------------------------------------------------------------
;-- Channel AN2 ----------------
mTouch_GetRawSampleChAN2
        movlw   b'00001001'                 ;switch A/D MUX to AN2, power up converter
        movwf   ADCON0
        bcf     LATA,LATA2                  ;drain channel
        bcf     TRISA,TRISA2
        bsf     TRISA,TRISA2                ;set channel as input (Hi-Z)
        rcall   mTouch_ReadRawSample        ;read input don't enable interrupts
    return
;-- Channel AN3 ----------------
mTouch_GetRawSampleChAN3
        movlw   b'00001101'                 ;switch to AN3, power up converter
        movwf   ADCON0
        bcf     LATA,LATA3
        bcf     TRISA,TRISA3
        bsf     TRISA,TRISA3
        rcall   mTouch_ReadRawSample
    return
;-- Channel AN4 ----------------
mTouch_GetRawSampleChAN4
        movlw   b'00010001'                 ;switch to AN4, power up converter
        movwf   ADCON0
        bcf     LATA,LATA5
        bcf     TRISA,TRISA5
        bsf     TRISA,TRISA5
        rcall   mTouch_ReadRawSample
    return
;-- Channel AN8 ----------------
mTouch_GetRawSampleChAN8
        movlw   b'00100001'                 ;switch to AN8, power up converter
        movwf   ADCON0
        bcf     LATB,LATB1
        bcf     TRISB,TRISB1
        bsf     TRISB,TRISB1
        rcall   mTouch_ReadRawSample
    return
;-- Channel AN9 ----------------
mTouch_GetRawSampleChAN9
        movlw   b'00100101'                 ;switch to AN9, power up converter
        movwf   ADCON0
        bcf     LATB,LATB4
        bcf     TRISB,TRISB4
        bsf     TRISB,TRISB4
        rcall   mTouch_ReadRawSample
    return
;-- Channel AN10 ---------------
mTouch_GetRawSampleChAN10
        movlw   b'00101001'                 ;switch to AN10, power up converter
        movwf   ADCON0
        bcf     LATB,LATB0
        bcf     TRISB,TRISB0
        bsf     TRISB,TRISB0
        rcall   mTouch_ReadRawSample
    return

;------------------------------------------------------------------------------
; Routine:          mTOUCH READ CHANNEL
;------------------------------------------------------------------------------
; Overview:         Reads channel
;    Input:         Set particular channel as analog input
;   Output:         none
;------------------------------------------------------------------------------
mTouch_ReadRawSample
        btfsc   INTCON,GIEH                 ;test if interrupt is enabled
        bra     $ + 6                       ;yes, so disable and then read   
        ;don't change interrupts
        rcall   mTouch_ReadRawSampleNow
    return
        ;disable interrupts first
        call    DisAllInt                   ;disable interrupt
        rcall   mTouch_ReadRawSampleNow
        call    ReEnAllInt                  ;reenable interrupt
    return
;-------------------------------
mTouch_ReadRawSampleNow                    
        banksel CTMUCONL
        bsf     ADCON0,GO                   ;start sampling, conversion will start in 6Tad=6.0us
        bsf     CTMUCONL,EDG1STAT           ;start injecting current                       0.5us
        nop                                 ;0.5us @ 8MHz                                  1.0us
        nop                                 ;0.5us @ 8MHz                                  1.5us
        nop                                 ;0.5us @ 8MHz                                  2.0us
        nop                                 ;0.5us @ 8MHz                                  2.5us
        nop                                 ;0.5us @ 8MHz                                  3.0us
        nop                                 ;0.5us @ 8MHz                                  3.5us
        nop                                 ;0.5us @ 8MHz                                  4.0us
        bcf     CTMUCONL,EDG1STAT           ;stop injecting current                        4.5us        
        btfsc   ADCON0,GO                   ;wait till conversion finished                 5.0us
        bra     $ - 2
    return


;------------------------------------------------------------------------------
; Routine:          mTOUCH BUTTON STATUS
;------------------------------------------------------------------------------
; Overview:         Routine decodes button status for each channel
;    Input:         SAMPLE, BASELINE, THRESHOLD ON, OFF registers for each channel
;   Output:         BUTTONSTATUS register
;------------------------------------------------------------------------------
mTouch_DecodeButtonStatus
        rcall   mTouch_DecodeButtonStatusAN2
        rcall   mTouch_DecodeButtonStatusAN3
        rcall   mTouch_DecodeButtonStatusAN4
        rcall   mTouch_DecodeButtonStatusAN8
        rcall   mTouch_DecodeButtonStatusAN9
        rcall   mTouch_DecodeButtonStatusAN10
    return

;-- Channel AN2 ----------------
mTouch_DecodeButtonStatusAN2
        banksel mT_BUTTONSTATUS
        movff   mT_SAMPCHAN2H,mT_WORKSAMPH          ;take sample value
        movff   mT_SAMPCHAN2L,mT_WORKSAMPL
        movff   mT_BASECHAN2H,mT_WORKBASEH          ;take baseline
        movff   mT_BASECHAN2L,mT_WORKBASEL
        movff   mT_ONTHRESAN2H,mT_WORKONTHRESH      ;take threshold ON value
        movff   mT_ONTHRESAN2L,mT_WORKONTHRESL
        movff   mT_OFFTHRESAN2H,mT_WORKOFFTHRESH    ;take threshold OFF value
        movff   mT_OFFTHRESAN2L,mT_WORKOFFTHRESL
        rcall   mTouch_DecodeButtonON               ;check if button is pressed
        tstfsz  WREG                                ;clear?
        bsf     mT_BUTTONSTATUS,2                   ;no, so button is pressed
        rcall   mTouch_DecodeButtonOFF              ;check if button is released
        tstfsz  WREG                                ;clear?
        bcf     mT_BUTTONSTATUS,2                   ;no, so button is released
    return
;-- Channel AN3 ----------------
mTouch_DecodeButtonStatusAN3
        banksel mT_BUTTONSTATUS
        movff   mT_SAMPCHAN3H,mT_WORKSAMPH          ;take sample value
        movff   mT_SAMPCHAN3L,mT_WORKSAMPL
        movff   mT_BASECHAN3H,mT_WORKBASEH          ;take baseline
        movff   mT_BASECHAN3L,mT_WORKBASEL
        movff   mT_ONTHRESAN3H,mT_WORKONTHRESH      ;take threshold ON value
        movff   mT_ONTHRESAN3L,mT_WORKONTHRESL
        movff   mT_OFFTHRESAN3H,mT_WORKOFFTHRESH    ;take threshold OFF value
        movff   mT_OFFTHRESAN3L,mT_WORKOFFTHRESL
        rcall   mTouch_DecodeButtonON               ;check if button is pressed
        tstfsz  WREG                                ;clear?
        bsf     mT_BUTTONSTATUS,3                   ;no, so button is pressed
        rcall   mTouch_DecodeButtonOFF              ;check if button is released
        tstfsz  WREG                                ;clear?
        bcf     mT_BUTTONSTATUS,3                   ;no, so button is released
    return
;-- Channel AN4 ----------------
mTouch_DecodeButtonStatusAN4
        banksel mT_BUTTONSTATUS
        movff   mT_SAMPCHAN4H,mT_WORKSAMPH          ;take sample value
        movff   mT_SAMPCHAN4L,mT_WORKSAMPL
        movff   mT_BASECHAN4H,mT_WORKBASEH          ;take baseline
        movff   mT_BASECHAN4L,mT_WORKBASEL
        movff   mT_ONTHRESAN4H,mT_WORKONTHRESH      ;take threshold ON value
        movff   mT_ONTHRESAN4L,mT_WORKONTHRESL
        movff   mT_OFFTHRESAN4H,mT_WORKOFFTHRESH    ;take threshold OFF value
        movff   mT_OFFTHRESAN4L,mT_WORKOFFTHRESL
        rcall   mTouch_DecodeButtonON               ;check if button is pressed
        tstfsz  WREG                                ;clear?
        bsf     mT_BUTTONSTATUS,4                   ;no, so button is pressed
        rcall   mTouch_DecodeButtonOFF              ;check if button is released
        tstfsz  WREG                                ;clear?
        bcf     mT_BUTTONSTATUS,4                   ;no, so button is released
    return
;-- Channel AN8 ----------------
mTouch_DecodeButtonStatusAN8
        banksel mT_BUTTONSTATUS
        movff   mT_SAMPCHAN8H,mT_WORKSAMPH          ;take sample value
        movff   mT_SAMPCHAN8L,mT_WORKSAMPL
        movff   mT_BASECHAN8H,mT_WORKBASEH          ;take baseline
        movff   mT_BASECHAN8L,mT_WORKBASEL
        movff   mT_ONTHRESAN8H,mT_WORKONTHRESH      ;take threshold ON value
        movff   mT_ONTHRESAN8L,mT_WORKONTHRESL
        movff   mT_OFFTHRESAN8H,mT_WORKOFFTHRESH    ;take threshold OFF value
        movff   mT_OFFTHRESAN8L,mT_WORKOFFTHRESL
        rcall   mTouch_DecodeButtonON               ;check if button is pressed
        tstfsz  WREG                                ;clear?
        bsf     mT_BUTTONSTATUS,5                   ;no, so button is pressed
        rcall   mTouch_DecodeButtonOFF              ;check if button is released
        tstfsz  WREG                                ;clear?
        bcf     mT_BUTTONSTATUS,5                   ;no, so button is released
    return
;-- Channel AN9 ----------------
mTouch_DecodeButtonStatusAN9
        banksel mT_BUTTONSTATUS
        movff   mT_SAMPCHAN9H,mT_WORKSAMPH          ;take sample value
        movff   mT_SAMPCHAN9L,mT_WORKSAMPL
        movff   mT_BASECHAN9H,mT_WORKBASEH          ;take baseline
        movff   mT_BASECHAN9L,mT_WORKBASEL
        movff   mT_ONTHRESAN9H,mT_WORKONTHRESH      ;take threshold ON value
        movff   mT_ONTHRESAN9L,mT_WORKONTHRESL
        movff   mT_OFFTHRESAN9H,mT_WORKOFFTHRESH    ;take threshold OFF value
        movff   mT_OFFTHRESAN9L,mT_WORKOFFTHRESL
        rcall   mTouch_DecodeButtonON               ;check if button is pressed
        tstfsz  WREG                                ;clear?
        bsf     mT_BUTTONSTATUS,6                   ;no, so button is pressed
        rcall   mTouch_DecodeButtonOFF              ;check if button is released
        tstfsz  WREG                                ;clear?
        bcf     mT_BUTTONSTATUS,6                   ;no, so button is released
    return
;-- Channel AN10 ---------------
mTouch_DecodeButtonStatusAN10
        banksel mT_BUTTONSTATUS
        movff   mT_SAMPCHAN10H,mT_WORKSAMPH          ;take sample value
        movff   mT_SAMPCHAN10L,mT_WORKSAMPL
        movff   mT_BASECHAN10H,mT_WORKBASEH          ;take baseline
        movff   mT_BASECHAN10L,mT_WORKBASEL
        movff   mT_ONTHRESAN10H,mT_WORKONTHRESH      ;take threshold ON value
        movff   mT_ONTHRESAN10L,mT_WORKONTHRESL
        movff   mT_OFFTHRESAN10H,mT_WORKOFFTHRESH    ;take threshold OFF value
        movff   mT_OFFTHRESAN10L,mT_WORKOFFTHRESL
        rcall   mTouch_DecodeButtonON               ;check if button is pressed
        tstfsz  WREG                                ;clear?
        bsf     mT_BUTTONSTATUS,7                   ;no, so button is pressed
        rcall   mTouch_DecodeButtonOFF              ;check if button is released
        tstfsz  WREG                                ;clear?
        bcf     mT_BUTTONSTATUS,7                   ;no, so button is released
    return

;-------------------------------
;if readbutton<baseline-ONthreshold then button is pressed (returns 1)
mTouch_DecodeButtonON
        banksel mT_WORKONTHRESL
        movf    mT_WORKONTHRESL,W           ;BASELINE-ONTHRESHOLD
        subwf   mT_WORKBASEL,W
        movwf   mT_WORKTHRESL
        movf    mT_WORKONTHRESH,W
        subwfb  mT_WORKBASEH,W
        movwf   mT_WORKTHRESH
        movf    mT_WORKTHRESL,W             ;SAMPCHAN-(BASELINE-ONTHRESHOLD)
        subwf   mT_WORKSAMPL,W
        movf    mT_WORKTHRESH,W
        subwfb  mT_WORKSAMPH,W
        btfss   WREG,7                      ;result negative?
    retlw   0x00                            ;no
    retlw   0x01                            ;yes, so button PRESSED
;if readbutton>baseline-OFFthreshold then button in released (returns 1)
mTouch_DecodeButtonOFF
        banksel mT_WORKOFFTHRESL
        movf    mT_WORKOFFTHRESL,W          ;BASELINE-OFFTHRESHOLD
        subwf   mT_WORKBASEL,W
        movwf   mT_WORKTHRESL
        movf    mT_WORKOFFTHRESH,W
        subwfb  mT_WORKBASEH,W
        movwf   mT_WORKTHRESH
        movf    mT_WORKSAMPL,W              ;(BASELINE-OFFTHRESHOLD)-SAMPCHAN
        subwf   mT_WORKTHRESL,W
        movf    mT_WORKSAMPH,W
        subwfb  mT_WORKTHRESH,W
        btfss   WREG,7                      ;result negative?
    retlw   0x00                            ;no
    retlw   0x01                            ;yes, so button RELEASED

;------------------------------------------------------------------------------
; Routine:          mTOUCH UPDATE BASELINE
;------------------------------------------------------------------------------
; Overview:         It updates baseline value for each channel using slew rate
;                   limiter filter
;    Input:         Button status, SAMPLE, BASELINE registers
;   Output:         Updated BASELINE
;------------------------------------------------------------------------------
mTouch_UpdateBaseline
        banksel mT_DEACTIVTIMER
        tstfsz  mT_DEACTIVTIMER
    return                                  ;exit because buttons are deactivated
        rcall   mTouch_UpdateBaselineAN2
        rcall   mTouch_UpdateBaselineAN3
        rcall   mTouch_UpdateBaselineAN4
        rcall   mTouch_UpdateBaselineAN8
        rcall   mTouch_UpdateBaselineAN9
        rcall   mTouch_UpdateBaselineAN10
    return

;-- Channel AN2 ----------------
mTouch_UpdateBaselineAN2
        banksel mT_BUTTONSTATUS
        btfsc   mT_BUTTONSTATUS,2           ;button pressed
    return                                  ;yes, so don't update baseline for this channel
        movff   mT_SAMPCHAN2H,mT_WORKSAMPH  ;take sample value
        movff   mT_SAMPCHAN2L,mT_WORKSAMPL
        movff   mT_BASECHAN2H,mT_WORKBASEH  ;take baseline
        movff   mT_BASECHAN2L,mT_WORKBASEL
        rcall   mTouch_CalculateBaseline
        movff   mT_WORKBASEH,mT_BASECHAN2H  ;move back updated baseline
        movff   mT_WORKBASEL,mT_BASECHAN2L
    return
;-- Channel AN3 ----------------
mTouch_UpdateBaselineAN3
        banksel mT_BUTTONSTATUS
        btfsc   mT_BUTTONSTATUS,3           ;button pressed
    return                                  ;yes, so don't update baseline for this channel
        movff   mT_SAMPCHAN3H,mT_WORKSAMPH  ;take sample value
        movff   mT_SAMPCHAN3L,mT_WORKSAMPL
        movff   mT_BASECHAN3H,mT_WORKBASEH  ;take baseline
        movff   mT_BASECHAN3L,mT_WORKBASEL
        rcall   mTouch_CalculateBaseline
        movff   mT_WORKBASEH,mT_BASECHAN3H  ;move back updated baseline
        movff   mT_WORKBASEL,mT_BASECHAN3L
    return
;-- Channel AN4 ----------------
mTouch_UpdateBaselineAN4
        banksel mT_BUTTONSTATUS
        btfsc   mT_BUTTONSTATUS,4           ;button pressed
    return                                  ;yes, so don't update baseline for this channel
        movff   mT_SAMPCHAN4H,mT_WORKSAMPH  ;take sample value
        movff   mT_SAMPCHAN4L,mT_WORKSAMPL
        movff   mT_BASECHAN4H,mT_WORKBASEH  ;take baseline
        movff   mT_BASECHAN4L,mT_WORKBASEL
        rcall   mTouch_CalculateBaseline
        movff   mT_WORKBASEH,mT_BASECHAN4H  ;move back updated baseline
        movff   mT_WORKBASEL,mT_BASECHAN4L
    return
;-- Channel AN8 ----------------
mTouch_UpdateBaselineAN8
        banksel mT_BUTTONSTATUS
        btfsc   mT_BUTTONSTATUS,5           ;button pressed
    return                                  ;yes, so don't update baseline for this channel
        movff   mT_SAMPCHAN8H,mT_WORKSAMPH  ;take sample value
        movff   mT_SAMPCHAN8L,mT_WORKSAMPL
        movff   mT_BASECHAN8H,mT_WORKBASEH  ;take baseline
        movff   mT_BASECHAN8L,mT_WORKBASEL
        rcall   mTouch_CalculateBaseline
        movff   mT_WORKBASEH,mT_BASECHAN8H  ;move back updated baseline
        movff   mT_WORKBASEL,mT_BASECHAN8L
    return
;-- Channel AN9 ----------------
mTouch_UpdateBaselineAN9
        banksel mT_BUTTONSTATUS
        btfsc   mT_BUTTONSTATUS,6           ;button pressed
    return                                  ;yes, so don't update baseline for this channel
        movff   mT_SAMPCHAN9H,mT_WORKSAMPH  ;take sample value
        movff   mT_SAMPCHAN9L,mT_WORKSAMPL
        movff   mT_BASECHAN9H,mT_WORKBASEH  ;take baseline
        movff   mT_BASECHAN9L,mT_WORKBASEL
        rcall   mTouch_CalculateBaseline
        movff   mT_WORKBASEH,mT_BASECHAN9H  ;move back updated baseline
        movff   mT_WORKBASEL,mT_BASECHAN9L
    return
;-- Channel AN10 ---------------
mTouch_UpdateBaselineAN10
        banksel mT_BUTTONSTATUS
        btfsc   mT_BUTTONSTATUS,7           ;button pressed
    return                                  ;yes, so don't update baseline for this channel
        movff   mT_SAMPCHAN10H,mT_WORKSAMPH ;take sample value
        movff   mT_SAMPCHAN10L,mT_WORKSAMPL
        movff   mT_BASECHAN10H,mT_WORKBASEH ;take baseline
        movff   mT_BASECHAN10L,mT_WORKBASEL
        rcall   mTouch_CalculateBaseline
        movff   mT_WORKBASEH,mT_BASECHAN10H ;move back updated baseline
        movff   mT_WORKBASEL,mT_BASECHAN10L
    return

;-------------------------------
mTouch_CalculateBaseline   
        banksel mT_WORKSAMPL
        movf    mT_WORKSAMPL,W              ;BASELINE-SAMPCHAN
        subwf   mT_WORKBASEL,W
        movf    mT_WORKSAMPH,W
        subwfb  mT_WORKBASEH,W
        btfss   WREG,7                      ;result negative?
        bra     $ + .8        
        infsnz  mT_WORKBASEL                ;yes, so increment baseline
        incf    mT_WORKBASEH
    return
        movlw   .1                          ;no, so decrement baseline
        subwf   mT_WORKBASEL
        movlw   .0
        subwfb  mT_WORKBASEH
    return



;------------------------------------------------------------------------------
