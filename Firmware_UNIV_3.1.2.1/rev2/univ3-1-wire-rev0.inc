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
;   Filename:               univ3-1-wire-rev0.inc
;   Associated diagram:     none
;   Author:                 Jacek Siwilo, based on Dallas Semiconductor
;   Description:            UNIV 3 CPU 1-wire functions		
;==============================================================================
;   Revision History
;   Rev:  Date:     Details:
;   0     08.2012   Original version									
;==============================================================================
;   Description:
;   Processor has 8MHz clock and 0.5�s per instruction cycle.
;   TRISDQ      - 1-wire port (TRIS reg)
;   PORTDQ      - 1-wire port (PORT reg)
;   DQ          - 1-wire pin
;   ROMCODE0    - first byte of 64bit rom code
;   TMPO        - time counter reg
;   PDBYTE      - presence reg
;   COUNT       - counter reg
;   COUNT2      - counter reg 2
;   IOBYTE      - input/output byte reg
;   IOBYTE2     - temp input/output byte reg
;   CRCREG      - CRC register
;==============================================================================


; These constants are standard 1-Wire ROM commands

		constant SRCHROM=0xF0
		constant RDROM=0x33
 		constant MTCHROM=0x55
		constant SKPROM=0xCC

;------------------------------------------------------------------------------
;       Dallas Semiconductor 1-Wire MACROS
;------------------------------------------------------------------------------
OW_HIZ:MACRO
		bsf         TRISDQ, DQ					; Make DQ pin High Z
		ENDM
;-------------------------------
OW_LO:MACRO
		bcf         PORTDQ, DQ                  ; Clear the DQ bit
		bcf         TRISDQ, DQ                  ; Make DQ pin an output
		ENDM
;-------------------------------
WAIT:MACRO TIME
;Delay for TIME �s.
;Variable time must be in multiples of 5�s.
        nop
        nop
		movlw       (TIME/5)-1                  ;1�s
        nop
		movwf       TMP0                        ;1�s
        nop
		call        Wait5U                      ;2�s
		ENDM

;------------------------------------------------------------------------------
;       Dallas Semiconductor 1-Wire ROUTINES
;------------------------------------------------------------------------------
Wait5U:
;This takes 5�S to complete
        nop
        nop
        nop
		nop                                     ;1�s
        nop
		nop                                     ;1�s
        nop
		decfsz      TMP0,F                      ;0.5�s or 1�s
		goto        Wait5U                      ;1�s
		retlw 0                                 ;1�s
;-------------------------------
DSReset:
		OW_HIZ                                  ; Start with the line high
		clrf 		PDBYTE                      ; Clear the PD byte
		OW_LO
		WAIT        .500                        ; Drive Low for 500�s
		OW_HIZ
		WAIT        .70                         ; Release line and wait 70�s for PD Pulse
		btfss       PORTDQ,DQ                   ; Read for a PD Pulse
		incf        PDBYTE,F                    ; Set PDBYTE to 1 if get a PD Pulse
		WAIT        .400                        ; Wait 400�s after PD Pulse
		retlw 0
;-------------------------------
DSRXByte:										 ; Byte read is stored in IOBYTE
		movlw       .8
		movwf       COUNT                       ; Set COUNT equal to 8 to count the bits
DSRXLP:
		OW_LO
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop                                     ; Bring DQ low for 6�s
		OW_HIZ
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop                                     ; Change to HiZ and Wait 4�s
		bcf			STATUS,C
		btfsc		PORTDQ,DQ
		bsf			STATUS,C
		rrcf        IOBYTE,F                    ; Shift C into IOBYTE
		WAIT        .50                         ; Wait 50�s to end of time slot
		decfsz      COUNT,F                     ; Decrement the bit counter
		goto        DSRXLP
		retlw       0
;-------------------------------
DSTXByte:                                       ; Byte to send starts in IOBYTE
		movwf       IOBYTE                      ; We send it from IOBYTE
		movlw       .8
		movwf       COUNT                       ; Set COUNT equal to 8 to count the bits
DSTXLP:
		OW_LO
		nop
		nop
		nop
		nop
		nop
		nop                                     ; Drive the line low for 3�s
		rrcf        IOBYTE,F
		btfsc       STATUS,C                    ; Check the LSB of IOBYTE for 1 or 0
		bsf         TRISDQ,DQ                   ; HiZ the line  if LSB is 1
      	WAIT        .60                         ; Continue driving line for 60�s
      	OW_HIZ                                  ; Release the line for pullup
      	nop
		nop
		nop
      	nop                                     ; Recovery time of 2�s
      	decfsz      COUNT,F                     ; Decrement the bit counter
      	goto        DSTXLP
      	retlw       0
;-------------------------------
CalcCrc:
		movlw		.8
		movwf		COUNT
CalcCrcLoop:
		movf		CRCREG,W					;find D7 bit
		xorwf		IOBYTE,W
		rrcf		WREG						;get D7 bit
		bnc		D7Zero							;invert bit  <4:3> if D7=1
		movlw		b'00011000'
		xorwf		CRCREG,F
D7Zero
		rrcf		CRCREG						;done CRCREG
		rrncf		IOBYTE						;take next bit
		decfsz		COUNT
		bra		CalcCrcLoop
		retlw		0
;-------------------------------
ReadBit											;bit is written to 7th bit of IOBYTE
		OW_LO
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop                                     ; Bring DQ low for 6�s
		OW_HIZ
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop                                     ; Change to HiZ and Wait 4�s
		bcf			STATUS,C
		btfsc		PORTDQ,DQ
		bsf			STATUS,C
		clrf		IOBYTE
		rrcf        IOBYTE,F                    ; Shift C into IOBYTE
		WAIT        .50                         ; Wait 50�s to end of time slot
		retlw		0 
;-------------------------------
SendBit0
		OW_LO
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop                                     ; Drive the line low for 5�s
		nop
		nop
      	WAIT        .60                         ; Continue driving line for 60�s
      	OW_HIZ                                  ; Release the line for pullup
      	nop
      	nop
		nop
		nop
		retlw		0 
;------------------------------- 
SendBit1
		OW_LO
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop                                     ; Drive the line low for 5�s
		nop
		nop
		bsf         TRISDQ,DQ                   ; HiZ the line  if LSB is 1
      	WAIT        .60                         ; Continue driving line for 60�s
      	OW_HIZ                                  ; Release the line for pullup
      	nop
      	nop
      	nop
      	nop  
		retlw		0 
;-------------------------------
CheckROM:										;checks if there is only one device on network
		movlw       SRCHROM                  	;send Search ROM Command (0xF0)
		call        DSTXByte
		;read device
		movlw		.64
		movwf		COUNT
CheckROMLoop:
		;read 1 bit
		call		ReadBit
		movff		IOBYTE,IOBYTE2
		;read 2 bit
		call		ReadBit
		movf		IOBYTE,W
		cpfseq		IOBYTE2
		bra			$ + 4
	bra			ManyDevices
		btfsc		IOBYTE2,7					;check
		bra			$ + 8
		call		SendBit0
		bra			$ + 6
		call		SendBit1
		decfsz		COUNT
	bra			CheckROMLoop
		retlw		0
ManyDevices:
		retlw		1
;-------------------------------
ReadROM:									    ;reads 64 bit ROM CODE (returns 0 if ok or 1 if CRC problem)
		movlw       RDROM                  	    ;Send Search ROM Command (0x33)
		call        DSTXByte
		clrf		CRCREG					    ;init CRC
		;read device
		movlw		.8
		movwf		COUNT2
		lfsr		FSR0,ROMCODE0
ReadROMLoop:		
		call        DSRXByte           		    ;ROMCODE0-7
		call		CalcCrc
		movff       IOBYTE,POSTINC0                        
		decfsz		COUNT2
	bra			ReadROMLoop
		tstfsz		CRCREG					    ;test if CRC correct
	bra			$ + 4
		retlw		0						    ;CRC ok
		retlw		1					        ;wrong CRC