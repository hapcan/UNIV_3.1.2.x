	NOLIST
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

;==============================================================================
;                         DEFAULT CONFIG DATA
;==============================================================================
EEPROM      code    0xF00008
    ;button 
    DE      0x09,0x09,0x09,0x09,0x09,0x09,0x09  ;buttons send only on & off msg
;            |____|____|____|____|____|____|_________________________________________________ on & off
    ;thermometer
            code    0xF00014
    DE      0xFF,0x08,0x00,0x01,0x40,0x02,0x00,0x00
;            |    |    |    |    |    |    |____|__________________________________ temp offset = 0deg
;            |    |    |    |    |    |___________________________________________ hysteresis = 0.5deg
;            |    |    |    |____|________________________________________ last saved setpoint = 20deg
;            |    |____|___________________________________ take setpoint last saved value at power up
;
    ;touch button thresholds: press = 60 & release = 30
EEPROM2     code    0xF00050
    DE      0x00,0x3C,0x00,0x1E,0x00,0x3C,0x00,0x1E,0x00,0x3C,0x00,0x1E
    DE      0x00,0x3C,0x00,0x1E,0x00,0x3C,0x00,0x1E,0x00,0x3C,0x00,0x1E
FLASH       code    0x008800
    ;toggle led when button touched
    #define     ID2     0x01            ;node number
    #define     ID3     0x01            ;group number
    DE      0x30,0x10,ID2 ,ID3 ,0xFF,0xFF,0x01,0xFF,0xFF,0xFF,0xFF,0xFF,0x01,0x01,0x01,0x01 ;button1
    DE      0x01,0x01,0x01,0x01,0x00,0x00,0x00,0x00,0x02,0x01,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF
    DE      0x30,0x10,ID2 ,ID3 ,0xFF,0xFF,0x02,0xFF,0xFF,0xFF,0xFF,0xFF,0x01,0x01,0x01,0x01 ;button2
    DE      0x01,0x01,0x01,0x01,0x00,0x00,0x00,0x00,0x02,0x02,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF
    DE      0x30,0x10,ID2 ,ID3 ,0xFF,0xFF,0x03,0xFF,0xFF,0xFF,0xFF,0xFF,0x01,0x01,0x01,0x01 ;button3
    DE      0x01,0x01,0x01,0x01,0x00,0x00,0x00,0x00,0x02,0x04,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF
    DE      0x30,0x10,ID2 ,ID3 ,0xFF,0xFF,0x04,0xFF,0xFF,0xFF,0xFF,0xFF,0x01,0x01,0x01,0x01 ;button4
    DE      0x01,0x01,0x01,0x01,0x00,0x00,0x00,0x00,0x02,0x08,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF
    DE      0x30,0x10,ID2 ,ID3 ,0xFF,0xFF,0x05,0xFF,0xFF,0xFF,0xFF,0xFF,0x01,0x01,0x01,0x01 ;button5
    DE      0x01,0x01,0x01,0x01,0x00,0x00,0x00,0x00,0x02,0x10,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF
    DE      0x30,0x10,ID2 ,ID3 ,0xFF,0xFF,0x06,0xFF,0xFF,0xFF,0xFF,0xFF,0x01,0x01,0x01,0x01 ;button6
    DE      0x01,0x01,0x01,0x01,0x00,0x00,0x00,0x00,0x02,0x20,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF
    DE      0x30,0x10,ID2 ,ID3 ,0xFF,0xFF,0x07,0xFF,0xFF,0xFF,0xFF,0xFF,0x01,0x01,0x01,0x01 ;button7
    DE      0x01,0x01,0x01,0x01,0x00,0x00,0x00,0x00,0x02,0x3F,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF

;==============================================================================
;                         FIRMWARE REGISTER DEFINITIONS
;==============================================================================
;CONFIG0  - button 1 config, bit<0>- on <1>- 400ms <2>- 4s <3>- off <4>- <400ms <5>- <4s <6>- >4s
;CONFIG1  - button 2 config, bit<0>- on <1>- 400ms <2>- 4s <3>- off <4>- <400ms <5>- <4s <6>- >4s
;CONFIG2  - button 3 config, bit<0>- on <1>- 400ms <2>- 4s <3>- off <4>- <400ms <5>- <4s <6>- >4s
;CONFIG3  - button 4 config, bit<0>- on <1>- 400ms <2>- 4s <3>- off <4>- <400ms <5>- <4s <6>- >4s
;CONFIG4  - button 5 config, bit<0>- on <1>- 400ms <2>- 4s <3>- off <4>- <400ms <5>- <4s <6>- >4s
;CONFIG5  - button 6 config, bit<0>- on <1>- 400ms <2>- 4s <3>- off <4>- <400ms <5>- <4s <6>- >4s
;CONFIG6  - all buttons at the same time config, bit<0>- on <1>- 400ms <2>- 4s <3>- off <4>- <400ms <5>- <4s <6>- >4s
E_SETPOINTMSB       SET CONFIG14    ;setpoint at power up. Possible values: 0xFC90-0x07D0 (-55deg - +125deg), value=0x0800 then THERM value will be taken from last saved
E_SETPOINTLSB       SET CONFIG15    ;setpoint at power up LSB byte
E_SETPOINTSAVEDMSB  SET CONFIG16    ;last saved setpoint. Possible values: 0xFC90-0x07D0 (-55deg - +125deg)
E_SETPOINTSAVEDLSB  SET CONFIG17    ;last saved setpoint LSB byte
E_HYSTERE           SET CONFIG18    ;hysteresis <7:2>.<1:0>(2 bits after decimal point). Possible values: (0= 0deg, 1= 0.25deg, 2= 0.50deg, 3= 0.75deg etc... 255= 63.75deg)
E_OFFSETMSB         SET CONFIG19    ;temperature offset. Possible values: 0xFC90-0x07D0 (-55deg - +125deg)
E_OFFSETLSB         SET CONFIG20    ;temperature offset LSB byte
E_TEMPCHANGE        SET CONFIG21    ;minimum temperature change when temperature message is sent. Possible values: 1-255 (1= 0.25deg, 2= 0.50deg, 3= 0.75deg etc... 255= 63.75deg), value=0 disables messages at any change
E_TCSENS            SET CONFIG22    ;temperature controller sensitivity - defines the minimum error (difference between setpoint and measured temperature) change to which the controller must react. Possible values: (0= 0.0625deg, 1= 0.1250deg, 2= 0.1875deg etc... 255= 16.0000deg)
E_TCPWMPERIOD       SET CONFIG23    ;temperature controller PWM period. Possible values: 0-255 (1min - 256min)
E_MODSTATESOURCE    SET CONFIG24    ;power up source: bit <0> - thermostat, <1> - controller. Possible values: '1' - power up from last saved, '0' - from defined power up values
E_MODSTATESET       SET CONFIG25    ;defined thermometer states at the power up: bit <0> - thermostat, <1> - controller. Possible values: '1' - turned ON, '0' - turned OFF
E_MODSTATESAVED     SET CONFIG26    ;last saved thermometer states: bit <0> - thermostat, <1> - controller. Possible values: '1' - turned ON, '0' - turned OFF
E_TMCOUNTER         SET CONFIG27    ;temperature periodical message timer. Possible values:1-255 (1s-255s), value=0 disables periodical messages
E_THCOUNTER         SET CONFIG28    ;thermostat periodical message timer. Possible values:1-255 (1s-255s), value=0 disables periodical messages
E_TCCOUNTER         SET CONFIG29    ;controler periodical message timer. Possible values:1-255 (1s-255s), value=0 disables periodical messages
E_ENABLEDPER        SET CONFIG30    ;enabled peripheral: bit <0> - temperature, <1> - thermostat, <2> - controller. Possible values: '1' - enabled, '0' - disabled
ENABLETEMP          EQU 0           ;E_ENABLEDPER bits
ENABLETHERM         EQU 1
ENABLECTRL          EQU 2
;button hardware definitions
BUTCnfg             SET  CONFIG0    ;reg where button config starts (CONFIG0)
;temperature controller definitions
PWMHEATTAB1 EQU     0x008600        ;heating PWM duty cycle table address
PWMCOOLTAB1 EQU     0x008620        ;cooling PWM duty cycle table address

;button procedures regs
ButtonsA            EQU  H'0000'    ;current button 1-6 states
DiodesA             EQU  H'0001'    ;current diode 1-8 states
BUT1Cnt             EQU  H'0002'    ;button pressed timer
BUT2Cnt             EQU  H'0003'
BUT3Cnt             EQU  H'0004'
BUT4Cnt             EQU  H'0005'
BUT5Cnt             EQU  H'0006'
BUT6Cnt             EQU  H'0007'
BUT7Cnt             EQU  H'0008'

;mTouch regs
mT_SAMPCHAN2H       EQU  H'0700'    ;average sample
mT_SAMPCHAN2L       EQU  H'0701'
mT_SAMPCHAN3H       EQU  H'0702'
mT_SAMPCHAN3L       EQU  H'0703'
mT_SAMPCHAN4H       EQU  H'0704'
mT_SAMPCHAN4L       EQU  H'0705'
mT_SAMPCHAN8H       EQU  H'0706'
mT_SAMPCHAN8L       EQU  H'0707'
mT_SAMPCHAN9H       EQU  H'0708'
mT_SAMPCHAN9L       EQU  H'0709'
mT_SAMPCHAN10H      EQU  H'070A'
mT_SAMPCHAN10L      EQU  H'070B'
mT_BASECHAN2H       EQU  H'070C'    ;base line
mT_BASECHAN2L       EQU  H'070D'
mT_BASECHAN3H       EQU  H'070E'
mT_BASECHAN3L       EQU  H'070F'
mT_BASECHAN4H       EQU  H'0710'
mT_BASECHAN4L       EQU  H'0711'
mT_BASECHAN8H       EQU  H'0712'
mT_BASECHAN8L       EQU  H'0713'
mT_BASECHAN9H       EQU  H'0714'
mT_BASECHAN9L       EQU  H'0715'
mT_BASECHAN10H      EQU  H'0716'
mT_BASECHAN10L      EQU  H'0717'
mT_TRIMAN2          EQU  H'0718'    ;current source trim bits
mT_TRIMAN3          EQU  H'0719'
mT_TRIMAN4          EQU  H'071A'
mT_TRIMAN8          EQU  H'071B'
mT_TRIMAN9          EQU  H'071C'
mT_TRIMAN10         EQU  H'071D'
mT_WORKSAMPH        EQU  H'071E'    ;working sample value
mT_WORKSAMPL        EQU  H'071F'
mT_WORKBASEH        EQU  H'0720'    ;working baseline
mT_WORKBASEL        EQU  H'0721'
mT_WORKONTHRESH     EQU  H'0722'    ;working threshold ON value
mT_WORKONTHRESL     EQU  H'0723'
mT_WORKOFFTHRESH    EQU  H'0724'    ;working threshold OFF value
mT_WORKOFFTHRESL    EQU  H'0725'
mT_WORKTHRESH       EQU  H'0726'    ;working threshold value
mT_WORKTHRESL       EQU  H'0727'
mT_BUTTONSTATUS     EQU  H'0728'    ;decoded buttons status
mT_CNTR1            EQU  H'0729'    ;working counter
mT_SAMPCNTR         EQU  H'072A'    ;A/D sample counter
mT_SAMPH            EQU  H'072B'    ;A/D sample
mT_SAMPL            EQU  H'072C'
mT_TRIMBITS         EQU  H'072D'    ;working trim bits reg
mT_DEACTIVTIMER     EQU  H'072E'    ;touch buttons deactivation timer
mT_LEDSHADOW        EQU  H'072F'    ;shadow reg of led statuses

;mTouch config registers
mT_ONTHRESAN2H      EQU  H'0250'    ;button 1
mT_ONTHRESAN2L      EQU  H'0251'
mT_OFFTHRESAN2H     EQU  H'0252'
mT_OFFTHRESAN2L     EQU  H'0253'
mT_ONTHRESAN3H      EQU  H'0254'    ;button 2
mT_ONTHRESAN3L      EQU  H'0255'
mT_OFFTHRESAN3H     EQU  H'0256'
mT_OFFTHRESAN3L     EQU  H'0257'
mT_ONTHRESAN4H      EQU  H'0258'    ;button 3
mT_ONTHRESAN4L      EQU  H'0259'
mT_OFFTHRESAN4H     EQU  H'025A'
mT_OFFTHRESAN4L     EQU  H'025B'
mT_ONTHRESAN8H      EQU  H'0264'    ;button 6
mT_ONTHRESAN8L      EQU  H'0265'
mT_OFFTHRESAN8H     EQU  H'0266'
mT_OFFTHRESAN8L     EQU  H'0267'
mT_ONTHRESAN9H      EQU  H'025C'    ;button 4
mT_ONTHRESAN9L      EQU  H'025D'
mT_OFFTHRESAN9H     EQU  H'025E'
mT_OFFTHRESAN9L     EQU  H'025F'
mT_ONTHRESAN10H     EQU  H'0260'    ;button 5
mT_ONTHRESAN10L     EQU  H'0261'
mT_OFFTHRESAN10H    EQU  H'0262'
mT_OFFTHRESAN10L    EQU  H'0263'

;temperature registers
TEMPLSB     EQU     H'0012'		;read current LSB TEMP byte
TEMPMSB     EQU     H'0013'		;read current MSB TEMP byte
TEMPLSBPRE  EQU     H'0014'		;previous value of TEMP LSB byte 
TEMPMSBPRE  EQU     H'0015'		;previous value of TEMP MSB byte 
TTEMPLSB    EQU     H'0016'		;temporary LSB TEMP
TTEMPMSB    EQU     H'0017'		;temporary MSB TEMP
SETPOINTLSB EQU     H'0018'		;setpoint of temperature LSB reg
SETPOINTMSB EQU     H'0019'		;setpoint of temperature MSB reg
OFFSETMSB   EQU     H'001A'     ;temperature offset MSB byte
OFFSETLSB   EQU     H'001B'     ;temperature offset LSB byte
ERRORREG    EQU     H'001C'		;1-wire error reg
CNVRTFLAG   EQU     H'001D'		;flag (indicates conversion in progress)
TMCOUNTERH  EQU     H'001E'     ;temperature periodical message counter
TMCOUNTERL  EQU     H'001F'
MODSTATE    EQU     H'0020'     ;module current status
MODSTATENEW EQU     H'0021'     ;module requested status
MODTHERM    EQU     0           ;thermostat module
MODPWMCTRL  EQU     1           ;controller module

;thermostat registers
THERMOS     EQU     H'0022' 	;thermostat status
HYSTELSB    EQU     H'0023'		;hysteresis LSB reg
HYSTEMSB    EQU     H'0024'		;hysteresis MSB reg
THCOUNTERH  EQU     H'0025'     ;thermostat periodical message counter
THCOUNTERL  EQU     H'0026'

;temperature controller registers
TCERRLSB    EQU     H'0027'     ;temperature controller error LSB byte (difference between set thermostat and current temperature)
TCERRMSB    EQU     H'0028'     ;temperature controller error MSB byte
TCERRLSBPRE EQU     H'0029'     ;temperature controller previous value of error LSB byte
TCERRMSBPRE EQU     H'002A'     ;temperature controller previous value of error MSB byte
TCDERRLSB   EQU     H'002B'		;temperature controller delta error LSB byte (difference between current and previous error)
TCDERRMSB   EQU     H'002C'		;temperature controller delta error MSB byte
TCWORKLSB   EQU     H'002D'		;temperature controller working LSB byte
TCWORKMSB   EQU     H'002E'		;temperature controller working MSB byte
TCHEATPWMV  EQU     H'002F'     ;heating PWM value taken from config table
TCCOOLPWMV  EQU     H'0030'     ;cooling PWM value taken from config table
TCHEATPWMS  EQU     H'0031'     ;heating PWM state
TCCOOLPWMS  EQU     H'0032'     ;cooling PWM state
TCHEATPWMVPRE  EQU  H'0033'     ;previous value of heating PWM value taken from config table
TCCOOLPWMVPRE  EQU  H'0034'     ;previous value of cooling PWM value taken from config table
TCHEATPWMSPRE  EQU  H'0035'     ;previous value of heating PWM state
TCCOOLPWMSPRE  EQU  H'0036'     ;previous value of cooling PWM state
TCSENS      EQU     H'0037'     ;temperature controller sensitivity 
TCPWMPERIOD EQU     H'0038'     ;temperature controller PWM period
TCPWMPELSB  EQU     H'0039'     ;PWM period LSB counter
TCPWMPEMSB  EQU     H'003A'     ;PWM period MSB counter
TCPWMONLSB  EQU     H'003B'     ;PWM ON LSB counter
TCPWMONMSB  EQU     H'003C'     ;PWM ON MSB counter
TCFLAGS     EQU     H'003D'     ;temperature controller flags
HeatState   EQU          0      ;controller in heating state 
CoolState   EQU          1      ;controller in cooling state 
PwmOnEnd    EQU          2      ;end of PWM ON time 
PwmPeriodEnd   EQU       3      ;end of PWM PERIOD 
TCCOUNTERH  EQU     H'003E'     ;controler periodical message counter
TCCOUNTERL  EQU     H'003F'

;1-wire register settings
PORTDQ      EQU     PORTC       ;set 1-wire port & pin
TRISDQ      EQU     TRISC
DQ          EQU     6
ROMCODE0    EQU     H'0040'		;first byte of 64bit rom code 0x40-0x47
TMP0        EQU     H'0048'		;timer reg
PDBYTE      EQU     H'0049'		;presence byte	
COUNT       EQU     H'004A'		;bit counter
COUNT2      EQU     H'004B'
IOBYTE      EQU     H'004C'		;input/output byte
IOBYTE2     EQU     H'004D'		;temp input/output byte
CRCREG      EQU     H'004E'		;CRC reg

;==============================================================================
;                             ROUTINES REGISTERS
;==============================================================================

;------------------------------
;   WORKING INSTRUCTION REGS
;------------------------------
INSTR1      EQU     H'0050'
INSTR2      EQU     H'0051'
INSTR3      EQU     H'0052'
INSTR4      EQU     H'0053'
INSTR5      EQU     H'0054'
INSTR6      EQU     H'0055'
INSTR7      EQU     H'0056'
INSTR8      EQU     H'0057'
TIMER       EQU     H'0058'
SUBTIMER1   EQU     H'0059'
SUBTIMER2   EQU     H'005A'

;------------------------------
;   INTERRUPT SHADOW REGS
;------------------------------
STATUS_HIGH EQU     H'0060'
WREG_HIGH   EQU     H'0061'
BSR_HIGH    EQU     H'0062'
FSR0L_HIGH  EQU     H'0063'
FSR0H_HIGH  EQU     H'0064'
FSR1L_HIGH  EQU     H'0065'
FSR1H_HIGH  EQU     H'0066'
STATUS_LOW  EQU     H'0067'
WREG_LOW    EQU     H'0068'
BSR_LOW     EQU     H'0069'
FSR0L_LOW   EQU     H'006A'
FSR0H_LOW   EQU     H'006B'
FSR1L_LOW   EQU     H'006C'
FSR1H_LOW   EQU     H'006D'
INTCON_SHD  EQU     H'006E'

;------------------------------
;   FIFO REGISTERS
;------------------------------
RXFIFOSIZE  EQU     .42         ;size of CAN RX FIFO buffer
TXFIFOSIZE  EQU     .42         ;size of CAN TX FIFO buffer
RXFIFORAM   EQU     0x300       ;begining of RX FIFO in RAM
TXFIFORAM   EQU     0x500       ;begining of TX FIFO in RAM
;CAN RX FIFO input buffer
RXFIFOIN0   EQU     H'0070'     
RXFIFOIN1   EQU     H'0071'
RXFIFOIN2   EQU     H'0072'
RXFIFOIN3   EQU     H'0073'
RXFIFOIN4   EQU     H'0074'
RXFIFOIN5   EQU     H'0075'
RXFIFOIN6   EQU     H'0076'
RXFIFOIN7   EQU     H'0077'
RXFIFOIN8   EQU     H'0078'
RXFIFOIN9   EQU     H'0079'
RXFIFOIN10  EQU     H'007A'
RXFIFOIN11  EQU     H'007B'
;CAN RX FIFO output buffer
RXFIFO0     EQU     H'007C'     
RXFIFO1     EQU     H'007D'
RXFIFO2     EQU     H'007E'
RXFIFO3     EQU     H'007F'
RXFIFO4     EQU     H'0080'
RXFIFO5     EQU     H'0081'
RXFIFO6     EQU     H'0082'
RXFIFO7     EQU     H'0083'
RXFIFO8     EQU     H'0084'
RXFIFO9     EQU     H'0085'
RXFIFO10    EQU     H'0086'
RXFIFO11    EQU     H'0087'
;CAN TX FIFO input buffer
TXFIFOIN0   EQU     H'0088'     
TXFIFOIN1   EQU     H'0089'
TXFIFOIN2   EQU     H'008A'
TXFIFOIN3   EQU     H'008B'
TXFIFOIN4   EQU     H'008C'
TXFIFOIN5   EQU     H'008D'
TXFIFOIN6   EQU     H'008E'
TXFIFOIN7   EQU     H'008F'
TXFIFOIN8   EQU     H'0090'
TXFIFOIN9   EQU     H'0091'
TXFIFOIN10  EQU     H'0092'
TXFIFOIN11  EQU     H'0093'
;CAN TX FIFO output buffer
TXFIFO0     EQU     H'0094'     
TXFIFO1     EQU     H'0095'
TXFIFO2     EQU     H'0096'
TXFIFO3     EQU     H'0097'
TXFIFO4     EQU     H'0098'
TXFIFO5     EQU     H'0099'
TXFIFO6     EQU     H'009A'
TXFIFO7     EQU     H'009B'
TXFIFO8     EQU     H'009C'
TXFIFO9     EQU     H'009D'
TXFIFO10    EQU     H'009E'
TXFIFO11    EQU     H'009F'
;CAN FIFO RX registers
RXTOP       EQU     H'00A0'     ;FIFO RX stack top
RXCNT       EQU     H'00A1'     ;FIFO RX current stack depth
RXCNTMX     EQU     H'00A2'     ;FIFO RX maximum reached stack depth
RXFIFOR0    EQU     H'00A3'     ;working register
RXFIFOR1    EQU     H'00A4'     ;working register
;CAN FIFO TX registers
TXTOP       EQU     H'00A5'     ;FIFO RX stack top
TXCNT       EQU     H'00A6'     ;FIFO RX current stack depth
TXCNTMX     EQU     H'00A7'     ;FIFO RX maximum reached stack depth
TXFIFOR0    EQU     H'00A8'     ;working register
TXFIFOR1    EQU     H'00A9'     ;working register

;------------------------------
;   HEALTH CHECK REGISTERS
;------------------------------
CANINTCNT   EQU     H'00AA'     ;counts can reinitialization events
TRSMTTIMER  EQU     H'00AB'     ;counts seconds when transmit buffers are busy

;------------------------------
;   UPTIME REGISTERS
;------------------------------
UPTIME0     EQU     H'00AC'
UPTIME1     EQU     H'00AD'
UPTIME2     EQU     H'00AE'
UPTIME3     EQU     H'00AF'

;------------------------------
;   WORKING REGISTERS
;------------------------------
ROUT0       EQU     H'00B0'     ;CanTransmit routine
ROUT1       EQU     H'00B1'     ;ProcessSystemMessage routine (HealthAskClear)
ROUT2       EQU     H'00B2'     ;CheckAgainstBoxes routine
ROUT3       EQU     H'00B3'     ;CheckAgainstBoxes routine
;Flag register
TIMER0_1000ms EQU   H'00BD'     ;Timer 0 interrupt occurred flag
TIMER2_20ms EQU     H'00BE'     ;Timer 2 interrupt occurred flag
EEPROMTIMER EQU     H'00BF'     ;Save node states to EEPROM timer (6s)


;------------------------------
;   BOX REGISTERS
;------------------------------
BOXADR      EQU     H'00C0'
BOXEN       EQU     H'00C1'
SAVEENBITS  EQU     H'00C2'     ;Box enable bits need to be saved into EEPROM flag

BOXMASK1    EQU     H'00D0'     ;enable/disable/toggle box mask
BOXMASK2    EQU     H'00D1'
BOXMASK3    EQU     H'00D2'
BOXMASK4    EQU     H'00D3'
BOXMASK5    EQU     H'00D4'
BOXMASK6    EQU     H'00D5'
BOXMASK7    EQU     H'00D6'
BOXMASK8    EQU     H'00D7'
BOXMASK9    EQU     H'00D8'
BOXMASK10   EQU     H'00D9'
BOXMASK11   EQU     H'00DA'
BOXMASK12   EQU     H'00DB'
BOXMASK13   EQU     H'00DC'
BOXMASK14   EQU     H'00DD'
BOXMASK15   EQU     H'00DE'
BOXMASK16   EQU     H'00DF'

BOXFIL1     EQU     H'00E0'     ;message filter
BOXFIL2     EQU     H'00E1'
BOXFIL3     EQU     H'00E2'
BOXFIL4     EQU     H'00E3'
BOXFIL5     EQU     H'00E4'
BOXFIL6     EQU     H'00E5'
BOXFIL7     EQU     H'00E6'
BOXFIL8     EQU     H'00E7'
BOXFIL9     EQU     H'00E8'
BOXFIL10    EQU     H'00E9'
BOXFIL11    EQU     H'00EA'
BOXFIL12    EQU     H'00EB'
BOXFIL1C    EQU     H'00EC'     ;message filter condition
BOXFIL2C    EQU     H'00ED'
BOXFIL3C    EQU     H'00EE'
BOXFIL4C    EQU     H'00EF'
BOXFIL5C    EQU     H'00F0'
BOXFIL6C    EQU     H'00F1'
BOXFIL7C    EQU     H'00F2'
BOXFIL8C    EQU     H'00F3'
BOXFIL9C    EQU     H'00F4'
BOXFIL10C   EQU     H'00F5'
BOXFIL11C   EQU     H'00F6'
BOXFIL12C   EQU     H'00F7'
BOXINSTR1   EQU     H'00F8'     ;instruction in box
BOXINSTR2   EQU     H'00F9'
BOXINSTR3   EQU     H'00FA'
BOXINSTR4   EQU     H'00FB'
BOXINSTR5   EQU     H'00FC'
BOXINSTR6   EQU     H'00FD'
BOXINSTR7   EQU     H'00FE'
BOXINSTR8   EQU     H'00FF'

;------------------------------
;   CHANNEL REGISTERS
;------------------------------
Instr1Ch1   EQU     H'01B0'
Instr2Ch1   EQU     H'01B1'
Instr3Ch1   EQU     H'01B2'
Instr4Ch1   EQU     H'01B3'
Instr5Ch1   EQU     H'01B4'
Instr6Ch1   EQU     H'01B5'
Instr7Ch1   EQU     H'01B6'
Instr8Ch1   EQU     H'01B7'
TimerCh1    EQU     H'01B8'
SubTmr1Ch1  EQU     H'01B9'
SubTmr2Ch1  EQU     H'01BA'
Instr1Ch2   EQU     H'01BB'
Instr2Ch2   EQU     H'01BC'
Instr3Ch2   EQU     H'01BD'
Instr4Ch2   EQU     H'01BE'
Instr5Ch2   EQU     H'01BF'
Instr6Ch2   EQU     H'01C0'
Instr7Ch2   EQU     H'01C1'
Instr8Ch2   EQU     H'01C2'
TimerCh2    EQU     H'01C3'
SubTmr1Ch2  EQU     H'01C4'
SubTmr2Ch2  EQU     H'01C5'
Instr1Ch3   EQU     H'01C6'
Instr2Ch3   EQU     H'01C7'
Instr3Ch3   EQU     H'01C8'
Instr4Ch3   EQU     H'01C9'
Instr5Ch3   EQU     H'01CA'
Instr6Ch3   EQU     H'01CB'
Instr7Ch3   EQU     H'01CC'
Instr8Ch3   EQU     H'01CD'
TimerCh3    EQU     H'01CE'
SubTmr1Ch3  EQU     H'01CF'
SubTmr2Ch3  EQU     H'01D0'
Instr1Ch4   EQU     H'01D1'
Instr2Ch4   EQU     H'01D2'
Instr3Ch4   EQU     H'01D3'
Instr4Ch4   EQU     H'01D4'
Instr5Ch4   EQU     H'01D5'
Instr6Ch4   EQU     H'01D6'
Instr7Ch4   EQU     H'01D7'
Instr8Ch4   EQU     H'01D8'
TimerCh4    EQU     H'01D9'
SubTmr1Ch4  EQU     H'01DA'
SubTmr2Ch4  EQU     H'01DB'
Instr1Ch5   EQU     H'01DC'
Instr2Ch5   EQU     H'01DD'
Instr3Ch5   EQU     H'01DE'
Instr4Ch5   EQU     H'01DF'
Instr5Ch5   EQU     H'01E0'
Instr6Ch5   EQU     H'01E1'
Instr7Ch5   EQU     H'01E2'
Instr8Ch5   EQU     H'01E3'
TimerCh5    EQU     H'01E4'
SubTmr1Ch5  EQU     H'01E5'
SubTmr2Ch5  EQU     H'01E6'
Instr1Ch6   EQU     H'01E7'
Instr2Ch6   EQU     H'01E8'
Instr3Ch6   EQU     H'01E9'
Instr4Ch6   EQU     H'01EA'
Instr5Ch6   EQU     H'01EB'
Instr6Ch6   EQU     H'01EC'
Instr7Ch6   EQU     H'01ED'
Instr8Ch6   EQU     H'01EE'
TimerCh6    EQU     H'01EF'
SubTmr1Ch6  EQU     H'01F0'
SubTmr2Ch6  EQU     H'01F1'

;==============================================================================
;                             COPPIED EEPROM
;==============================================================================
;eeprom last used address
EEPROM_U    EQU     H'0202'
EEPROM_H    EQU     H'0203'
EEPROM_L    EQU     H'0204'
;flash last used address
FLASH_U     EQU     H'0205'
FLASH_H     EQU     H'0206'
FLASH_L     EQU     H'0207'
;eeprom module config
CONFIG0     EQU     H'0208'
CONFIG1     EQU     H'0209'
CONFIG2     EQU     H'020A'
CONFIG3     EQU     H'020B'
CONFIG4     EQU     H'020C'
CONFIG5     EQU     H'020D'
CONFIG6     EQU     H'020E'
CONFIG7     EQU     H'020F'
CONFIG8     EQU     H'0210'
CONFIG9     EQU     H'0211'
CONFIG10    EQU     H'0212'
CONFIG11    EQU     H'0213'
CONFIG12    EQU     H'0214'
CONFIG13    EQU     H'0215'
CONFIG14    EQU     H'0216'
CONFIG15    EQU     H'0217'
CONFIG16    EQU     H'0218'
CONFIG17    EQU     H'0219'
CONFIG18    EQU     H'021A'
CONFIG19    EQU     H'021B'
CONFIG20    EQU     H'021C'
CONFIG21    EQU     H'021D'
CONFIG22    EQU     H'021E'
CONFIG23    EQU     H'021F'
CONFIG24    EQU     H'0280'
CONFIG25    EQU     H'0281'
CONFIG26    EQU     H'0282'
CONFIG27    EQU     H'0283'
CONFIG28    EQU     H'0284'
CONFIG29    EQU     H'0285'
CONFIG30    EQU     H'0286'
CONFIG31    EQU     H'0287'
CONFIG32    EQU     H'0288'
CONFIG33    EQU     H'0289'
CONFIG34    EQU     H'028A'
;module id
NODENR      EQU     H'0226'     ;node number
GROUPNR     EQU     H'0227'     ;node group number
;health check regs
;res        EQU     H'0228'
;res        EQU     H'0229'
;res        EQU     H'022A'
RXCNTMXEE   EQU     H'022B'     ;maximum achieved CAN RX FIFO depth
TXCNTMXEE   EQU     H'022C'     ;maximum achieved CAN TX FIFO depth
CANINTCNTEE EQU     H'022D'     ;can engine reinitialization number
RXERRCNTEE  EQU     H'022E'     ;CAN receive errors
TXERRCNTEE  EQU     H'022F'     ;CAN transmit errors
;enable box registers
ENBOX08     EQU     H'0240'     ;boxes 1-8
ENBOX16     EQU     H'0241'
ENBOX24     EQU     H'0242'
ENBOX32     EQU     H'0243'
ENBOX40     EQU     H'0244'
ENBOX48     EQU     H'0245'
ENBOX56     EQU     H'0246'
ENBOX64     EQU     H'0247'
ENBOX72     EQU     H'0248'
ENBOX80     EQU     H'0249'
ENBOX88     EQU     H'024A'
ENBOX96     EQU     H'024B'
ENBOX104    EQU     H'024C'
ENBOX112    EQU     H'024D'
ENBOX120    EQU     H'024E'
ENBOX128    EQU     H'024F'     ;boxes 121-128


;==============================================================================
;                   BOOTLOADER REGISTERS - DO NOT CHANGE
;==============================================================================
;CAN receive buffer
RxBCON      EQU     H'0100'
CANFRAME1   EQU     H'0101'
CANFRAME2   EQU     H'0102'
CANNODE     EQU     H'0103'
CANGROUP    EQU     H'0104'
CANDLC      EQU     H'0105'
CAND0       EQU     H'0106'
CAND1       EQU     H'0107'
CAND2       EQU     H'0108'
CAND3       EQU     H'0109'
CAND4       EQU     H'010A'
CAND5       EQU     H'010B'
CAND6       EQU     H'010C'
CAND7       EQU     H'010D'
CANFULL     EQU     H'010E'     ;CAN buffer full flag
FIRMREADY   EQU     H'010F'     ;firmware started completely flag
;------------------------------
;UART receive buffer
UART0       EQU     H'0110'
UART1       EQU     H'0111'
UART2       EQU     H'0112'
UART3       EQU     H'0113'
UART4       EQU     H'0114'
UART5       EQU     H'0115'
UART6       EQU     H'0116'
UART7       EQU     H'0117'
UART8       EQU     H'0118'
UART9       EQU     H'0119'
UART10      EQU     H'011A'
UART11      EQU     H'011B'
UART12      EQU     H'011C'
UART13      EQU     H'011D'
UART14      EQU     H'011E'
UART15      EQU     H'011F'
UART16      EQU     H'0120'
UART17      EQU     H'0121'
UART18      EQU     H'0122'
UART19      EQU     H'0123'
UART20      EQU     H'0124'
UART21      EQU     H'0125'
UART22      EQU     H'0126'
UART23      EQU     H'0127'
UART24      EQU     H'0128'
UART25      EQU     H'0129'
UART26      EQU     H'012A'
UART27      EQU     H'012B'
UART28      EQU     H'012C'
UART29      EQU     H'012D'
UARTOVF     EQU     H'012E'     ;overflow indicator
UARTCNT     EQU     H'012F'     ;uart buffer counter
;------------------------------
;bootloader other regs
UARTON      EQU     H'016F'     ;indicates that UART bootloader is on when 0xFF
;------------------------------
;bootloader shadow registers for high interrupt
STATUS_H    EQU     H'0170'
WREG_H      EQU     H'0171'
BSR_H       EQU     H'0172'
PIR1_H      EQU     H'0173'
PIR4_H      EQU     H'0174'
PIR5_H      EQU     H'0175'
CANSTAT_H   EQU     H'0176'
CANCON_H    EQU     H'0177'
FSR0L_H     EQU     H'0178'
FSR0H_H     EQU     H'0179'
FSR1L_H     EQU     H'017A'
FSR1H_H     EQU     H'017B'
TABLAT_H    EQU     H'017C'
TBLPTRL_H   EQU     H'017D'
TBLPTRH_H   EQU     H'017E'
TBLPTRU_H   EQU     H'017F'
INTCON_H    EQU     H'0180'
EECON1_H    EQU     H'0181'
EEDATA_H    EQU     H'0182'
EEADRH_H    EQU     H'0183'
EEADR_H     EQU     H'0184'
TRISA_H     EQU     H'0185'
ADCON2_H    EQU     H'0186'
ADCON1_H    EQU     H'0187'
ADCON0_H    EQU     H'0188'
ANCON0_H    EQU     H'0189'
ADRESL_H    EQU     H'018A'
ADRESH_H    EQU     H'018B'
RCSTA1_H    EQU     H'018C'
TXSTA1_H    EQU     H'018D'
PMD1_H      EQU     H'018E'
COMSTAT_H   EQU     H'018F'
;------------------------------
;bootloader shadow registers for low interrupt
STATUS_L    EQU     H'0190'
WREG_L      EQU     H'0191'
BSR_L       EQU     H'0192'
PIR1_L      EQU     H'0193'
PIR4_L      EQU     H'0194'
PIR5_L      EQU     H'0195'
CANSTAT_L   EQU     H'0196'
CANCON_L    EQU     H'0197'
FSR0L_L     EQU     H'0198'
FSR0H_L     EQU     H'0199'
FSR1L_L     EQU     H'019A'
FSR1H_L     EQU     H'019B'
TABLAT_L    EQU     H'019C'
TBLPTRL_L   EQU     H'019D'
TBLPTRH_L   EQU     H'019E'
TBLPTRU_L   EQU     H'019F'
INTCON_L    EQU     H'01A0'
EECON1_L    EQU     H'01A1'
EEDATA_L    EQU     H'01A2'
EEADRH_L    EQU     H'01A3'
EEADR_L     EQU     H'01A4'
TRISA_L     EQU     H'01A5'
ADCON2_L    EQU     H'01A6'
ADCON1_L    EQU     H'01A7'
ADCON0_L    EQU     H'01A8'
ANCON0_L    EQU     H'01A9'
ADRESL_L    EQU     H'01AA'
ADRESH_L    EQU     H'01AB'
RCSTA1_L    EQU     H'01AC'
TXSTA1_L    EQU     H'01AD'
PMD1_L      EQU     H'01AE'
COMSTAT_L   EQU     H'01AF'
;==============================================================================
;==============================================================================
;==============================================================================
    LIST
