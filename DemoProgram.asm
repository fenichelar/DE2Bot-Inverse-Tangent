; DemoProgram.asm
; 
; Demos the GoTo function.
; Takes in 2 coordinate inputs from the switches, and goes to the desired coordinates in the shortest path possible.
; 
; Alec Fenichel, Mike Lewis, Billbang Sayasean, Qu Xu
; ECE 2031 - L09
; 2015.04.26


ORG			&H000			; Begin program at x000

;***************************************************************
;* Initialization
;***************************************************************
Init:
	; Always a good idea to make sure the robot
	; stops in the event of a reset.
	LOAD	Zero
	OUT		LVELCMD			; Stop motors
	OUT		RVELCMD
	OUT		SONAREN			; Disable sonar (optional)
	
	CALL	SetupI2C		; Configure the I2C to read the battery voltage
	CALL	BattCheck		; Get battery voltage (and end if too low).

WaitForSafety:
	; Wait for safety switch to be toggled
	IN		XIO				; XIO contains SAFETY signal
	AND		Mask4			; SAFETY signal is bit 4
	JPOS	WaitForUser		; If ready, jump to wait for PB3
	IN		TIMER			; We'll use the timer value to
	AND		Mask1			;  blink LED17 as a reminder to toggle SW17
	SHIFT	8				; Shift over to LED17
	OUT		XLEDS			; LED17 blinks at 2.5Hz (10Hz/4)
	JUMP	WaitForSafety
	
WaitForUser:
	; Wait for user to press PB3
	IN		TIMER			; We'll blink the LEDs above PB3
	AND		Mask1
	SHIFT	5				; Both LEDG6 and LEDG7
	STORE	Temp			; (overkill, but looks nice)
	SHIFT	1
	OR		Temp
	OUT		XLEDS
	IN		XIO				; XIO contains KEYs
	AND		Mask2			; KEY3 mask (KEY0 is reset and can't be read)
	JPOS	WaitForUser		; not ready (KEYs are active-low, hence JPOS)
	LOAD	Zero
	OUT		XLEDS			; clear LEDs once ready to continue


;***************************************************************
;* Main code
;***************************************************************
Main: ; "Real" program starts here.
	OUT		RESETPOS		; reset odometer in case wheels moved after programming	
	CALL	UARTClear		; empty the UART receive FIFO of old data

	LOADI	90
	OUT		LCDEN

DemoGoTo:
	; Get Inputs
	CALL	GetInputA
	CALL	GetInputB
	; Store Inputs
	LOAD	InputA
	STORE	goToX
	LOAD	InputB
	STORE	goToY
	; Call GoTo
	CALL	GoTo
DemoGoToDone:
	; Blink LED6 and LED7
	IN		TIMER
	AND		Mask1
	SHIFT	5
	STORE	Temp
	SHIFT	1
	OR		Temp
	OUT		XLEDS
	; Disabled LEDS and continue when KEY3 is pressed
	IN		XIO
	AND		Mask2
	JPOS	DemoGoToDone
	LOAD	Zero
	OUT		XLEDS
	JUMP	DemoGoTo

Die:
; Sometimes it's useful to permanently stop execution.
; This will also catch the execution if it accidentally
; falls through from above.
	LOAD	Zero			; Stop everything.
	OUT		LVELCMD
	OUT		RVELCMD
	OUT		SONAREN
	LOAD	DEAD			; An indication that we are dead
	OUT		SSEG2
Forever:
	JUMP	Forever			; Do this forever.


;***************************************************************
;* Test Functions
;***************************************************************
TestArcTan:
	; Get Inputs
	CALL	GetInputA
	CALL	GetInputB
	; Store Inputs
	LOAD	InputA
	STORE	arcTanX
	LOAD	InputB
	STORE	arcTanY
	; Call Inverse Tangent
	CALL	ArcTan
	LOAD	arcTanTheta
	OUT		LCD4
	STORE	rotateToTheta
	CALL	RotateTo
TestArcTanDone:
	; Blink LED6 and LED7
	IN		TIMER
	AND		Mask1
	SHIFT	5
	STORE	Temp
	SHIFT	1
	OR		Temp
	OUT		XLEDS
	; Disabled LEDS and continue when KEY3 is pressed
	IN		XIO
	AND		Mask2
	JPOS	TestArcTanDone
	LOAD	Zero
	OUT		XLEDS
	RETURN

TestRotateTo:
	CALL	GetInputA
	LOAD	InputA
	STORE	rotateToTheta
	CALL	RotateTo
	RETURN

TestMultiplyDivide:
	; Get Inputs
	CALL	GetInputA
	CALL	GetInputB
	; Divide
	LOAD	InputA
	DIV		InputB
	OUT		LCD4
	; Multiply
	LOAD	InputA
	MULT	InputB
	OUT		LCD6
TestMultiplyDivideDone:
	; Blink LED6 and LED7
	IN		TIMER
	AND		Mask1
	SHIFT	5
	STORE	Temp
	SHIFT	1
	OR		Temp
	OUT		XLEDS
	; Disabled LEDS and continue when KEY3 is pressed
	IN		XIO
	AND		Mask2
	JPOS	TestMultiplyDivideDone
	LOAD	Zero
	OUT		XLEDS
	RETURN


;***************************************************************
;* Helper Functions
;***************************************************************
GetInputA:
	; Output SWITCHES to LCD1
	IN		SWITCHES
	OUT		LCD1

	; Blink LED4 and LED5
	IN		TIMER
	AND		Mask1
	SHIFT	3
	STORE	Temp
	SHIFT	1
	OR		Temp
	OUT		XLEDS

	; Store SWITCHES in InputA and disabled LEDS when KEY2 is pressed
	IN		XIO
	AND		Mask1
	JPOS	GetInputA
	LOAD	Zero
	OUT		XLEDS
	IN		SWITCHES
	STORE	InputA
	RETURN

GetInputB:
	; Output SWITCHES to LCD3
	IN		SWITCHES
	OUT		LCD3

	; Blink LED2 and LED3
	IN		TIMER
	AND		Mask1
	SHIFT	1
	STORE	Temp
	SHIFT	1
	OR		Temp
	OUT		XLEDS
	
	; Store SWITCHES in InputB and disabled LEDS when KEY1 is pressed
	IN		XIO
	AND		Mask0
	JPOS	GetInputB
	LOAD	Zero
	OUT		XLEDS
	IN		SWITCHES
	STORE	InputB
	RETURN

; Subroutine to wait (block) for 1 second
Wait1:
	OUT		TIMER
Wloop:
	IN		TIMER
	OUT		XLEDS			; User-feedback that a pause is occurring.
	ADDI	-10				; 1 second in 10Hz.
	JNEG	Wloop
	RETURN

; Subroutine to wait the number of timer counts currently in AC
WaitAC:
	STORE	WaitTime
	OUT		Timer
WACLoop:
	IN		Timer
	OUT		XLEDS			; User-feedback that a pause is occurring.
	SUB		WaitTime
	JNEG	WACLoop
	RETURN
	
; This subroutine will get the battery voltage,
; and stop program execution if it is too low.
; SetupI2C must be executed prior to this.
BattCheck:
	CALL	GetBattLvl
	JZERO	BattCheck		; A/D hasn't had time to initialize
	SUB		MinBatt
	JNEG	DeadBatt
	ADD		MinBatt			; get original value back
	RETURN
; If the battery is too low, we want to make
; sure that the user realizes it...
DeadBatt:
	LOAD	Four
	OUT		BEEP			; start beep sound
	CALL	GetBattLvl		; get the battery level
	OUT		SSEG1			; display it everywhere
	OUT		SSEG2
	OUT		LCD
	LOAD	Zero
	ADDI	-1				; 0xFFFF
	OUT		LEDS			; all LEDs on
	OUT		XLEDS
	CALL	Wait1			; 1 second
	Load	Zero
	OUT		BEEP			; stop beeping
	LOAD	Zero
	OUT		LEDS			; LEDs off
	OUT		XLEDS
	CALL	Wait1			; 1 second
	JUMP	DeadBatt		; repeat forever
	
; Subroutine to read the A/D (battery voltage)
; Assumes that SetupI2C has been run
GetBattLvl:
	LOAD	I2CRCmd			; 0x0190 (write 0B, read 1B, addr 0x90)
	OUT		I2C_CMD			; to I2C_CMD
	OUT		I2C_RDY			; start the communication
	CALL	BlockI2C		; wait for it to finish
	IN		I2C_DATA		; get the returned data
	RETURN

; Subroutine to configure the I2C for reading batt voltage
; Only needs to be done once after each reset.
SetupI2C:
	CALL	BlockI2C		; wait for idle
	LOAD	I2CWCmd			; 0x1190 (write 1B, read 1B, addr 0x90)
	OUT		I2C_CMD			; to I2C_CMD register
	LOAD	Zero			; 0x0000 (A/D port 0, no increment)
	OUT		I2C_DATA		; to I2C_DATA register
	OUT		I2C_RDY			; start the communication
	CALL	BlockI2C		; wait for it to finish
	RETURN
	
; Subroutine to block until I2C device is idle
BlockI2C:
	LOAD	Zero
	STORE	Temp			; Used to check for timeout
BI2CL:
	LOAD	Temp
	ADDI	1				; this will result in ~0.1s timeout
	STORE	Temp
	JZERO	I2CError		; Timeout occurred; error
	IN		I2C_RDY			; Read busy signal
	JPOS	BI2CL			; If not 0, try again
	RETURN					; Else return
I2CError:
	LOAD	Zero
	ADDI	&H12C			; "I2C"
	OUT		SSEG1
	OUT		SSEG2			; display error message
	JUMP	I2CError

; Subroutine to send AC value through the UART,
; formatted for default base station code:
; [ AC(15..8) | AC(7..0)]
; Note that special characters such as \lf are
; escaped with the value 0x1B, thus the literal
; value 0x1B must be sent as 0x1B1B, should it occur.
UARTSend:
	STORE	UARTTemp
	SHIFT	-8
	ADDI	-27				; escape character
	JZERO	UEsc1
	ADDI	27
	OUT		UART_DAT
	JUMP	USend2
UEsc1:
	ADDI	27
	OUT		UART_DAT
	OUT		UART_DAT
USend2:
	LOAD	UARTTemp
	AND		LowByte
	ADDI	-27				; escape character
	JZERO	UEsc2
	ADDI	27
	OUT		UART_DAT
	RETURN
UEsc2:
	ADDI	27
	OUT		UART_DAT
	OUT		UART_DAT
	RETURN

; Subroutine to send a newline to the computer log
UARTNL:
	LOAD	NL
	OUT		UART_DAT
	SHIFT	-8
	OUT		UART_DAT
	RETURN

; Subroutine to clear the internal UART receive FIFO.
UARTClear:
	IN		UART_DAT
	JNEG	UARTClear
	RETURN


;***************************************************************
;* Main Function
;***************************************************************
; Subroutine to send the robot to coordinates specified in goToX and goToY
GoTo:
	IN		XPOS			; get the current X position
	OUT		LCD4
	STORE	Temp
	LOAD	goToX
	SUB		Temp
	STORE	arcTanX			; store change in X position required

	IN 		YPOS			; get the current Y position
	OUT		LCD6
	STORE	Temp
	LOAD	goToY
	SUB		Temp
	STORE	arcTanY			; store change in Y position required

GoToGetXYMagnitude:
	LOAD	arcTanX
	STORE	arcTanXMagnitude
	JPOS	GoToGetYMagnitude
	
	SUB		arcTanX
	SUB		arcTanX
	STORE	arcTanXMagnitude
	
GoToGetYMagnitude:
	LOAD	arcTanY
	STORE	arcTanYMagnitude
	JPOS	CheckPosition
	
	SUB		arcTanY
	SUB		arcTanY
	STORE	arcTanYMagnitude

CheckPosition:
	ADDI	-10
	JPOS	GoToCalculateTheta
	LOAD	arcTanXMagnitude
	ADDI	-11
	JNEG	PositionReached

GoToCalculateTheta:
	LOAD	goToThetaCount
	JPOS	MoveForward
	ADDI	5
	STORE	goToThetaCount
	CALL	ArcTan
	LOAD	arcTanTheta
	STORE	rotateToTheta
	CALL	RotateTo

MoveForward:
	ADDI	-1
	STORE	goToThetaCount
	LOAD	FSlow
	OUT		LVELCMD
	OUT		RVELCMD
	JUMP	GoTo

PositionReached:
	LOAD	Zero
	STORE	goToThetaCount
	OUT		LVELCMD
	OUT		RVELCMD
	RETURN	

; Subroutine to rotate the robot to angle specified in rotateToTheta
RotateTo:
	LOAD	rotateToTheta
	OUT		SSEG2
	IN		THETA			; get the current angular position
	OUT		SSEG1
	STORE	Temp
	LOAD	rotateToTheta
	SUB		Temp
	ADDI	180
	JNEG	LessThanNegative180
	ADDI	-360
	JPOS	GreaterThanPositive180
	ADDI	180
	JUMP	ExecuteRotate
LessThanNegative180:
	ADDI	360
	JUMP	ExecuteRotate
GreaterThanPositive180:
	ADDI	-360
ExecuteRotate:
	ADDI	1
	JNEG	TurnRight		; if difference is negative turn right
	ADDI	-2
	JPOS	TurnLeft		; if difference is positive turn left
	LOAD	Zero			; otherwise difference is 0 so done
	OUT		LVELCMD
	OUT		RVELCMD
	RETURN

TurnLeft:
	LOAD	RSlow
	JUMP	Turn

TurnRight:
	LOAD	FSlow
	JUMP	Turn
	
Turn:
	STORE	Temp
	OUT		LVELCMD
	LOAD	Zero
	SUB		Temp
	OUT		RVELCMD
	JUMP	rotateTo

;***************************************************************
;* Inverse Tangent Function
;***************************************************************
ArcTan:
	LOAD	arcTanY
	JZERO	ArcTanY0CheckXSign
	
	LOAD	arcTanX
	JZERO	ArcTanX0CheckYSign


ArcTanGetXYMagnitude:
	LOAD	arcTanX
	STORE	arcTanXMagnitude
	JPOS	ArcTanGetYMagnitude
	
	SUB		arcTanX
	SUB		arcTanX
	STORE	arcTanXMagnitude
	
ArcTanGetYMagnitude:
	LOAD	arcTanY
	STORE	arcTanYMagnitude
	JPOS	ArcTanCalculateRatio
	
	SUB		arcTanY
	SUB		arcTanY
	STORE	arcTanYMagnitude


ArcTanCalculateRatio:
	LOAD	arcTanXMagnitude
	SUB		arcTanYMagnitude
	JNEG	ArcTanYGreater

ArcTanXGreater:
	LOAD	arcTanYMagnitude
	DIV		arcTanXMagnitude
	JUMP	ArcTanCalculate

ArcTanYGreater:
	LOAD	arcTanXMagnitude
	DIV		arcTanYMagnitude

	
ArcTanCalculate:
	JPOS	ArcTanCalculateTaylorSeries
	
	STORE	arcTanRatio
	SUB		arcTanRatio
	SUB		arcTanRatio

ArcTanCalculateTaylorSeries:
	; Subtract center of polynomial from input so that the value can be used in later calculations.
	SUB	 	arcTanRatioOffset
	STORE	arcTanRatio
	
	; Calculate first-order term, ADD to output, using value of input still in accumulator
	MULT	arcTanCoefficient1
	STORE	arcTanTheta
	
	; Calculate the square of the input, STORE for further multiplication later.
	LOAD	arcTanRatio
	MULT	arcTanRatio
	
	; Calculate second-order term, ADD to output.
	MULT	arcTanCoefficient2
	ADD		arcTanTheta
	
	; Calculate zeroth-order term, use to initialize output.
	ADD 	arcTanCoefficient0
	STORE	arcTanTheta
	
	JNEG	ArcTanReturn0
	
	LOADI	-45
	SHIFT	8
	ADD		arcTanTheta
	JNEG	ArcTanFixedPointToInteger
	
	LOADI	45
	STORE	arcTanTheta
	JUMP	ArcTanCorrectQuadrant


ArcTanFixedPointToInteger:
	LOAD	arcTanTheta
	AND		Mask7
	JZERO	ArcTanRoundFixedPointDown

ArcTanRoundFixedPointUp:
	LOAD	arcTanTheta
	ADDI	&B0000000100000000
	JUMP	ArcTanTruncateFixedPoint

ArcTanRoundFixedPointDown:
	LOAD	arcTanTheta

ArcTanTruncateFixedPoint:
	SHIFT	-8
	STORE	arcTanTheta


ArcTanCorrectOctant:
	LOAD	arcTanYMagnitude
	SUB		arcTanXMagnitude
	JNEG	ArcTanCorrectQuadrant
	
	LOADI	90
	SUB		arcTanTheta
	STORE	arcTanTheta


ArcTanCorrectQuadrant:
	LOAD	arcTanX
	JNEG	ArcTanQuadrants2And3


ArcTanQuadrants1And4:
	LOAD	arcTanY
	JNEG	ArcTanQuadrant4

ArcTanQuadrant1:
	RETURN

ArcTanQuadrant4:
	LOAD	Deg360
	SUB		arcTanTheta
	JUMP	ArcTanReturnAccumulator


ArcTanQuadrants2And3:
	LOAD	arcTanY
	JNEG	ArcTanQuadrant3

ArcTanQuadrant2:
	LOAD	Deg180
	SUB		arcTanTheta
	JUMP	ArcTanReturnAccumulator

ArcTanQuadrant3:
	LOAD	Deg180
	ADD		arcTanTheta
	JUMP	ArcTanReturnAccumulator


ArcTanX0CheckYSign:
	LOAD	arcTanY
	JNEG	ArcTanReturn270

ArcTanReturn90:
	LOAD	Deg90
	JUMP	ArcTanReturnAccumulator

ArcTanReturn270:
	LOAD	Deg270
	JUMP	ArcTanReturnAccumulator


ArcTanY0CheckXSign:
	LOAD	arcTanX
	JNEG	ArcTanReturn180

ArcTanReturn0:
	LOAD	zero
	JUMP	ArcTanReturnAccumulator

ArcTanReturn180:
	LOAD	Deg180


ArcTanReturnAccumulator:
	STORE	arcTanTheta
	RETURN


;***************************************************************
;* Variables
;***************************************************************
Temp:				DW 0
WaitTime:			DW 0
InputA:				DW 0
InputB:				DW 0
rotateToTheta:		DW 0
goToX:				DW 0
goToY:				DW 0
goToThetaCount:		DW 0
arcTanX:			DW 0
arcTanY:			DW 0
arcTanTheta:		DW 0
arcTanRatio:		DW 0
arcTanXMagnitude:	DW 0
arcTanYMagnitude:	DW 0

;***************************************************************
;* Constants
;* (though there is nothing stopping you from writing to these)
;***************************************************************
NegOne:		DW -1
Zero:		DW 0
One:		DW 1
Two:		DW 2
Three:		DW 3
Four:		DW 4
Five:		DW 5
Six:		DW 6
Seven:		DW 7
Eight:		DW 8
Nine:		DW 9
Ten:		DW 10

; Some bit masks.
; Masks of multiple bits can be constructed by ORing these
; 1-bit masks together.
Mask0:		DW &B00000001
Mask1:		DW &B00000010
Mask2:		DW &B00000100
Mask3:		DW &B00001000
Mask4:		DW &B00010000
Mask5:		DW &B00100000
Mask6:		DW &B01000000
Mask7:		DW &B10000000
LowByte:	DW &HFF			; binary 00000000 11111111
HighByte:	DW &HFF00		; binary 11111111 00000000
AllBytes:	DW &HFFFF		; binary 11111111 11111111
LowNibl:	DW &HF			; 0000 0000 0000 1111

; some useful movement values
OneMeter:	DW 961			; ~1m in 1.05mm units
HalfMeter:	DW 481			; ~0.5m in 1.05mm units
TwoFeet:	DW 586			; ~2ft in 1.05mm units
Deg90:		DW 90			; 90 degrees in odometer units
Deg180:		DW 180			; 180
Deg270:		DW 270			; 270
Deg360:		DW 360			; can never actually happen; for math only
FSlow:		DW 100			; 100 is about the lowest velocity value that will move
RSlow:		DW -100
FMid:		DW 350			; 350 is a medium speed
RMid:		DW -350
FFast:		DW 500			; 500 is almost max speed (511 is max)
RFast:		DW -500

MinBatt:	DW 130			; 13.0V - minimum safe battery voltage
I2CWCmd:	DW &H1190		; write one i2c byte, read one byte, addr 0x90
I2CRCmd:	DW &H0190		; write nothing, read one byte, addr 0x90
UARTTemp:	DW 0
NL:			DW &H0A1B
DEAD:		DW &HDEAD

arcTanRatioOffset:	DW &B0000000001111010 ; 0.4765625 ~= 0.4750
arcTanCoefficient0:	DW &B0001100101101000 ; 25.40625 ~= 25.4077
arcTanCoefficient1:	DW &B0010111011000000 ; 46.75 ~= 46.7482
arcTanCoefficient2:	DW &B1110111111100010 ; -16.1171875 ~= -18.1176

;***************************************************************
;* IO address space map
;***************************************************************
SWITCHES:	EQU &H00		; slide switches
LEDS:		EQU &H01		; red LEDs
TIMER:		EQU &H02		; timer, usually running at 10 Hz
XIO:		EQU &H03		; pushbuttons and some misc. inputs
SSEG1:		EQU &H04		; seven-segment display (4-digits only)
SSEG2:		EQU &H05		; seven-segment display (4-digits only)
LCD:		EQU &H06		; primitive 4-digit LCD display
XLEDS:		EQU &H07		; Green LEDs (and Red LED16+17)
BEEP:		EQU &H0A		; Control the beep
CTIMER:		EQU &H0C		; Configurable timer for interrupts
LPOS:		EQU &H80		; left wheel encoder position (read only)
LVEL:		EQU &H82		; current left wheel velocity (read only)
LVELCMD:	EQU &H83		; left wheel velocity command (write only)
RPOS:		EQU &H88		; same values for right wheel...
RVEL:		EQU &H8A		; ...
RVELCMD:	EQU &H8B		; ...
I2C_CMD:	EQU &H90		; I2C module's CMD register,
I2C_DATA:	EQU &H91		; ... DATA register,
I2C_RDY:	EQU &H92		; ... and BUSY register
UART_DAT:	EQU &H98		; UART data
UART_RDY:	EQU &H98		; UART status
SONAR:		EQU &HA0		; base address for more than 16 registers....
DIST0:		EQU &HA8		; the eight sonar distance readings
DIST1:		EQU &HA9		; ...
DIST2:		EQU &HAA		; ...
DIST3:		EQU &HAB		; ...
DIST4:		EQU &HAC		; ...
DIST5:		EQU &HAD		; ...
DIST6:		EQU &HAE		; ...
DIST7:		EQU &HAF		; ...
SONALARM:	EQU &HB0		; Write alarm distance; read alarm register
SONARINT:	EQU &HB1		; Write mask for sonar interrupts
SONAREN:	EQU &HB2		; register to control which sonars are enabled
XPOS:		EQU &HC0		; Current X-position (read only)
YPOS:		EQU &HC1		; Y-position
THETA:		EQU &HC2		; Current rotational position of robot (0-359)
RESETPOS:	EQU &HC3		; write anything here to reset odometry to 0
LCD0:		EQU &H70		; the eight LCDs
LCD1:		EQU &H71		; ...
LCD2:		EQU &H72		; ...
LCD3:		EQU &H73		; ...
LCD4:		EQU &H74		; ...
LCD5:		EQU &H75		; ...
LCD6:		EQU &H76		; ...
LCD7:		EQU &H77		; ...
LCDEN:		EQU &H78		; register to control which LCDs are enabled
