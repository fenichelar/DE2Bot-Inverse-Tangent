;***************************************************************
;* Inverse Tangent Function
;*
;* Team: Castaways
;* Members: Alec Fenichel, Mike Lewis, Billbang Sayasean, Qu Xu
;* Section: L09
;*
;* Place X coordinate into arcTanX
;* Place Y coordinate into arcTanY
;* Call ArcTan
;* Result will be placed in arcTanTheta
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


arcTanX:			DW 0
arcTanY:			DW 0
arcTanTheta:		DW 0
arcTanRatio:		DW 0
arcTanXMagnitude:	DW 0
arcTanYMagnitude:	DW 0
arcTanRatioOffset:	DW &B0000000001111010 ; 0.4765625 ~= 0.4750
arcTanCoefficient0:	DW &B0001100101101000 ; 25.40625 ~= 25.4077
arcTanCoefficient1:	DW &B0010111011000000 ; 46.75 ~= 46.7482
arcTanCoefficient2:	DW &B1110111111100010 ; -16.1171875 ~= -18.1176
