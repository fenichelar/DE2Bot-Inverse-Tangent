InverseTangent:
	load			inverseTangentY
	jumpIfZero		InverseTangentY0CheckXSign
	
	load			inverseTangentX
	jumpIfZero		InverseTangentX0CheckYSign


InverseTangentCalculateRatio:
	subtract		inverseTangentY
	jumpIfNegative	InverseTangentYGreater

InverseTangentXGreater:
	load			inverseTangentY
	divide			inverseTangentX
	jump			InverseTangentCalculate

InverseTangentYGreater:
	loadImmediate	45
	shift			8
	store			inverseTangentTheta
	
	load			inverseTangentX
	divide			inverseTangentY


InverseTangentCalculate:
	; Subtract center of polynomial from input so that the value can be used in later calculations.
	subtract	 	inverseTangentRatioOffset
	store			inverseTangentRatio
	
	; Calculate first-order term, add to output, using value of input still in accumulator
	multiply		inverseTangentCoefficient1
	add				inverseTangentTheta
	store			inverseTangentTheta
	
	; Calculate the square of the input, store for further multiplication later.
	load			inverseTangentRatio
	multiply		inverseTangentRatio
	;store			inverseTangentRatio
	
	; Calculate second-order term, add to output.
	multiply		inverseTangentCoefficient2
	add				inverseTangentTheta
	
	; Calculate zeroth-order term, use to initialize output.
	add 			inverseTangentCoefficient0
	store			inverseTangentTheta
	
	; Calculate the cube of the input. No need to store due to this being the final term.
	;load			inverseTangentRatio
	;multiply		inverseTangentRatio
	
	; Calculate the third-order term, add to output.
	;multiply		inverseTangentCoefficient3
	;add			inverseTangentTheta
	
	jumpIfNegative	InverseTangentReturn0
	
	loadImmediate	-45
	shift			8
	add				inverseTangentTheta
	jumpIfNegative	InverseTangentFixedPointToInteger
	
	loadImmediate	45
	jump			InverseTangentReturnAccumulator

InverseTangentFixedPointToInteger:
	load			inverseTangentTheta
	and				Mask7
	jumpIfZero		InverseTangentRoundFixedPointDown

InverseTangentRoundFixedPointUp:
	load			inverseTangentTheta
	addImmediate	&B0100000000
	jump			InverseTangentTruncateFixedPoint

InverseTangentRoundFixedPointDown:
	load			inverseTangentTheta

InverseTangentTruncateFixedPoint:
	shift			-8
	store			inverseTangentTheta


InverseTangentCorrectQuadrant:
	load			inverseTangentX
	jumpIfNegative	InverseTangentQuadrants2And3


InverseTangentQuadrants1And4:
	load			inverseTangentY
	jumpIfNegative	InverseTangentQuadrant4

InverseTangentQuadrant1:
	return

InverseTangentQuadrant4:
	load			Deg360
	subtract		inverseTangentTheta
	jump			InverseTangentReturnAccumulator


InverseTangentQuadrants2And3:
	load			inverseTangentY
	jumpIfNegative	InverseTangentQuadrant3

InverseTangentQuadrant2:
	load			Deg180
	subtract		inverseTangentTheta
	jump			InverseTangentReturnAccumulator

InverseTangentQuadrant3:
	load			Deg180
	add				inverseTangentTheta
	jump			InverseTangentReturnAccumulator


InverseTangentX0CheckYSign:
	load			inverseTangentY
	jumpIfNegative	InverseTangentReturn270

InverseTangentReturn90:
	load			Deg90
	jump			InverseTangentReturnAccumulator

InverseTangentReturn270:
	load			Deg270
	jump			InverseTangentReturnAccumulator


InverseTangentY0CheckXSign:
	load			inverseTangentX
	jumpIfNegative	InverseTangentReturn180

InverseTangentReturn0:
	load			zero
	jump			InverseTangentReturnAccumulator

InverseTangentReturn180:
	load			Deg180


InverseTangentReturnAccumulator:
	store			inverseTangentTheta
	return
	
	
InverseTangentRatioOffset:
	DW				&B0000000001111010 ; 0.4765625 ~= 0.4750

InverseTangentCoefficient0:
	DW				&B0001100101101000 ; 25.40625 ~= 25.4077

InverseTangentCoefficient1:
	DW				&B0010111011000000 ; 46.75 ~= 46.7482

InverseTangentCoefficient2:
	DW				&B1110111111100010 ; -16.1171875 ~= -18.1176

;InverseTangentCoefficient3:
;	DW				&B1111110010100110 ; -3.3515625 ~= -3.3519