PULSONIX_LIBRARY_ASCII "SamacSys ECAD Model"
//1805407/434495/2.49/21/3/Integrated Circuit

(asciiHeader
	(fileUnits MM)
)
(library Library_1
	(padStyleDef "r80_30"
		(holeDiam 0)
		(padShape (layerNumRef 1) (padShapeType Rect)  (shapeWidth 0.3) (shapeHeight 0.8))
		(padShape (layerNumRef 16) (padShapeType Ellipse)  (shapeWidth 0) (shapeHeight 0))
	)
	(padStyleDef "r275_175"
		(holeDiam 0)
		(padShape (layerNumRef 1) (padShapeType Rect)  (shapeWidth 1.75) (shapeHeight 2.75))
		(padShape (layerNumRef 16) (padShapeType Ellipse)  (shapeWidth 0) (shapeHeight 0))
	)
	(textStyleDef "Normal"
		(font
			(fontType Stroke)
			(fontFace "Helvetica")
			(fontHeight 1.27)
			(strokeWidth 0.127)
		)
	)
	(patternDef "QFN50P300X400X80-21N" (originalName "QFN50P300X400X80-21N")
		(multiLayer
			(pad (padNum 1) (padStyleRef r80_30) (pt -1.5, 1.25) (rotation 90))
			(pad (padNum 2) (padStyleRef r80_30) (pt -1.5, 0.75) (rotation 90))
			(pad (padNum 3) (padStyleRef r80_30) (pt -1.5, 0.25) (rotation 90))
			(pad (padNum 4) (padStyleRef r80_30) (pt -1.5, -0.25) (rotation 90))
			(pad (padNum 5) (padStyleRef r80_30) (pt -1.5, -0.75) (rotation 90))
			(pad (padNum 6) (padStyleRef r80_30) (pt -1.5, -1.25) (rotation 90))
			(pad (padNum 7) (padStyleRef r80_30) (pt -0.75, -2) (rotation 0))
			(pad (padNum 8) (padStyleRef r80_30) (pt -0.25, -2) (rotation 0))
			(pad (padNum 9) (padStyleRef r80_30) (pt 0.25, -2) (rotation 0))
			(pad (padNum 10) (padStyleRef r80_30) (pt 0.75, -2) (rotation 0))
			(pad (padNum 11) (padStyleRef r80_30) (pt 1.5, -1.25) (rotation 90))
			(pad (padNum 12) (padStyleRef r80_30) (pt 1.5, -0.75) (rotation 90))
			(pad (padNum 13) (padStyleRef r80_30) (pt 1.5, -0.25) (rotation 90))
			(pad (padNum 14) (padStyleRef r80_30) (pt 1.5, 0.25) (rotation 90))
			(pad (padNum 15) (padStyleRef r80_30) (pt 1.5, 0.75) (rotation 90))
			(pad (padNum 16) (padStyleRef r80_30) (pt 1.5, 1.25) (rotation 90))
			(pad (padNum 17) (padStyleRef r80_30) (pt 0.75, 2) (rotation 0))
			(pad (padNum 18) (padStyleRef r80_30) (pt 0.25, 2) (rotation 0))
			(pad (padNum 19) (padStyleRef r80_30) (pt -0.25, 2) (rotation 0))
			(pad (padNum 20) (padStyleRef r80_30) (pt -0.75, 2) (rotation 0))
			(pad (padNum 21) (padStyleRef r275_175) (pt 0, 0) (rotation 0))
		)
		(layerContents (layerNumRef 18)
			(attr "RefDes" "RefDes" (pt 0, 0) (textStyleRef "Normal") (isVisible True))
		)
		(layerContents (layerNumRef Courtyard_Top)
			(line (pt -2.125 2.625) (pt 2.125 2.625) (width 0.05))
		)
		(layerContents (layerNumRef Courtyard_Top)
			(line (pt 2.125 2.625) (pt 2.125 -2.625) (width 0.05))
		)
		(layerContents (layerNumRef Courtyard_Top)
			(line (pt 2.125 -2.625) (pt -2.125 -2.625) (width 0.05))
		)
		(layerContents (layerNumRef Courtyard_Top)
			(line (pt -2.125 -2.625) (pt -2.125 2.625) (width 0.05))
		)
		(layerContents (layerNumRef 28)
			(line (pt -1.5 2) (pt 1.5 2) (width 0.025))
		)
		(layerContents (layerNumRef 28)
			(line (pt 1.5 2) (pt 1.5 -2) (width 0.025))
		)
		(layerContents (layerNumRef 28)
			(line (pt 1.5 -2) (pt -1.5 -2) (width 0.025))
		)
		(layerContents (layerNumRef 28)
			(line (pt -1.5 -2) (pt -1.5 2) (width 0.025))
		)
		(layerContents (layerNumRef 28)
			(line (pt -1.5 1.5) (pt -1 2) (width 0.025))
		)
		(layerContents (layerNumRef 18)
			(arc (pt -1.9, 2) (radius 0) (width 0.25))
		)
	)
	(symbolDef "ISL91107IRTNZ-T7A" (originalName "ISL91107IRTNZ-T7A")

		(pin (pinNum 1) (pt 0 mils 0 mils) (rotation 0) (pinLength 200 mils) (pinDisplay (dispPinName true)) (pinName (text (pt 230 mils -25 mils) (rotation 0]) (justify "Left") (textStyleRef "Normal"))
		))
		(pin (pinNum 2) (pt 0 mils -100 mils) (rotation 0) (pinLength 200 mils) (pinDisplay (dispPinName true)) (pinName (text (pt 230 mils -125 mils) (rotation 0]) (justify "Left") (textStyleRef "Normal"))
		))
		(pin (pinNum 3) (pt 0 mils -200 mils) (rotation 0) (pinLength 200 mils) (pinDisplay (dispPinName true)) (pinName (text (pt 230 mils -225 mils) (rotation 0]) (justify "Left") (textStyleRef "Normal"))
		))
		(pin (pinNum 4) (pt 0 mils -300 mils) (rotation 0) (pinLength 200 mils) (pinDisplay (dispPinName true)) (pinName (text (pt 230 mils -325 mils) (rotation 0]) (justify "Left") (textStyleRef "Normal"))
		))
		(pin (pinNum 5) (pt 0 mils -400 mils) (rotation 0) (pinLength 200 mils) (pinDisplay (dispPinName true)) (pinName (text (pt 230 mils -425 mils) (rotation 0]) (justify "Left") (textStyleRef "Normal"))
		))
		(pin (pinNum 6) (pt 0 mils -500 mils) (rotation 0) (pinLength 200 mils) (pinDisplay (dispPinName true)) (pinName (text (pt 230 mils -525 mils) (rotation 0]) (justify "Left") (textStyleRef "Normal"))
		))
		(pin (pinNum 7) (pt 400 mils -1200 mils) (rotation 90) (pinLength 200 mils) (pinDisplay (dispPinName true)) (pinName (text (pt 425 mils -970 mils) (rotation 90]) (justify "Left") (textStyleRef "Normal"))
		))
		(pin (pinNum 8) (pt 500 mils -1200 mils) (rotation 90) (pinLength 200 mils) (pinDisplay (dispPinName true)) (pinName (text (pt 525 mils -970 mils) (rotation 90]) (justify "Left") (textStyleRef "Normal"))
		))
		(pin (pinNum 9) (pt 600 mils -1200 mils) (rotation 90) (pinLength 200 mils) (pinDisplay (dispPinName true)) (pinName (text (pt 625 mils -970 mils) (rotation 90]) (justify "Left") (textStyleRef "Normal"))
		))
		(pin (pinNum 10) (pt 700 mils -1200 mils) (rotation 90) (pinLength 200 mils) (pinDisplay (dispPinName true)) (pinName (text (pt 725 mils -970 mils) (rotation 90]) (justify "Left") (textStyleRef "Normal"))
		))
		(pin (pinNum 11) (pt 1200 mils 0 mils) (rotation 180) (pinLength 200 mils) (pinDisplay (dispPinName true)) (pinName (text (pt 970 mils -25 mils) (rotation 0]) (justify "Right") (textStyleRef "Normal"))
		))
		(pin (pinNum 12) (pt 1200 mils -100 mils) (rotation 180) (pinLength 200 mils) (pinDisplay (dispPinName true)) (pinName (text (pt 970 mils -125 mils) (rotation 0]) (justify "Right") (textStyleRef "Normal"))
		))
		(pin (pinNum 13) (pt 1200 mils -200 mils) (rotation 180) (pinLength 200 mils) (pinDisplay (dispPinName true)) (pinName (text (pt 970 mils -225 mils) (rotation 0]) (justify "Right") (textStyleRef "Normal"))
		))
		(pin (pinNum 14) (pt 1200 mils -300 mils) (rotation 180) (pinLength 200 mils) (pinDisplay (dispPinName true)) (pinName (text (pt 970 mils -325 mils) (rotation 0]) (justify "Right") (textStyleRef "Normal"))
		))
		(pin (pinNum 15) (pt 1200 mils -400 mils) (rotation 180) (pinLength 200 mils) (pinDisplay (dispPinName true)) (pinName (text (pt 970 mils -425 mils) (rotation 0]) (justify "Right") (textStyleRef "Normal"))
		))
		(pin (pinNum 16) (pt 1200 mils -500 mils) (rotation 180) (pinLength 200 mils) (pinDisplay (dispPinName true)) (pinName (text (pt 970 mils -525 mils) (rotation 0]) (justify "Right") (textStyleRef "Normal"))
		))
		(pin (pinNum 17) (pt 400 mils 700 mils) (rotation 270) (pinLength 200 mils) (pinDisplay (dispPinName true)) (pinName (text (pt 425 mils 470 mils) (rotation 90]) (justify "Right") (textStyleRef "Normal"))
		))
		(pin (pinNum 18) (pt 500 mils 700 mils) (rotation 270) (pinLength 200 mils) (pinDisplay (dispPinName true)) (pinName (text (pt 525 mils 470 mils) (rotation 90]) (justify "Right") (textStyleRef "Normal"))
		))
		(pin (pinNum 19) (pt 600 mils 700 mils) (rotation 270) (pinLength 200 mils) (pinDisplay (dispPinName true)) (pinName (text (pt 625 mils 470 mils) (rotation 90]) (justify "Right") (textStyleRef "Normal"))
		))
		(pin (pinNum 20) (pt 700 mils 700 mils) (rotation 270) (pinLength 200 mils) (pinDisplay (dispPinName true)) (pinName (text (pt 725 mils 470 mils) (rotation 90]) (justify "Right") (textStyleRef "Normal"))
		))
		(pin (pinNum 21) (pt 800 mils 700 mils) (rotation 270) (pinLength 200 mils) (pinDisplay (dispPinName true)) (pinName (text (pt 825 mils 470 mils) (rotation 90]) (justify "Right") (textStyleRef "Normal"))
		))
		(line (pt 200 mils 500 mils) (pt 1000 mils 500 mils) (width 6 mils))
		(line (pt 1000 mils 500 mils) (pt 1000 mils -1000 mils) (width 6 mils))
		(line (pt 1000 mils -1000 mils) (pt 200 mils -1000 mils) (width 6 mils))
		(line (pt 200 mils -1000 mils) (pt 200 mils 500 mils) (width 6 mils))
		(attr "RefDes" "RefDes" (pt 1050 mils 700 mils) (justify Left) (isVisible True) (textStyleRef "Normal"))
		(attr "Type" "Type" (pt 1050 mils 600 mils) (justify Left) (isVisible True) (textStyleRef "Normal"))

	)
	(compDef "ISL91107IRTNZ-T7A" (originalName "ISL91107IRTNZ-T7A") (compHeader (numPins 21) (numParts 1) (refDesPrefix IC)
		)
		(compPin "1" (pinName "LX2_1") (partNum 1) (symPinNum 1) (gateEq 0) (pinEq 0) (pinType Unknown))
		(compPin "2" (pinName "LX2_2") (partNum 1) (symPinNum 2) (gateEq 0) (pinEq 0) (pinType Unknown))
		(compPin "3" (pinName "PGND_1") (partNum 1) (symPinNum 3) (gateEq 0) (pinEq 0) (pinType Unknown))
		(compPin "4" (pinName "PGND_2") (partNum 1) (symPinNum 4) (gateEq 0) (pinEq 0) (pinType Unknown))
		(compPin "5" (pinName "LX1_1") (partNum 1) (symPinNum 5) (gateEq 0) (pinEq 0) (pinType Unknown))
		(compPin "6" (pinName "LX1_2") (partNum 1) (symPinNum 6) (gateEq 0) (pinEq 0) (pinType Unknown))
		(compPin "7" (pinName "PVIN_1") (partNum 1) (symPinNum 7) (gateEq 0) (pinEq 0) (pinType Unknown))
		(compPin "8" (pinName "PVIN_2") (partNum 1) (symPinNum 8) (gateEq 0) (pinEq 0) (pinType Unknown))
		(compPin "9" (pinName "PVIN_3") (partNum 1) (symPinNum 9) (gateEq 0) (pinEq 0) (pinType Unknown))
		(compPin "10" (pinName "PVIN_4") (partNum 1) (symPinNum 10) (gateEq 0) (pinEq 0) (pinType Unknown))
		(compPin "16" (pinName "FB") (partNum 1) (symPinNum 11) (gateEq 0) (pinEq 0) (pinType Unknown))
		(compPin "15" (pinName "SGND") (partNum 1) (symPinNum 12) (gateEq 0) (pinEq 0) (pinType Unknown))
		(compPin "14" (pinName "MODE") (partNum 1) (symPinNum 13) (gateEq 0) (pinEq 0) (pinType Unknown))
		(compPin "13" (pinName "EN") (partNum 1) (symPinNum 14) (gateEq 0) (pinEq 0) (pinType Unknown))
		(compPin "12" (pinName "VIN") (partNum 1) (symPinNum 15) (gateEq 0) (pinEq 0) (pinType Unknown))
		(compPin "11" (pinName "NC") (partNum 1) (symPinNum 16) (gateEq 0) (pinEq 0) (pinType Unknown))
		(compPin "21" (pinName "EP") (partNum 1) (symPinNum 17) (gateEq 0) (pinEq 0) (pinType Unknown))
		(compPin "20" (pinName "VOUT_4") (partNum 1) (symPinNum 18) (gateEq 0) (pinEq 0) (pinType Unknown))
		(compPin "19" (pinName "VOUT_3") (partNum 1) (symPinNum 19) (gateEq 0) (pinEq 0) (pinType Unknown))
		(compPin "18" (pinName "VOUT_2") (partNum 1) (symPinNum 20) (gateEq 0) (pinEq 0) (pinType Unknown))
		(compPin "17" (pinName "VOUT_1") (partNum 1) (symPinNum 21) (gateEq 0) (pinEq 0) (pinType Unknown))
		(attachedSymbol (partNum 1) (altType Normal) (symbolName "ISL91107IRTNZ-T7A"))
		(attachedPattern (patternNum 1) (patternName "QFN50P300X400X80-21N")
			(numPads 21)
			(padPinMap
				(padNum 1) (compPinRef "1")
				(padNum 2) (compPinRef "2")
				(padNum 3) (compPinRef "3")
				(padNum 4) (compPinRef "4")
				(padNum 5) (compPinRef "5")
				(padNum 6) (compPinRef "6")
				(padNum 7) (compPinRef "7")
				(padNum 8) (compPinRef "8")
				(padNum 9) (compPinRef "9")
				(padNum 10) (compPinRef "10")
				(padNum 11) (compPinRef "11")
				(padNum 12) (compPinRef "12")
				(padNum 13) (compPinRef "13")
				(padNum 14) (compPinRef "14")
				(padNum 15) (compPinRef "15")
				(padNum 16) (compPinRef "16")
				(padNum 17) (compPinRef "17")
				(padNum 18) (compPinRef "18")
				(padNum 19) (compPinRef "19")
				(padNum 20) (compPinRef "20")
				(padNum 21) (compPinRef "21")
			)
		)
		(attr "Manufacturer_Name" "Renesas Electronics")
		(attr "Manufacturer_Part_Number" "ISL91107IRTNZ-T7A")
		(attr "Mouser Part Number" "968-ISL91107IRTNZT7A")
		(attr "Mouser Price/Stock" "https://www.mouser.co.uk/ProductDetail/Renesas-Intersil/ISL91107IRTNZ-T7A/?qs=DXptKE%252B5xICfvCrqtryiaQ%3D%3D")
		(attr "Arrow Part Number" "ISL91107IRTNZ-T7A")
		(attr "Arrow Price/Stock" "https://www.arrow.com/en/products/isl91107irtnz-t7a/renesas-electronics")
		(attr "Description" "Integrated FET Regulators High Efficiency Buck-Boost Regulator with 4.1A Switches")
		(attr "<Hyperlink>" "https://www.renesas.com/en-us/www/doc/datasheet/isl91107ir.pdf")
		(attr "<Component Height>" "0.8")
		(attr "<STEP Filename>" "ISL91107IRTNZ-T7A.stp")
		(attr "<STEP Offsets>" "X=0;Y=0;Z=0")
		(attr "<STEP Rotation>" "X=0;Y=0;Z=0")
	)

)
