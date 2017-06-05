EESchema Schematic File Version 2
LIBS:power
LIBS:device
LIBS:transistors
LIBS:conn
LIBS:linear
LIBS:regul
LIBS:74xx
LIBS:cmos4000
LIBS:adc-dac
LIBS:memory
LIBS:xilinx
LIBS:microcontrollers
LIBS:dsp
LIBS:microchip
LIBS:analog_switches
LIBS:motorola
LIBS:texas
LIBS:intel
LIBS:audio
LIBS:interface
LIBS:digital-audio
LIBS:philips
LIBS:display
LIBS:cypress
LIBS:siliconi
LIBS:opto
LIBS:atmel
LIBS:contrib
LIBS:valves
LIBS:nanolib
LIBS:nanomount-cache
EELAYER 26 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 1 1
Title ""
Date ""
Rev ""
Comp ""
Comment1 ""
Comment2 ""
Comment3 ""
Comment4 ""
$EndDescr
$Comp
L CONN_02X13 J1
U 1 1 59307677
P 3100 3650
F 0 "J1" H 3100 4350 50  0000 C CNN
F 1 "CONN_02X13" V 3100 3650 50  0000 C CNN
F 2 "" H 3100 2500 50  0001 C CNN
F 3 "" H 3100 2500 50  0001 C CNN
	1    3100 3650
	1    0    0    -1  
$EndComp
Text GLabel 3350 3050 2    60   Input ~ 0
5V
Text GLabel 2850 3050 0    60   Input ~ 0
3V3
Text GLabel 2850 3350 0    60   Input ~ 0
DIRAZ
Text GLabel 2850 3550 0    60   Input ~ 0
BRKAZ
Text GLabel 2850 3650 0    60   Input ~ 0
PWMAZ
Text GLabel 2850 3750 0    60   Input ~ 0
THAZ
Text GLabel 2850 3950 0    60   Input ~ 0
DIREL
Text GLabel 2850 4050 0    60   Input ~ 0
BRKEL
Text GLabel 2850 4150 0    60   Input ~ 0
PWMEL
Text GLabel 3350 3250 2    60   Input ~ 0
GND
Text GLabel 3350 3350 2    60   Input ~ 0
ELA
Text GLabel 3350 3450 2    60   Input ~ 0
ELB
Text GLabel 3350 3750 2    60   Input ~ 0
AZEND
Text GLabel 3350 3850 2    60   Input ~ 0
ELEND
Text GLabel 3350 4050 2    60   Input ~ 0
AZB
Text GLabel 3350 4150 2    60   Input ~ 0
AZA
Text GLabel 3350 4250 2    60   Input ~ 0
THEL
Text GLabel 4700 4250 0    60   Input ~ 0
GND
Text GLabel 4700 4050 0    60   Input ~ 0
5V
Text GLabel 5500 3350 2    60   Input ~ 0
PWMEL
Text GLabel 5500 3250 2    60   Input ~ 0
PWMAZ
$Comp
L ArduinoNano U1
U 1 1 59307DA0
P 5100 3550
F 0 "U1" H 5500 3550 60  0000 C CNN
F 1 "ArduinoNano" H 4850 3550 60  0000 C CNN
F 2 "" H 4750 3550 60  0001 C CNN
F 3 "" H 4750 3550 60  0001 C CNN
	1    5100 3550
	0    1    1    0   
$EndComp
Text GLabel 5500 2950 2    60   Input ~ 0
ELA
Text GLabel 5500 3050 2    60   Input ~ 0
ELB
Text GLabel 5500 3150 2    60   Input ~ 0
AZEND
Text GLabel 5500 3450 2    60   Input ~ 0
ELEND
Text GLabel 5500 3550 2    60   Input ~ 0
AZB
Text GLabel 5500 3650 2    60   Input ~ 0
AZA
Text GLabel 5500 3750 2    60   Input ~ 0
THEL
Text GLabel 5500 3850 2    60   Input ~ 0
BRKEL
Text GLabel 5500 3950 2    60   Input ~ 0
DIREL
Text GLabel 5500 4050 2    60   Input ~ 0
THAZ
Text GLabel 4700 2950 0    60   Input ~ 0
BRKAZ
Text GLabel 4700 3050 0    60   Input ~ 0
DIRAZ
$EndSCHEMATC
