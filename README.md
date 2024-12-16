# CPE301 Final Project Fall24

## Author
Edgar Rodriguez-Angulo, 12/15/2024

## Description
The code found in this repository is the basis for a primitive swamp cooling system. Put simply, the system this code is intended for monitors water levels, the temperature and humidity of the surrounding environment, is able to control a simple vent, and a user may start, stop, and reset the system. Additionally, changes made in the state of the cooler and the position of the vent are reported in the serial monitor.

### Materials
The components used come from the Mega 2560 "The Most Complete Starter Kit Mega Project" sold by Elegoo.
-Project starter kit: https://www.amazon.com/gp/product/B01EWNUUUA/
The following list of components can be found in the starter kit
-MEGA 2560 controller board (Arduino Board) (https://ww1.microchip.com/downloads/en/devicedoc/atmel-2549-8-bit-avr-microcontroller-atmega640-1280-1281-2560-2561_datasheet.pdf)
-LCD1602 Module (LCD screen) (https://www.waveshare.com/datasheet/LCD_en_PDF/LCD1602.pdf)
-Power supply module (https://components101.com/modules/5v-mb102-breadboard-power-supply-module)
-Stepper motor (https://components101.com/motors/28byj-48-stepper-motor)
-ULN2003 Stepper motor driver module (https://www.electronicoscaldas.com/datasheet/ULN2003A-PCB.pdf)
-Water level detection sensor module (https://www.biomaker.org/block-catalogue/2021/12/17/water-level-sensor-tzt-water-level-sensor)
-DS1307 RTC module (Real Time Clock) (https://www.sparkfun.com/datasheets/Components/DS1307.pdf)
-DHT11 Temperature and Humidity module (https://components101.com/sensors/dht11-temperature-sensor)
-L293D chip (https://components101.com/ics/l293d-pinout-features-datasheet)
-Potentiometer 10k
-Fan blade and 3-6V motor (https://thecustomizewindows.com/2015/08/arduino-3v-dc-motor-control-transistor-ic-more/)
-Power adapter for the power supply module
-Jumper wires
-Female to male wires
-USB cable (Connects the Arduino board to a PC)
-100ohm and 330ohm resistors
-Blue, green, yellow, red LEDs
-Buttons

### Design Overview

LEDs
Four simple LEDs are used to visibly indicate which state the cooler is in. Yellow is disabled, green is idle, blue is running, and red is error. Each LED has a 330 ohm resistor for the power pin.

Sensors
The water level sensor has a minimum threshold of about 300 CHANGE SOON (threshold determined by the datasheet provided above), as it indicates that the water reservoir is less than half full. The temperature and humidity module has a minimum threshold of 700. Both are directly connected to ground and 5V power on the ground and power rails of the breadboard.

Buttons
Three buttons are used to Start, Stop, and Reset the system, and all three are wired up in a pull resistor configuration. Each button uses a 100 ohm resistor.

Stepper Motor and fan
The Arduino board itself cannot safely supply power to the stepper motor and the fan, so instead of being directly connected to the board or to power and ground rails drawing from the board, an external power supply is used. Using female-to-male cables, a small section of the breadboard is reserved to output 5V and GND from the external power supply, and from that, the fan motor, the stepper motor, and the fan motor's driver chip draws power from it.

LCD
I personally was running out of female-to-male cables, so I directly plugged my LCD into a spare breadboard I had, which draws power from the original breadboard's power rails. I then connected jumper cables accordingly. Rather than connecting VSS to a potentiometer, I simply connected it to ground. If you have problems reading the LCD display, then I recommend using a potentiometer.

RTC
This module was plugged into my spare breadboard, and I used jumper cables to connect the module to its respective pins on the Arduino board. It is directly connected to power and ground, using no resistor

### System constraints
Water Threshold: 700

A damp rag is placed on top of my water level sensor to simulate a water reservoir. The aforementioned threshold is about above the average reading I got during testing. When the rag is removed, the sensor levels drop below 700, and they rise when the damp rag is placed back onto it.

Temp/Humidity Threshold: 650

This threshold is also about above average of the readings I got in my room. In testing, it appears that the module's readings were inversely proportional to the temperature of the room, so when the reading went above the threshold, the system would enter an IDLE state. Conversely, when the reading went at or below the threshold, the system would enter the RUNNING state.

Power Constraints

As previously mentioned, the Arduino board itself is not capable of safely providing the power required by the motors in order to be operational. An external power source is required to supply power to the motors and I initially used the 9V battery included in the kit, however the battery drained very quickly, so it is advised to use the wall adapter to supply the power source module.

States
The following subsections describe how the state machine is used to dictate the actions of the swamp cooler, and determine how a user can and cannot interact with the system.

ALL STATES
1.) The real time clock (RTC) must be used to report the transitioning of one state to another. Additionally, a report must be generated and outputted to the serial monitor. This report includes the date and time of the state change, and from what state to the next state the system had transitioned to.

IDLE
1.) Green LED ON, the rest are OFF
2.) Allow potentiometer to change the position of the stepper motor.
3.) Monitor the temp/humidity and the water level, and display that information to the LCD. If the water is too low, transition to the ERROR state. 
4.) Fan is OFF
RUNNING
1.) Blue LED ON, the rest are OFF
2.) Allow potentiometer to change the position of the stepper motor
3.) Fan is ON
4.) Monitor the temp/humidity and the water level, and display those readings to the LCD. If the temp/humidity reading goes above the threshold, the system should transition to IDLE. If the water level goes below the threshold, the system should transition to the ERROR state
5.) If the stop button is pressed, the system should transition to the DISABLED state
DISABLED
1.) Yellow LED ON, the rest are OFF
2.) Do NOT monitor temp/humidity and water levels. LCD should be cleared
3.) Start button monitored by ISR. If pressed, enter the RUNNING state
4.) Do NOT allow the vent/stepper motor to change position
5.) Fan is OFF
ERROR
1.) Red LED ON, the rest are OFF
2.) Do NOT display the temp/humidity or the water level. Instead, display a Water Level LOW message.
3.) Pressing the reset button should cause the system to transition to the IDLE state. If the water level is not sufficient, the IDLE state will return to ERROR
4.) Fan is OFF

### Video demonstration
https://drive.google.com/file/d/18-a7dweWBv35Zxsdg4fr2F8MQ52iMcrx/view?usp=drive_link