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
-9V battery
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

LCD

RTC

### System constraints