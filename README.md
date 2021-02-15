# bldc Analyst
Brushless motor analyst for small electrical vehicles
Analyst to monitor performance of bldc motor and battery pack. As well as to gather 
performance data to analyze autonomy and range in various situations.
This repository is not yet finnished and needs to be ported to an ESP32 as I am running
out of memory on the Arduino Nano.

I have tried to add as much comments in the code as possible, for you to understand what is going on.

Following hardware is used to perform the analysis:
   - Arduino Nano
   - D54 IR obstruction avoidance sensor, to be replaced with the hall sensor output
   - ST7735 full color SPI display 128 x 160, to be replaced with a 160 x 320 screen
   - ACS724 ± 50A current sensor
   - ADS1115 I2C voltage sensor
   - DS18B20 one wire temp sensor for battery
   - Piezo buzzer 5V for alarms
   - White LED to show motor is TURNING
   - DS1302 RTC module to keep track of time

ATTENTION!! TODO:
     switch to HALL pulses instead of IR avoidance sensor D54
     swap Arduino Nano for ESP32 board
     ditch the DS1302 RTC and get a DS3132, DS1302 has too much drift
     incorporate the data logging
     set timestamp for data collection
     add GPS tracking and geo fencing
     migrate code to a 2.4" tft screen


Sketch uses     29300 bytes = 95% of program memory
Global variables 1686 bytes = 82% of dynamic memory
