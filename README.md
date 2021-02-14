# bldcAnalyst
Brushless motor analyst for small electrical vehicles
Analyst to monitor performance of bldc motor and battery pack. As well as to gather 
performance data to analyze autonomy and range in various situations.
This repository is not yet finnished and needs to be ported to an ESP32 as I am running
out of memory on the Arduino Nano.

I have tried to add as much comments as possible for you to understand what is going on.

Folllowing hardware is used to perform the analysis:
   - Arduino Nano
   - D54 IR obstruction avoidance sensor, to be replaced with the hall sensor output
   - ST7735 full color SPI display 128 x 160, to be replaced with a 160 x 320 screen
   - ACS724 ± 50A current sensor
   - ADS1115 I2C voltage sensor
   - DS18B20 one wire temp sensor for battery
   - Piezo buzzer 5V for alarms
   - White LED to show motor is TURNING
   - DS1302 RTC module to keep track of time
   
    ________________________________________
   |     |                                  |
   | VCC | GND  CS   RST  A0  SDA  SCK  LED | PINOUT ST7735
   |_____|__________________________________|

   Ard     D54     ST7735    ADS1115    ACS724    DS18b20   LED    Piezo   Button    DS1302
    5V     VCC       VCC       VCC       VCC        VCC
   GND     GND       GND       GND       GND        GND
    D2     Output
    D3                                                                         +
    D4                                              Data
    D5                                                       +
    D6                                                               +
    D7              SD/CS
    D8               RST
    D9              A0/DC
   D10               CS
   D11           SDA SD/MOSI
   D12             SD/MISO
   D13            SCK SD/SCK
    A0                                   OUT
    A1                                                                                CLK
    A2                                                                                 IO
    A3                                                                                 CE
    A4                         SDA
    A5                         SCL
    5V               LED


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
