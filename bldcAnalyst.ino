/*    Electrical Vehicle BLDC motor Analyst
 * 
 * Analyst to monitor performance of bldc motor and battery pack. As well as to gather 
 * performance data to analyze autonomy and range in various situations.
 * Folllowing hardware is used to perform the analysis:
 *    - Arduino Nano
 *    - D54 IR obstruction avoidance sensor, to be replaced with the hall sensor output
 *    - ST7735 full color SPI display 128 x 160, to be replaced with a 160 x 320 screen
 *    - ACS724 ± 50A current sensor
 *    - ADS1115 I2C voltage sensor
 *    - DS18B20 one wire temp sensor for battery
 *    - Piezo buzzer 5V for alarms
 *    - White LED to show motor is TURNING
 *    - DS1302 RTC module to keep track of time
 *    
 *     ________________________________________
 *    |     |                                  |
 *    | VCC | GND  CS   RST  A0  SDA  SCK  LED | PINOUT ST7735
 *    |_____|__________________________________|
 * 
 *    Ard     D54     ST7735    ADS1115    ACS724    DS18b20   LED    Piezo   Button    DS1302
 *     5V     VCC       VCC       VCC       VCC        VCC
 *    GND     GND       GND       GND       GND        GND
 *     D2     Output
 *     D3                                                                         +
 *     D4                                              Data
 *     D5                                                       +
 *     D6                                                               +
 *     D7              SD/CS
 *     D8               RST
 *     D9              A0/DC
 *    D10               CS
 *    D11           SDA SD/MOSI
 *    D12             SD/MISO
 *    D13            SCK SD/SCK
 *     A0                                   OUT
 *     A1                                                                                CLK
 *     A2                                                                                 IO
 *     A3                                                                                 CE
 *     A4                         SDA
 *     A5                         SCL
 *     5V               LED
 * 
 * 
 * ATTENTION!! TODO:
 *      switch to HALL pulses instead of IR avoidance sensor D54
 *      swap Arduino Nano for ESP32 board
 *      ditch the DS1302 RTC and get a DS3132, DS1302 has too much drift
 *      incorporate the data logging
 *      set timestamp for data collection
 *      add GPS tracking and geo fencing
 *      migrate code to a 2.4" tft screen
 * 
 * 
 * Sketch uses     29300 bytes = 95% of program memory
 * Global variables 1686 bytes = 82% of dynamic memory
 * 
 * RdB, January 2021
*/

#define outputToSerial 0                    // for debugging, 0 = off, 1 = on

#include <TFT_ST7735.h>
#include "fonts/minipixel.h"                // 7x10 pixels
#include <OneWire.h>
#include <DS1302.h>
#include <DS18B20.h>                        // https://github.com/RobTillaart/DS18B20_RT small footprint
#include "ADS1X15.h"                        // https://github.com/RobTillaart/ADS1X15/blob/master/ADS1X15.h

#define sdCS    7
#define RST     8
#define DC      9
#define CS     10
#define rtcCLK 15
#define rtcIO  16
#define rtcCE  17

#define tft_BLUE      0x001F
#define tft_RED       0xF800
#define tft_ORANGE    0xFDCE
#define tft_GREEN     0x07E0
#define tft_CYAN      0x07FF
#define tft_MAGENTA   0xF81F
#define tft_YELLOW    0xFFE0
#define tft_GREY      0x5AEB
#define tft_WHITE     0xFFFF
#define tft_BLACK     0x0000

TFT_ST7735 tft = TFT_ST7735( CS, DC, RST);
ADS1115 ads( 0x48);
DS1302 rtc( rtcCE, rtcIO, rtcCLK);          // init the DS1302
Time t;                                     // init a time-data structure

const byte ampsPin = A0;                    // set pin to input for ampere value
const byte tachoPin = 2;                    // set pin to input pulse from tacho
const byte switchPin = 3;                   // set pin to input from button
const byte tempPin = 4;                     // set pin to input temperature sensor
const byte ledPin = 5;                      // set pin to output to a LED
const byte piezoPin = 6;                    // set pin to output piezo
const byte yBat = 86;                       // set start of battery bar graph meter
const byte yDateTime = 20;                  // start y-axes date and time data

OneWire oneWire( tempPin);
DS18B20 tmp( &oneWire);


// ----- generic variables
int wheelCirc = 1756;                       // 1756 mm for a 20" rim with 4" fat tire - 2075 mm 24" rim and 4.0" fat tire
float refVolt = 4.798;                      // real voltage on the arduino measured with DVM
unsigned long prevMillis = 0;               // used for new "old" time
unsigned long interval = 1000;              // value to keep in line with the time


// ----- button variables
byte pageCount = 3;                         // start on first page
byte pageBuild = 0;                         // remember if page is active
byte switchState = HIGH;                    // because of internal pullup consider switch open
byte oldSwitchState = HIGH;                 // because of internal pullup consider switch open
const unsigned long debounceTime = 15;      // milliseconds
unsigned long lastSwitchTime;               // last time switch state changed


// ----- rpm variables
bool standStill = 1;
double rpmDistance = 0.0;
double totSpeed = 0.0;
float avgSpeed;                             // average speed trip
float maxSpeed;                             // max speed trip
float rpmFloat;                             // calculated value for rpm
float rpmSpeed;                             // calculated speed based on rpm an wheel cirumference
float totDistance;                          // total trip distance
unsigned int rpmValue;                      // 
volatile unsigned long rpmTime;


// ----- time in use variables
byte firstRun = 0;
byte prevSec;
int rtHours;
int rtMinutes;
int rtSeconds;
float decimalMinutes;
float decimalSeconds;
float decimalRuntime;
unsigned long runtimeSeconds;               // used to calculate decimale portion of time
unsigned long runtimeTotal;                 // runtime  variable


// ----- piezo alarm
byte alarmState = 0;                        // silence the alarm
unsigned long stateTime = 0;
unsigned long workingInterval = 0;


// ----- voltage variables
int16_t rawBatVoltage;                      // value from voltage sensor always 16bit
float curBatVoltage;                        // current battery voltage reading
float maxBatVoltage = 54.6;                 // max allowed voltage on battery pack
float minBatVoltage = 39.0;                 // min allowed voltage on battery pack
float batPercentage;                        // keep track of useable battery voltage
float bv;                                   // battery voltage helper


// ----- current variables
int acsAmpScale = 40;                       // mV per ampere
int rawAmpReading;                          // reading from the sensor
int rawAvgAmpReading;                       // average out ten readings
int zeroAmpVolt = 2150;                     // zero current value reading with DVM
float ampsVolt;                             // converted ADC values to millivolts
float ampHours = 0.0;                       // ampere hour calculation
float avgAmpere;                            // average ampere reading
float curAmpere;                            // current reading
float maxAmpere;                            // maximum reading
float totAmpere;                            // to calculate average ampere
float evEfficiency;                         // Km's per Ah


// ----- battery temperature variables
float batTemperature;                       // battery temperature
float minBatTemp  = 5.0;                    // min temp for battery pack for discharging
float maxBatTemp = 50.0;                    // max temp battery pack for discharging


// ----- power variables
int avgWattHours;
int curWattHours;
int maxWattHours;


// ----- message is SOS
int sosMessage[] = {1,1,1,3,2,2,2,3,1,1,1,4}; // dot = 1, dash = 2, inter letter space = 3, inter word space = 4
int messageLength = 12; // length of the dots, dashes, and spaces info
int messagePointer = 0;
unsigned long morseInterval = 80;             // mS per morse unit time


void setup() {
  #if outputToSerial == 1                     // for debugging
    Serial.begin( 115200);
  #endif

  tft.begin();
  tft.setRotation( 1);
  tft.setFont(&minipixel);
  tft.fillRect( 0, 0, 160, 128, tft_BLACK);
  tft.fillRect( 0, 0, 160,  16, tft_ORANGE);
  tft.setTextColor( tft_CYAN);
  tft.setCursor(  14, 2); tft.print( "C");
  tft.setCursor(  34, 2); tft.print( "A");
  tft.setCursor(  54, 2); tft.print( "N");
  tft.setCursor(  74, 2); tft.print( "E");
  tft.setCursor(  94, 2); tft.print( "G");
  tft.setCursor( 114, 2); tft.print( "O");
  tft.setCursor( 134, 2); tft.print("24\"");
  tft.fillRect( 0, yDateTime - 4, 160, 14, tft_WHITE);

  tmp.begin();
  tmp.setResolution( 11);
  
  TCCR1A = 0;                               // set entire Timer/Counter Control Register to 0
  TCCR1B = 0;                               // set entire Timer/Counter Control Register to 0
  TCCR1B |= ( 1 << CS12);                   // prescaler 256 to slow down frequency 16MHz/256
  TIMSK1 |= ( 1 << TOIE1);                  // enable timer overflow

  pinMode( ledPin, OUTPUT);                 // assign pin to output
  pinMode( switchPin, INPUT_PULLUP);        // assign pin to input
  pinMode( tachoPin, INPUT);                // assign pin to input
  pinMode( tempPin, INPUT);                 // assign pin to input
  pinMode( piezoPin, OUTPUT);               // assign pin to output

  for( int j = 0; j < 3; j++){
    digitalWrite( piezoPin, HIGH);
    delay(100);
    digitalWrite( piezoPin, LOW);
    delay(30);   
  }

  ads.begin();                              // initialize ADS1115
  ads.setGain(1);                           // 4.096 volt max scale

  rtc.halt(false);                          // Set the clock to run-mode
  rtc.writeProtect(false);                  // disable the write protection

// ----- the following lines can be commented out
// ----- to use the values already stored in the DS1302
//  rtc.setDOW( FRIDAY);        // Set Day-of-Week to FRIDAY
//  rtc.setTime( 12, 0, 0);     // Set the time to 12:00:00 (24hr format)
//  rtc.setDate( 6, 8, 2010);   // Set the date to August 6th, 2010

  attachInterrupt( digitalPinToInterrupt( tachoPin), tachoISR, FALLING);
}


ISR(TIMER1_OVF_vect){ standStill = 1;}


void loop() {

  t = rtc.getTime();
  piezoAlarm();

  switchState = digitalRead( switchPin);
  if( switchState != oldSwitchState){
    if( millis() - lastSwitchTime >= debounceTime){
      lastSwitchTime = millis();            // when the switch is closed
      oldSwitchState = switchState;         // remember for next time
      if( switchState == LOW){
        pageCount ++;
        pageBuild = 0;
        if( pageCount > 4) pageCount = 1;
      }
    }
  }

  if( prevSec != t.sec){
    rpmCalc();
    runningTime();
    getBatTemperature();
    getAmpReading();
    getVoltReading();
    getPowerReading();
    infoToDisplay();
    //dataLogging();  // future wish, using a 24LF256 eeprom or SD card
  }
} // end of loop


void piezoAlarm(){
  if( alarmState == 1){
    if( millis() - stateTime > workingInterval) alarmMessage();
  }
}

void tachoISR() {
  rpmTime = TCNT1;
  TCNT1 = 0;
  standStill = 0;
}

void rpmCalc(){                                                 // calculations in relation with the rpm and runtime
  if ( standStill == 1) {
    rpmSpeed = 0;
  } else {
    noInterrupts();
      unsigned long copyRpmTime = rpmTime;
    interrupts();
    rpmFloat = 120.0 / ((float) copyRpmTime / 31250.0);
    rpmValue = round( rpmFloat);
    rpmSpeed = rpmFloat * wheelCirc * 0.00006;
    rpmDistance = rpmFloat / 60 * wheelCirc * 0.000001;
    tone( ledPin, 100, 500);
  }
  totSpeed += rpmSpeed;
  avgSpeed = totSpeed / runtimeTotal;
  if( runtimeTotal == 0) avgSpeed = 0.0;                        // to avoid NaN and INF error on screen
  if( standStill != 1) totDistance += rpmDistance;
  if( maxSpeed < rpmSpeed) maxSpeed = rpmSpeed;
}


void runningTime(){ // *****  WHEN TO RESET RUNTIME??
  if( standStill == 0) runtimeTotal ++;
  runtimeSeconds = runtimeTotal;
  rtSeconds = runtimeSeconds %60; runtimeSeconds = runtimeSeconds / 60; // modulo calculates the remainder and returns the seconds after devision
  rtMinutes = runtimeSeconds %60; runtimeSeconds = runtimeSeconds / 60;
  rtHours   = runtimeSeconds %24;
  decimalMinutes = rtMinutes / 60.0;
  decimalSeconds = rtSeconds / 3600.0;
  decimalRuntime = rtHours + decimalMinutes + decimalSeconds;
}


void getBatTemperature(){
  tmp.requestTemperatures();                                    // send the command to get temperature readings 
  batTemperature = tmp.getTempC();                              // get temperature of sensor in °C
  if( batTemperature < minBatTemp || batTemperature > maxBatTemp){
    alarmState = 1;                                             // if battery temperature depasses set values 
  } else {
    alarmState = 0;                                             // silence the alarm
  }
}

void getAmpReading(){
  for( int n = 0; n < 10; n ++){
    rawAmpReading = analogRead( ampsPin);                       // get reading from the sensor
    rawAmpReading += rawAmpReading;                             // total the ampere readings
    delay( 10);                                                 // TODO ditch the delay
    if( n == 9) rawAvgAmpReading = rawAmpReading / 10;          // average the ampere reading
  }
  ampsVolt = ( rawAvgAmpReading / 1024) * refVolt * 1000;       // convert ADC values to millivolts
  curAmpere = (( ampsVolt - zeroAmpVolt) / acsAmpScale);        // convert volt reading to ampere
  if( maxAmpere < curAmpere) maxAmpere = curAmpere;             // determine maximum amperage
  if( standStill != 1){
    totAmpere += curAmpere;
    ampHours += ( curAmpere / 3600);
  }
  avgAmpere = totAmpere / runtimeTotal;                         // get average amperage
  if( runtimeTotal == 0) avgAmpere = 0.0;                       // to avoid NaN and INF error on screen
  evEfficiency = totDistance / ampHours;                        // distance / ampere hour gives 1Ah for ..km's
  if( totDistance == 0 || ampHours == 0) evEfficiency = 0;
}

void getVoltReading(){
  rawBatVoltage = ads.readADC(0);
  curBatVoltage = ads.toVoltage( rawBatVoltage) * 15; // * 15 = TESTVALUE;
  batPercentage = ( maxBatVoltage - curBatVoltage) / ( maxBatVoltage - minBatVoltage) * 100;
  bv = curBatVoltage;
  bv = map( bv * 10, minBatVoltage * 10, maxBatVoltage * 10, 1, 73);        // bv * 10 -> the map() function uses integer math
}

void getPowerReading(){
  float totWattHours;
  curWattHours = curBatVoltage * curAmpere;                     // get the power drain
  if( standStill != 1) totWattHours += curWattHours;            // only add to total when running
  avgWattHours = totWattHours / runtimeTotal;                   // calculate the average power
  if( maxWattHours < curWattHours) maxWattHours = curWattHours; // determine if we have to change max power
}

void alarmMessage(){                                           // set up time to next call of this function
  if( sosMessage[ messagePointer] == 1) workingInterval = morseInterval * 2;
  if( sosMessage[ messagePointer] == 2) workingInterval = morseInterval * 4;
  if( sosMessage[ messagePointer] == 3) workingInterval = morseInterval;
  if( sosMessage[ messagePointer] == 4) workingInterval = morseInterval * 16;
  stateTime = millis();
  // toggle output
  if( sosMessage[messagePointer] == 1 || sosMessage[messagePointer] == 2){
    if( digitalRead( piezoPin) == LOW) {
      digitalWrite( piezoPin, HIGH);
    } else {
      digitalWrite( piezoPin, LOW);
      updatePointer();
    } 
  } else {
    updatePointer();
  }
}

void updatePointer(){
  messagePointer ++;
  if( messagePointer >= messageLength){                         // end of message
    messagePointer = 0;
  }
}

void infoToDisplay(){
  switch( pageCount){
    case 1:
      if( pageBuild == 0){
        tft.fillRect( 0, 17, 160, 111, tft_BLACK);
        tft.setTextColor( tft_WHITE);
        tft.setCursor( tft.width()/2, yDateTime + 20, true); tft.print( "BATTERY");
        tft.setTextColor( tft_CYAN);
        tft.setCursor(  14, yDateTime + 28); tft.print( "cap");
        tft.setCursor( tft.width()/2, yDateTime + 35, true); tft.print( "temp");
        tft.setCursor( 120, yDateTime + 28); tft.print( "volt");
        tft.setTextColor( tft_WHITE, tft_BLACK);

        tft.setCursor( 46, yDateTime + 61); tft.print( "SPEED");
        tft.setTextColor( tft_BLACK); tft.print( "_");
        tft.setTextColor( tft_WHITE); tft.print( "Km/h");
        tft.setTextColor( tft_CYAN);
        tft.setCursor(  14, yDateTime + 76); tft.print( "avg");
        tft.setCursor( tft.width()/2, yDateTime + 83, true); tft.print( "cur");
        tft.setCursor( 122, yDateTime + 76); tft.print( "max");
        pageBuild = 1;
      }

      dateTime();
      
      tft.fillRect(   0, yDateTime + 44, 160, 12, tft_BLACK);
      tft.setTextColor( tft_YELLOW);
      tft.setCursor(  12, yDateTime + 44); tft.print( ampHours, 1);
      tft.setCursor( tft.width()/2 - 6, yDateTime + 49, true); tft.print( batTemperature, 1);
      tft.setCursor( 120,  yDateTime + 44); tft.print( curBatVoltage, 1);

      tft.fillRect( 0, yDateTime + 92, 160, 12, tft_BLACK);
      tft.setCursor(  12, yDateTime + 92); tft.print( avgSpeed, 1);
      tft.setCursor( tft.width()/2 - 6, yDateTime + + 99, true); tft.print( rpmSpeed, 1);
      tft.setCursor( 122, yDateTime + 92); tft.print( maxSpeed, 1);
    break;

    case 2:
      if( pageBuild == 0){
        tft.fillRect( 0, 17, 160, 111, tft_BLACK);
        tft.setTextColor( tft_WHITE, tft_BLACK);  
        tft.setCursor(  48, yDateTime + 13); tft.print( "AMPERES");
        tft.setTextColor( tft_BLACK); tft.print( "_");
        tft.setTextColor( tft_WHITE); tft.print( "A");
        tft.setTextColor( tft_CYAN);
        tft.setCursor(  10, yDateTime + 28); tft.print( "avg");
        tft.setCursor(  48, yDateTime + 28); tft.print( "cur");
        tft.setCursor(  86, yDateTime + 28); tft.print( "max");
        tft.setCursor( 128, yDateTime + 28); tft.print( "Ah");
        tft.setTextColor( tft_WHITE, tft_BLACK);
        tft.setCursor( tft.width()/2,  yDateTime + 69, true); tft.print( "POWER");
//        tft.setTextColor( tft_BLACK); tft.print( "_");
//        tft.setTextColor( tft_WHITE); tft.print( "Wh");
        tft.setTextColor( tft_CYAN);
        tft.setCursor(  14, yDateTime + 76); tft.print( "avg");
        tft.setCursor(  72, yDateTime + 76); tft.print( "Wh");
        tft.setCursor( 122, yDateTime + 76); tft.print( "max");
        pageBuild = 1;
      }

      dateTime();

      tft.fillRect(   0, yDateTime + 44, 160, 12, tft_BLACK);
      tft.setTextColor( tft_YELLOW);
      tft.setCursor(  10, yDateTime + 44); tft.print( avgAmpere, 1);
      tft.setCursor(  44, yDateTime + 44); tft.print( curAmpere, 1);
      tft.setCursor(  86, yDateTime + 44); tft.print( maxAmpere, 1);
      tft.setCursor( 120, yDateTime + 44); if( ampHours < 10) tft.print( ampHours, 2); else tft.print( ampHours, 1);

      tft.fillRect( 0, yDateTime + 92, 160, 12, tft_BLACK);
      tft.setTextColor( tft_YELLOW);
      tft.setCursor(  12, yDateTime + 92); tft.print( avgWattHours);
      tft.setCursor(  64, yDateTime + 92); tft.print( curWattHours);
      tft.setCursor( 122, yDateTime + 92); tft.print( maxWattHours);
    break;

    case 3:
      if( pageBuild == 0){
        tft.fillRect( 0, 17, 160, 111, tft_BLACK);
        tft.setTextColor( tft_WHITE, tft_BLACK);
        tft.setCursor( tft.width()/2, yDateTime + 20, true); tft.print( "OTHER");
        tft.setTextColor( tft_CYAN);
        tft.setCursor(  12, yDateTime + 28); tft.print( "runt");
        tft.setCursor( tft.width()/2, yDateTime + 36, true); tft.print( "dist");
        tft.setCursor( 125, yDateTime + 28); tft.print( "eff");

        tft.drawRect( 12, yDateTime + 64, 75,  20, tft_WHITE);        // battery
        tft.fillRect( 87, yDateTime + 71,  3,   6, tft_WHITE);        // plus pole
        tft.drawRect( 12, yDateTime + 84,  2,   6, tft_WHITE);
        tft.drawLine( 22, yDateTime + 84, 22, yDateTime + 86, tft_WHITE);
        tft.drawLine( 31, yDateTime + 84, 31, yDateTime + 88, tft_WHITE);
        tft.drawLine( 40, yDateTime + 84, 40, yDateTime + 86, tft_WHITE);
        tft.drawLine( 49, yDateTime + 84, 49, yDateTime + 89, tft_WHITE);
        tft.drawLine( 58, yDateTime + 84, 58, yDateTime + 86, tft_WHITE);
        tft.drawLine( 67, yDateTime + 84, 67, yDateTime + 88, tft_WHITE);
        tft.drawLine( 76, yDateTime + 84, 76, yDateTime + 86, tft_WHITE);
        tft.drawRect( 85, yDateTime + 84,  2, 4, tft_WHITE);

        tft.setFont( &internal);
        tft.setTextColor( tft_WHITE);
        tft.setCursor(  9, yDateTime + 93); tft.print(   "0");
        tft.setCursor( 43, yDateTime + 93); tft.print(  "50");
        tft.setCursor( 76, yDateTime + 93); tft.print( "100");
        tft.setFont( &minipixel);

        pageBuild = 1;
      }

      dateTime();

      tft.fillRect(   0, yDateTime + 44, 160, 12, tft_BLACK);
      tft.setTextColor( tft_YELLOW);
      if( decimalRuntime < 10){
        tft.setCursor(  12, yDateTime + 44); tft.print( decimalRuntime, 2);
      } else {
        tft.setCursor(  12, yDateTime + 44); tft.print( decimalRuntime, 1);
      }
      tft.setCursor( tft.width()/2, yDateTime + 52, true); tft.print( totDistance, 1);
      if( evEfficiency < 10){
        tft.setCursor( 122, yDateTime + 44); tft.print( evEfficiency, 2);
      } else {
        tft.setCursor( 122, yDateTime + 44); tft.print( evEfficiency, 1);
      }

      tft.fillRect( 105, yDateTime + 71, 55, 24, tft_BLACK);             // clear old values
      tft.setTextColor( tft_WHITE);
      tft.setCursor( 105, yDateTime + 67); tft.print( curBatVoltage, 1);
      tft.setCursor( 136, yDateTime + 67); tft.print( "V");
      tft.setCursor( 105, yDateTime + 82); tft.print( batPercentage, 0); // make this into a coulomb reading 1Ah = 3600C
      tft.setCursor( 136, yDateTime + 82); tft.print( "%");

      tft.fillRect( 13,  yDateTime + 65, 73,  18, tft_BLACK);
      if( batPercentage <= 100 && batPercentage >= 50) tft.fillRect( 13, yDateTime + 65, bv,  18, tft_GREEN);
      if( batPercentage < 50 && batPercentage >= 12.5) tft.fillRect( 13, yDateTime + 65, bv,  18, tft_YELLOW);
      if( batPercentage < 12.5 && batPercentage >= 0)  tft.fillRect( 13, yDateTime + 65, bv,  18, tft_RED);
    break;

    case 4:
      if( pageBuild == 0){
        tft.fillRect( 0, 17, 160, 111, tft_BLACK);
        tft.setTextColor( tft_WHITE, tft_BLACK);  
        tft.setCursor( tft.width()/2, 28, true); tft.print( "DIAGNOSTICS");
        tft.setTextColor( tft_CYAN);
        tft.setCursor(  14, 35); tft.print( "dC");
        tft.setCursor(  48, 35); tft.print( "col2");
        tft.setCursor(  88, 35); tft.print( "col3");
        tft.setCursor( 128, 35); tft.print( "col4");
        tft.setCursor(  17, 67); tft.print( "V");
        tft.setCursor(  50, 67); tft.print( "val2");
        tft.setCursor(  90, 67); tft.print( "val3");
        tft.setCursor( 130, 67); tft.print( "val4");
        tft.setCursor(  17, 99); tft.print( "%");
        tft.setCursor(  50, 99); tft.print( "inf2");
        tft.setCursor(  90, 99); tft.print( "inf3");
        tft.setCursor( 130, 99); tft.print( "inf3");
        pageBuild = 1;
      }

      tft.fillRect(   0, 51, 160, 12, tft_BLACK);
      tft.setTextColor( tft_YELLOW);
      tft.setCursor(   8,  51); tft.print( batTemperature, 1);
      tft.setCursor(  48,  51); tft.print( "0.00");
      tft.setCursor(  88,  51); tft.print( "0.00");
      tft.setCursor( 128,  51); tft.print( "0.00");
  
      tft.fillRect( 0, 83, 160, 12, tft_BLACK);
      tft.setTextColor( tft_YELLOW);
      tft.setCursor(   8,  83); tft.print( curBatVoltage, 1);
      tft.setCursor(  48,  83); tft.print( "0.00");
      tft.setCursor(  88,  83); tft.print( "0.00");
      tft.setCursor( 128,  83); tft.print( "0.00");

      tft.fillRect( 0, 115, 160, 12, tft_BLACK);
      tft.setTextColor( tft_YELLOW);
      tft.setCursor(   8, 115); tft.print( batPercentage, 1);
      tft.setCursor(  48, 115); tft.print( "0.00");
      tft.setCursor(  88, 115); tft.print( "0.00");
      tft.setCursor( 128, 115); tft.print( "0.00");
    break;

    default:
      tft.setTextColor( tft_YELLOW);
      tft.setCursor(  16,  55); tft.print( "Invalid");
      tft.setTextColor( tft_BLACK); tft.print( "_");
      tft.setTextColor( tft_YELLOW); tft.print( "switch");
      tft.setTextColor( tft_BLACK); tft.print( "_");
      tft.setTextColor( tft_YELLOW); tft.print( "state!");
    break;
  }
  prevSec = t.sec;
}

void dateTime(){
  tft.setCursor( 10, yDateTime);
  tft.setFont( &internal);
  tft.fillRect(   0, yDateTime - 4, 160, 14, tft_WHITE);
  tft.setTextColor( tft_GREY);
  printDigits(( t.date)); tft.print( "-");
  printDigits((  t.mon)); tft.print( "-");
  tft.print( t.year,DEC);
  tft.setCursor( 100, yDateTime);
  printDigits(( t.hour)); tft.print( ":");
  printDigits((  t.min)); tft.print( ":");
  printDigits((  t.sec));
  tft.setTextColor( tft_YELLOW);
  tft.setFont( &minipixel);
}

void printDigits(int digits){             // utility function prints leading 0
  if(digits < 10) tft.print('0');
  tft.print(digits);
}
