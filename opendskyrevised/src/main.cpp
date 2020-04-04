#include <Arduino.h>

/*
MIT License
Copyright (c) 2019 Ron Diamond
Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:
The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.
THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/


#include <Adafruit_NeoPixel.h>
#define PIN            6
#define NUMPIXELS      18
Adafruit_NeoPixel neoPixels = Adafruit_NeoPixel(NUMPIXELS, PIN, NEO_GRB + NEO_KHZ800);

#include "LedControl.h"
LedControl ledControl = LedControl(12,10,11,4);

#include "RTClib.h"
RTC_DS1307 realTimeClock;

#include<Wire.h>
const int MPU_addr=0x69;  // I2C address of the MPU-6050

// Timer funktion
#include <timer.h>
auto timer = timer_create_default();

#include <TinyGPS++.h>
TinyGPSPlus gps;

#include "SoftwareSerial.h"

#include "DFRobotDFPlayerMini.h"

SoftwareSerial mySoftwareSerial(5, 4); // RX, TX
DFRobotDFPlayerMini myDFPlayer;

/* Files on SD Card (have to be copied to the sdcard in exactly this order)
Can be found here:
https://www.nasa.gov/connect/sounds/index.html

0001_Silence.mp3
0002_Alarm.mp3
0003_JFK_I_Believe_Short.mp3
0004_JFK_We_Choose_Short.mp3
0005_A8_Genesis_Short.mp3
0006_A11_Eagle_Has_Landed_Short.mp3
0007_A13_problem_Short.mp3
0008_A11_Go_for_Landing.mp3
0009_A11_Launch_to_Orbit.mp3
0010_A12_Conrad_Bugs_Bunny.mp3
0011_A11_One_Small_Step.mp3
0012_A17_Strolling_Long.mp3
0013_JFK_I_Believe_Long.mp3
0014_JFK_We_Choose_Long.mp3
0015_A8_Genesis_Long.mp3
0016_A17_Orange_Soil_Short.mp3
0017_A13_problem_Long.mp3
0018_Quindar_Beeps.mp3
0019_Apollo_12_Cardiac_Sim.mp3
0020_Apollo_12_All_Weather_Testing.mp3
0021_Quindar_Beeps-trimmed.mp3

*/

enum Action: int {
    none                        = 0,
    displayIMUAttitude          = 1,
    displayRealTimeClock        = 2,
    displayGPS                  = 3,
    displayRangeWith1202Error   = 4,
    setTime                     = 5,
    setDate                     = 6,
    PlayAudioclip               = 7,
    PlaySelectedAudioclip       = 8,
    displayIMUGyro              = 9
};

enum Mode: int {
    modeIdle                = 0,
    modeInputVerb           = 1,
    modeInputNoun           = 2,
    modeInputProgram        = 3,
    modeLampTest            = 4
};

enum programNumber: int {
    programNone             = 0,
    programJFKAudio         = 62,
    programApollo11Audio    = 69,
    programApollo13Audio    = 70
};

enum lampNumber: int {
    lampNoun                = 0,
    lampProg                = 1,
    lampVerb                = 2,
    lampCompActy            = 3,
    lampTemp                = 4,
    lampGimbalLock          = 5,
    lampProgCond            = 6,
    lampRestart             = 7,
    lampTracker             = 8,
    lampAlt                 = 9,
    lampVel                 = 10,
    lampClk                 = 11,
    lampPosition            = 12,
    lampOprErr              = 13,
    lampKeyRelease          = 14,
    lampSTBY                = 15,
    lampNoAtt               = 16,
    lampUplinkActy          = 17
    
};

enum color: int
{
    green                   = 1,
    white                   = 2,
    yellow                  = 3,
    orange                  = 4,
    blue                    = 5,
    red                     = 6,
    off                     = 7
};

enum keyValues: int
{ // symbolic references to individual keys
    keyNone                 = 20,
    keyVerb                 = 10,
    keyNoun                 = 11,
    keyPlus                 = 12,
    keyMinus                = 13,
    keyNumber0              = 0,
    keyNumber1              = 1,
    keyNumber2              = 2,
    keyNumber3              = 3,
    keyNumber4              = 4,
    keyNumber5              = 5,
    keyNumber6              = 6,
    keyNumber7              = 7,
    keyNumber8              = 8,
    keyNumber9              = 9,
    keyClear                = 18,
    keyProceed              = 14,
    keyRelease              = 16,
    keyEnter                = 15,
    keyReset                = 17
};

enum verbValues: int
{ // Verbs 0,35,16,21
    verbNone                = 0,
    verbLampTest            = 35,
    verbDisplayDecimal      = 16,
    verbSetComponent        = 21
};

enum nounValues: int 
{ // Nouns 0,17,36,37,43,68,98
    nounNone                = 0,
    nounIMUAttitude         = 17,
    nounIMUgyro             = 18,
    nounClockTime           = 36,
    nounDate                = 37,
    nounLatLongAltitude     = 43,
    nounRangeTgoVelocity    = 68,
    nounSelectAudioclip      = 98
};

enum registerDisplayPositions: int 
{ // Display Register Positions
    register1Position       = 4,
    register2Position       = 5,
    register3Position       = 6
};
enum dregister: int
{
     register1              = 1,
     register2              = 2,
     register3              = 3
};

enum imumode: int
{ // imumode Gyro or Accelration
    Gyro                    = 1,
    Accel                   = 0
};

enum inputnumsign: int
{ // imumode Gyro or Accelration
    plus                    = 1,
    minus                   = 0
};


long valueForDisplay[7];
byte digitValue[7][7];
byte inputnum[5];
int inputnumsign = plus;
byte keyValue = keyNone;
byte oldKey = none;
bool fresh = true;
byte action = none;
byte currentAction = none;
byte verb = verbNone;
byte verb_old = verbNone;
byte verb_old2 = verbNone;
bool verb_error = false;
byte verbNew[2];
byte verbOld[2];
byte noun = nounNone;
bool noun_error = false;
byte noun_old = nounNone;
byte noun_old2 = nounNone;
byte nounNew[2];
byte nounOld[2];
byte currentProgram = programNone;
byte prog = 0;
byte prog_old = 0;
byte prog_old2 = 0;
byte progNew[2];
byte progOld[2];
bool newProg = false;
byte count = 0;
byte mode = modeIdle;
byte oldMode = modeIdle;
bool toggle = false;
bool toggle600 = false;
bool toggle250 = false;
bool toggled250 = false;
byte toggle600count = 0;
byte toggleCount = 0;
bool error = 0;
bool newAction = false;
byte audioTrack = 1;
bool blink = false;
bool blinkverb = true;
bool blinknoun = true;
bool blinkprog = true;
bool imutoggle = true;
bool printregtoggle = true;
bool uplink_compact_toggle = true;
unsigned long blink_previousMillis = 0; 
const long blink_interval = 600;
int clipnum = 1;
int clipcount = 0;

int lat = 0;
int lon = 0;
int alt = 0;

// IMU https://github.com/griegerc/arduino-gy521/blob/master/gy521-read-angle/gy521-read-angle.ino
const int ACCEL_OFFSET   = 200;
const int GYRO_OFFSET    = 151;  // 151
const int GYRO_SENSITITY = 131;  // 131 is sensivity of gyro from data sheet
const float GYRO_SCALE   = 0.2; //  0.02 by default - tweak as required
const float GYRO_TEMP_DRIFT   = 0.02; //  0.02 by default - tweak as required
const int GYRO_GRANGE = 2; // Gforce Range
const int ACCEL_SCALE = 16384; // Scalefactor of Accelerometer
const float LOOP_TIME    = 0.15; // 0.1 = 100ms
const int GYRO_OFFSET_X = 2; // change this to your system until gyroCorrX displays 0 if the DSKY sits still
const int GYRO_OFFSET_Y = 0; // change this to your system until gyroCorrY displays 0 if the DSKY sits still
const int GYRO_OFFSET_Z = 0; // change this to your system until gyroCorrZ displays 0 if the DSKY sits still
const int ACC_OFFSET_X = 2; // change this to your system until accAngleX displays 0 if the DSKY sits still
const int ACC_OFFSET_Y = 3; // change this to your system until accAngleY displays 0 if the DSKY sits still
const int ACC_OFFSET_Z = 0;  // change this to your system until accAngleZ displays 0 if the DSKY sits still

int globaltimer=0;
bool global_state_1sec=false;
bool global_state_600msec=false;

// GPS Definitions
bool GPS_READ_STARTED = true;
bool gpsread = true;
bool gpsfix = false;

// variables for Time display (10th of a second, 100th of a second)
unsigned long previousMillis = 0;
int oldSecond = 0;

// 1sec toogle
bool toggle_timer(void *)
{
  if(global_state_1sec==false){
    global_state_1sec=true;
    toggle = true;
  }
  else
  {
    global_state_1sec=false;
    toggle = false;
  }
  return true; // repeat? true
}

// 600msec toggle
bool toggle_timer_600(void *)
{
  if(global_state_600msec==false){
    global_state_600msec=true;
    toggle600 = true;
  }
  else
  {
    global_state_600msec=false;
    toggle600 = false;
  }
  return true; // repeat? true
}

bool toggle_timer_250(void *) {
  toggle250 = !toggle250;
  return true; // repeat? true
}

void validateAction()
{
    if (verb == verbLampTest) {
        mode = modeLampTest;
        //noun = noun_old;
        newAction = false;
    }
    else if ((verb == verbDisplayDecimal) && (noun == nounIMUAttitude)) {
        action = displayIMUAttitude;
        newAction = false;
    }
    else if ((verb == verbDisplayDecimal) && (noun == nounIMUgyro)) {
        action = displayIMUGyro;
        newAction = false;
    }
    else if ((verb == verbDisplayDecimal) && (noun == nounClockTime)) {
        action = displayRealTimeClock;
        newAction = false;
    }
    else if ((verb == verbDisplayDecimal) && (noun == nounLatLongAltitude)) {
        // Display current GPS
        action = displayGPS;
        newAction = false;
        count = 0;
    }
    else if ((verb == verbDisplayDecimal) && (noun == nounRangeTgoVelocity)) {
        // Display Range With 1202 ERROR
        action = displayRangeWith1202Error;
        newAction = false;
    }
    else if ((verb == verbSetComponent) && (noun == nounClockTime)) {
        action = setTime;
        newAction = false;
    }
    else if ((verb == verbSetComponent) && (noun == nounDate)) {
        action = setDate;
        newAction = false;
    }
    else if ((verb == verbSetComponent) && (noun == nounSelectAudioclip)) {
        action = PlayAudioclip;
        newAction = false;
    }
    else {
        // not (yet) a valid verb/noun combination
        action = none;
        newAction = false;
    }
}


void illuminateWithRGBAndLampNumber(byte r, byte g, byte b, int lamp) {
    neoPixels.setPixelColor(lamp, neoPixels.Color(r,g,b));
    neoPixels.show();   // show the updated pixel color on the hardware
}

void turnOffLampNumber(int lampNumber) {
    illuminateWithRGBAndLampNumber(0, 0, 0, lampNumber);
}

void setLamp(int color, int lampNumber)
{
    /*  green                   = 1,
        white                   = 2,
        yellow                  = 3,
        orange                  = 4,
        blue                    = 5,
        red                     = 6,
        off                     = 7
    */
    switch (color)
    {
        case green:
            // Statement(s)
            neoPixels.setPixelColor(lampNumber, neoPixels.Color(0,100,0));
            neoPixels.show();   // show the updated pixel color on the hardware
            break;
        case white:
            neoPixels.setPixelColor(lampNumber, neoPixels.Color(100,100,100));
            neoPixels.show();   // show the updated pixel color on the hardware
            // Statement(s)
            break;
        case yellow:
            neoPixels.setPixelColor(lampNumber, neoPixels.Color(100,100,0));
            neoPixels.show();   // show the updated pixel color on the hardware
            // Statement(s)
            break;
        case orange:
            neoPixels.setPixelColor(lampNumber, neoPixels.Color(255,165,0));
            neoPixels.show();   // show the updated pixel color on the hardware
            // Statement(s)
            break;
        case blue:
            neoPixels.setPixelColor(lampNumber, neoPixels.Color(0,0,100));
            neoPixels.show();   // show the updated pixel color on the hardware
            // Statement(s)
            break;
        case red:
            neoPixels.setPixelColor(lampNumber, neoPixels.Color(100,0,0));
            neoPixels.show();   // show the updated pixel color on the hardware
            // Statement(s)
            break;
        case off:
            neoPixels.setPixelColor(lampNumber, neoPixels.Color(0,0,0));
            neoPixels.show();   // show the updated pixel color on the hardware
            // Statement(s)
            break;
        default:
            // Statement(s)
            break; // Wird nicht benötigt, wenn Statement(s) vorhanden sind
    }
}

void setDigits()
{
    for (int indexa = 0; indexa < 8; indexa ++) {
        for (int index = 0; index < 7; index++) {
            digitValue[indexa][index] = 0;
        }
    }

    for (int indexa = 0; indexa < 7; indexa ++) {
        if (valueForDisplay[indexa] < 0) {
            valueForDisplay[indexa] = (valueForDisplay[indexa] - (valueForDisplay[indexa] + valueForDisplay[indexa]));
            digitValue[indexa][0] = 1;
        }
        else {
            digitValue[indexa][0] = 0;
        }
        for (int index = 0; valueForDisplay[indexa] >= 100000; valueForDisplay[indexa] = (valueForDisplay[indexa] - 100000)) {
            index++;
        }
        for (int index = 0; valueForDisplay[indexa] >= 10000; valueForDisplay[indexa] = (valueForDisplay[indexa] - 10000)) {
            index++;
            digitValue[indexa][1] = index;
        }
        for (int index = 0; valueForDisplay[indexa] >= 1000; valueForDisplay[indexa] = (valueForDisplay[indexa] - 1000)) {
            index++;
            digitValue[indexa][2] = index;
        }
        for (int index = 0; valueForDisplay[indexa] >= 100; valueForDisplay[indexa] = (valueForDisplay[indexa] - 100)) {
            index++;
            digitValue[indexa][3] = index;
        }
        for (int index = 0; valueForDisplay[indexa] >= 10; valueForDisplay[indexa] = (valueForDisplay[indexa] - 10)) {
            index++;
            digitValue[indexa][4] = index;
        }
        for (int index = 0; valueForDisplay[indexa] >= 1; valueForDisplay[indexa] = (valueForDisplay[indexa] - 1)) {
            index++;
            digitValue[indexa][5] = index;
        }
    }

    for (int index = 0; index < 3; index++) {
        // ledControl.clearDisplay(index+1);
        for (int i = 0; i < 6; i++) {
            if (i == 0) {
                if (digitValue[(index+4)][i] == 1) {
                    ledControl.setRow(index+1, i, B00100100);
                }
                else {
                    ledControl.setRow(index+1, i, B01110100);
                }
            }
            else {
                ledControl.setDigit(index+1, i, digitValue[index + 4][i], false);
            }
        }
    }
}
void clearRegister(int dregister)
{
    ledControl.clearDisplay(dregister);
    //ledControl.setRow(dregister,0,B00000000);
    //ledControl.setRow(dregister,1,B00000000);
    //ledControl.setRow(dregister,2,B00000000);
    //ledControl.setRow(dregister,3,B00000000);
    //ledControl.setRow(dregister,4,B00000000);
    //ledControl.setRow(dregister,5,B00000000);
}

void printRegister(int dregister, long number = 0, bool leadzero = true, bool blink = false, bool alarm = false)
{   // Print the Register 1, 2, or 3, the number, if you want leading zeros if you want to blink it, check if it is an alarm
    // Setdigit: Register 0 - 3, plus sign 0, 1-5 numbers
    //num4 = (fm_station / 10) % 10;
    //num3 = (fm_station / 100) % 10;
    //num2 = (fm_station / 1000) % 10;
    //num1 = (fm_station / 10000) % 10;
    int one = 0;
    int ten = 0;
    int hundred = 0;
    long thousand = 0;
    long tenthousand = 0;
    // first, check if the number is positive or negative and set the plus or minus sign
    if (number < 0)
    {
        number = -number;
        // Set the minus sign 
        ledControl.setRow(dregister, 0, B00100100);
    }
    else 
    {
        // Set the plus sign
        ledControl.setRow(dregister, 0, B01110100);
    }
    // now seperate the number
    if (number == 0)
    {
        one = int(number);
    }
    else if ((number > 0) && (number < 10))
    {
        one = int(number);
    }
    else if ((number >= 10) && (number < 100))
    {   
        one = number % 10;
        ten = (number - one) / 10;
    }
    else if ((number >= 100) && (number < 1000))
    {
        one = number % 10;
        ten = (number / 10) % 10;
        hundred = (number / 100) % 10;

    }
    else if ((number >= 1000) && (number < 10000))
    {
        one = number % 10;
        ten = (number / 10) % 10;
        hundred = (number / 100) % 10;
        thousand = (number / 1000) % 10;
    }
    else if ((number >= 10000) && (number < 100000))
    {
        one = number % 10;
        ten = (number / 10) % 10;
        hundred = (number / 100) % 10;
        thousand = (number / 1000) % 10;
        tenthousand = (number / 10000) % 10;
    }
    // show the number
    if (blink == false)
    {
        if (number >= 100000)
        {
            //ledControl.setRow(dregister,0,B00000000);
            //ledControl.setRow(dregister,1,B01001111);
            //ledControl.setRow(dregister,2,B00000000);
            //ledControl.setRow(dregister,3,B00000000);
            //ledControl.setRow(dregister,4,B00000000);
            //ledControl.setRow(dregister,5,B00000000);
            ledControl.setRow(dregister,0,B00000000);
            ledControl.setChar(dregister,1,' ',false);
            ledControl.setChar(dregister,2,'1', false);
            ledControl.setChar(dregister,3,'3', false);
            ledControl.setChar(dregister,4,'0', false);
            ledControl.setChar(dregister,5,'5', false);
        }
        else 
        {
            ledControl.setDigit(dregister, 5, one, false);
            ledControl.setDigit(dregister, 4, ten, false);
            ledControl.setDigit(dregister, 3, hundred, false);
            ledControl.setDigit(dregister, 2, thousand, false);
            ledControl.setDigit(dregister, 1, tenthousand, false);
        }
    }
    if (blink == true)
    {
        if ((toggle600 == true) && (printregtoggle == true))
        {   
            printregtoggle = false;
            ledControl.setDigit(dregister, 5, one, false);
            ledControl.setDigit(dregister, 4, ten, false);
            ledControl.setDigit(dregister, 3, hundred, false);
            ledControl.setDigit(dregister, 2, thousand, false);
            ledControl.setDigit(dregister, 1, tenthousand, false);
        }
        else if ((toggle600 == false) && (printregtoggle == false))
        {
            printregtoggle = true;
            ledControl.clearDisplay(dregister);
            //ledControl.setRow(dregister,0,B00000000);
            //ledControl.setRow(dregister,1,B00000000);
            //ledControl.setRow(dregister,2,B00000000);
            //ledControl.setRow(dregister,3,B00000000);
            //ledControl.setRow(dregister,4,B00000000);
            //ledControl.setRow(dregister,5,B00000000);
        }
    }
}

void printProg(int prog, bool blink = false)
{  // Print the Progam PROG
    int one = 0;
    int ten = 0;
    if (blink == false)
    {
        if (prog == 0)
        {
            ledControl.setRow(0,2,B00000000);
            ledControl.setRow(0,3,B00000000);
        }
        else if ((prog > 0) && (prog < 10))
        {
            ledControl.setDigit(0, 2, 0, false);
            ledControl.setDigit(0, 3, prog, false);
        }
        else if (prog >= 10)
        {   
            one = prog % 10;
            ten = (prog - one) / 10;
            ledControl.setDigit(0, 2, ten, false);
            ledControl.setDigit(0, 3, one, false);
        }
    }
    else if (blink == true)
    {
        ledControl.setRow(0,2,B00000000);
        ledControl.setRow(0,3,B00000000);
    }
}

void printVerb(int verb, bool blink = false)
{  // Print the verb VERB
    int one = 0;
    int ten = 0;
    if (blink == false)
    {
        if (verb == verbNone)
        {
            ledControl.setRow(0,0,B00000000);
            ledControl.setRow(0,1,B00000000);
        }
        else if ((verb > 0) && (verb < 10))
        {
            ledControl.setDigit(0, 0, 0, false);
            ledControl.setDigit(0, 1, verb, false);
        }
        else if (verb >= 10)
        {   
            one = verb % 10;
            ten = (verb - one) / 10;
            ledControl.setDigit(0, 0, ten, false);
            ledControl.setDigit(0, 1, one, false);
        }
    }
    else if (blink == true)
    {
        ledControl.setRow(0,0,B00000000);
        ledControl.setRow(0,1,B00000000);
    }
}

void printNoun(int noun, bool blink = false)
{  // Print the noun NOUN
    int one = 0;
    int ten = 0;
    if (blink == false)
    {
        if (noun == nounNone)
        {
            ledControl.setRow(0,4,B00000000);
            ledControl.setRow(0,5,B00000000);
        }
        else if ((noun > 0) && (noun < 10))
        {
            ledControl.setDigit(0, 4, 0, false);
            ledControl.setDigit(0, 5, noun, false);
        }
        else if (noun >= 10)
        {   
            one = noun % 10;
            ten = (noun - one) / 10;
            ledControl.setDigit(0, 4, ten, false);
            ledControl.setDigit(0, 5, one, false);
        }
    }
    else if (blink == true)
    {
        ledControl.setRow(0,4,B00000000);
        ledControl.setRow(0,5,B00000000);
    }
}


void setDigits(byte maximum, byte digit, byte value)
{//Serial.println("setDigits(byte ...)");
    ledControl.setDigit(maximum, digit, value, false);
}

void flasher()
{
    if (verb_error == true)
    {
        setLamp(orange, lampVerb);
    }
    if (noun_error == true)
    {
        setLamp(orange, lampNoun);
    }
    if (toggle == false) {
        setLamp(white,  lampOprErr);
    } else {
        setLamp(off, lampOprErr);
    }
}

int readKeyboard() {
    int oddRowDividerVoltage1 = 225;
    int oddRowDividerVoltage2 = 370;
    int oddRowDividerVoltage3 = 510;
    int oddRowDividerVoltage4 = 650;
    int oddRowDividerVoltage5 = 790;
    int oddRowDividerVoltage6 = 930;

    int evenRowDividerVoltage1 = 200;
    int evenRowDividerVoltage2 = 330;
    int evenRowDividerVoltage3 = 455;
    int evenRowDividerVoltage4 = 577;
    int evenRowDividerVoltage5 = 700;
    int evenRowDividerVoltage6 = 823;
    int evenRowDividerVoltage7 = 930;

    int value_row1 = analogRead(A0);
    int value_row2 = analogRead(A1);
    int value_row3 = analogRead(A2);
    if ((value_row1 > oddRowDividerVoltage6)
        && (value_row2 > oddRowDividerVoltage6)
        && (value_row3 > oddRowDividerVoltage6))
    {
        return keyNone;  // no key
    }

    // keyboard ~top row
    else if (value_row1 < oddRowDividerVoltage1) return keyVerb;
    else if (value_row1 < oddRowDividerVoltage2) return keyPlus;
    else if (value_row1 < oddRowDividerVoltage3) return keyNumber7;
    else if (value_row1 < oddRowDividerVoltage4) return keyNumber8;
    else if (value_row1 < oddRowDividerVoltage5) return keyNumber9;
    else if (value_row1 < oddRowDividerVoltage6) return keyClear;

    // keyboard ~middle row
    else if (value_row2 < evenRowDividerVoltage1) return keyNoun;
    else if (value_row2 < evenRowDividerVoltage2) return keyMinus;
    else if (value_row2 < evenRowDividerVoltage3) return keyNumber4;
    else if (value_row2 < evenRowDividerVoltage4) return keyNumber5;
    else if (value_row2 < evenRowDividerVoltage5) return keyNumber6;
    else if (value_row2 < evenRowDividerVoltage6) return keyProceed;
    else if (value_row2 < evenRowDividerVoltage7) return keyEnter;

    // keyboard ~bottom row
    else if (value_row3 < oddRowDividerVoltage1) return keyNumber0;
    else if (value_row3 < oddRowDividerVoltage2) return keyNumber1;
    else if (value_row3 < oddRowDividerVoltage3) return keyNumber2;
    else if (value_row3 < oddRowDividerVoltage4) return keyNumber3;
    else if (value_row3 < oddRowDividerVoltage5) return keyRelease;
    else if (value_row3 < oddRowDividerVoltage6) return keyReset;
    else {
        // no key
    }
}


void processIdleMode()
{
    if (keyValue != oldKey) {
        fresh = true;
        oldKey = keyValue;
    }
    if (fresh == true) {
        if (keyValue == keyVerb) {
            // verb
            mode = modeInputVerb;
            fresh = false;
            byte keeper = verb;
            for (int index = 0; keeper >= 10 ; keeper = (keeper - 10)) {
                index++;
                verbOld[0] = index;
            }
            for (int index = 0; keeper >= 1; keeper = (keeper - 1)) {
                index++;
                verbOld[1] = index;
            }
        }
        else if (keyValue == keyNoun) {
            // noun
            mode = modeInputNoun;
            fresh = false;
            byte keeper = noun;
            for (int index = 0; keeper >= 10; keeper = (keeper - 10)) {
                index++; nounOld[0] = index;
            }
            for (int index = 0;keeper >= 1; keeper = (keeper - 1)) {
                index++; nounOld[1] = index;
            }
        }
        else if (keyValue == keyProceed) {
            // program
            mode = modeInputProgram;
            fresh = false;
        }
        else if (keyValue == keyReset) {
            // resrt reeor
            error = 0;
            turnOffLampNumber(13);
            fresh = false;
        }
    }
}

void executeIdleMode()
{   // no action set just reading the kb
    if (newAction == true) {
        validateAction();
    }
    else {
        if (error == 1) {
            flasher();
        }
        keyValue = readKeyboard();
        processIdleMode();
    }
}

void toggleKeyReleaseLamp()
{
    if (toggle == false) {
        setLamp(white, lampKeyRelease);
    }
    else {
        setLamp(off, lampKeyRelease);
    }
}

void processVerbInputMode()
{
    if (keyValue == oldKey) {
        fresh = false;
    }
    else {
        fresh = true;
        oldKey = keyValue;
        if ((error == 1) && (keyValue == keyReset) && (fresh == true))
        {
            error = 0; 
            verb_error = false;
            //turnOffLampNumber(lampOprErr);
            setLamp(green, lampVerb);
            ledControl.setRow(0,0,0);
            ledControl.setRow(0,1,0);
            //verb = ((verbOld[0] * 10) + verbOld[1]);
            fresh = false;
        } //resrt reeor
        if ((keyValue == keyEnter) && (fresh == true)) {
            fresh = false;
            //das vorherige verb in verb_old speichern, mann weiss ja nie
            verb_old2 = verb_old;
            verb_old = verb;
            verb = ((verbNew[0] * 10) + (verbNew[1]));
            if (verb != verb_old)
            {
                // es wurde ein neues Verb eingegeben, daher muss noun auf 0 gesetzt werden
                noun_old2 = noun_old;
                noun_old = noun;
                noun = 0;
                printNoun(noun);
            }
            // wenn das neue Verb ein anderes als das neue Verb is, dann muss noun auf 0 gesetzt werden

            if ((verb != verbDisplayDecimal)
                && (verb != verbSetComponent)
                && (verb != verbLampTest)
                && (verb != verbNone)) {
                error = 1;
                verb_error = true;
                verb = ((verbOld[0] * 10) + verbOld[1]);    // restore prior verb
                setLamp(green, lampVerb);
            }
            else {
                turnOffLampNumber(lampOprErr);
                turnOffLampNumber(lampKeyRelease);
                //turnOffLampNumber(lampVerb);
                setLamp(green, lampVerb);
                mode = modeIdle;
                count = 0;
                fresh = false;
                error = 0;
                verb_error = false;
                newAction = true;
            }
        }

        if (fresh == true) {
            if (keyValue == keyRelease) {
                mode = oldMode;
                turnOffLampNumber(lampKeyRelease);
                //turnOffLampNumber(lampVerb);
                setLamp(green, lampVerb);
                count = 0;
                fresh = false;
                if (verb == verbNone) {
                    ledControl.setRow(0,0,0);
                    ledControl.setRow(0,1,0);
                }
                else {
                    setDigits(0, 0, verbOld[0]);
                    setDigits(0, 1, verbOld[1]);
                }
            }
            else if (keyValue == keyNoun) {
                mode = modeInputNoun;
                //turnOffLampNumber(lampVerb);
                setLamp(green, lampVerb);
                count = 0;
                fresh = false;
            }
            else if (keyValue == keyProceed) {
                //program
                mode = modeInputProgram;
                //turnOffLampNumber(lampVerb);
                setLamp(green, lampVerb);
                count = 0;
                fresh = false;
            }

        }

        if ((keyValue <= keyNumber9) && (count < 2)) {
            verbNew[count] = keyValue;
            setDigits(0, count, keyValue);
            count++;
            fresh = false;
        }
    }
}

void executeVerbInputMode()
{
    // inputting the verb
    setLamp(yellow, lampVerb);
    toggleKeyReleaseLamp();
    if (error == 1) {
        flasher();
    }
    keyValue = readKeyboard();
    processVerbInputMode();
}

void processNounInputMode()
{
    if (keyValue == oldKey) {
        fresh = false;
    }
    else {
        fresh = true;
        oldKey = keyValue;
        if ((error == 1) && (keyValue == keyReset) && (fresh == true)) {
            error = 0;
            noun_error = false;
            setLamp(green, lampNoun);
            setLamp(off,lampOprErr);
            fresh = false;
        } //resrt reeor

        if ((keyValue == keyEnter) && (fresh == true)) {
            fresh = false;
            noun_old2 = noun_old;
            noun_old = noun;
            noun = ((nounNew[0] * 10) + (nounNew[1]));
            fresh = false;
            if ((noun != nounIMUAttitude)
                && (noun != nounIMUgyro)
                && (noun != nounClockTime)
                && (noun != nounLatLongAltitude)
                && (noun != nounRangeTgoVelocity)
                && (noun != nounSelectAudioclip)
                && (noun != nounNone)) {
                noun = ((nounOld[0] * 10) + nounOld[1]);    // restore prior noun
                error = 1;
                noun_error = true;
                setLamp(green, lampNoun);
            }
            else {
                turnOffLampNumber(lampOprErr);
                turnOffLampNumber(lampKeyRelease);
                setLamp(green, lampNoun);
                mode = modeIdle;
                count = 0;
                fresh = false;
                error = 0;
                noun_error = false;
                newAction = true;
            }
        }

        if ((keyValue == keyRelease) && (fresh == true)) {
            mode = oldMode;
            turnOffLampNumber(lampKeyRelease);
            setLamp(green, lampNoun);
            count = 0;
            fresh = false;
            if (noun == 0) {
                //verb
                printNoun(noun);
                //ledControl.setRow(0, 4, 0);
                //ledControl.setRow(0, 5, 0);
            }
            else {
                printNoun(noun);
                //setDigits(0, 4, nounOld[0]);
                //setDigits(0, 5, nounOld[1]);
            }
        }
        if ((keyValue == keyVerb) && (fresh == true)) {
            //verb
            mode = modeInputVerb;
            setLamp(green, lampNoun);
            count = 0;
            fresh = false;
        }
        if ((keyValue == keyProceed) && (fresh == true)) {
            mode = modeInputProgram;
            setLamp(green, lampNoun);
            count = 0;
            fresh = false;
            //program
        }
        if ((keyValue <= keyNumber9)
            && (count < 2)) {
            nounNew[count] = keyValue;
            setDigits(0, (count + 4), keyValue);
            count++;

        }
    }
}

void executeNounInputMode()
{ // inputting the noun
    //Serial.println("Begin exexuteNounInputMode");
    setLamp(yellow, lampNoun);
    toggleKeyReleaseLamp();
    if (error == 1) {
        flasher();
    }
    keyValue = readKeyboard();
    //Serial.println("End exexuteNounInputMode");
    processNounInputMode();
}


void processProgramInputMode()
{
    if (keyValue == oldKey) {
        fresh = false;
    }
    else
    {
        fresh = true;
        oldKey = keyValue;
        if ((error == 1) && (keyValue == keyClear) && (fresh == true))
        {
            error = 0;
            prog = 0;
            newProg = false;
            mode = modeInputProgram;
            count = 0;
            fresh = false;
            turnOffLampNumber(lampOprErr);
            printProg(prog);
        }
        if ((keyValue == keyEnter) && (fresh == true)) 
        {
            fresh = false;
            prog_old2 = prog_old;
            prog_old = currentProgram;
            currentProgram = ((progNew[0] * 10) + (progNew[1]));
            prog = currentProgram;
            fresh = false;
            if ((currentProgram != programNone) && (currentProgram != programJFKAudio) && (currentProgram != programApollo11Audio) && (currentProgram != programApollo13Audio))
            {
                currentProgram = ((progOld[0] * 10) + progOld[1]);    // restore prior noun
                prog = prog_old;
                error = 1;
            }
            else
            {
                turnOffLampNumber(lampOprErr);
                turnOffLampNumber(lampKeyRelease);
                setLamp(green, lampProg);
                mode = modeIdle;
                count = 0;
                fresh = false;
                error = 0;
                newProg = true;
            }
        }
        if ((keyValue == keyRelease) && (fresh == true))
        {
            mode = oldMode;
            prog = prog_old;
            turnOffLampNumber(lampKeyRelease);
            setLamp(green, lampProg);
            count = 0;
            fresh = false;
            printProg(prog);
        }
        if ((keyValue <= keyNumber9) && (count < 2))
        { // now the actual prog values are read, stored and printed
            progNew[count] = keyValue;
            setDigits(0, count+2, keyValue);
            count++;
            fresh = false;
        }
    }
}

void executeProgramInputMode()
{ // inputting the program
    setLamp(yellow, lampProg);
    toggleKeyReleaseLamp();
    if (error == 1) {
        flasher();
    }
    keyValue = readKeyboard();
    processProgramInputMode();
}

//void processkeytime() {
//}

void executeLampTestModeWithDuration(int durationInMilliseconds)
{
    for (int index = 11; index < 18; index++) {
        // Uplink Acty, No Att, Stby, Key Rel, Opr Err, --, --
        delay(200);
        illuminateWithRGBAndLampNumber(100, 100, 60, index);    // less blue = more white
    }

    for (int index = 4; index < 11; index++) {
        // Temp, Gimbal Loc, Prog, Restart, Tracker, Alt, Vel
        delay(200);
        illuminateWithRGBAndLampNumber(120, 110, 0, index);     // more yellow
    }

    for (int lampNumber = 0; lampNumber < 4; lampNumber++) {
        // Comp Acty, Prog, Verb, Noun
        delay(200);
        illuminateWithRGBAndLampNumber(0, 150, 0, lampNumber);
    }

    int lampTestDigitValue = 8;
    // passes number "8" to all the 7-segment numeric displays
    for (int row = 0; row < 4; row++) {
        // row 0 = Prog/Verb/Noun
        // row 1 = Register 1
        // row 2 = Register 2
        // row 3 = Register 3
        // ... each has six positions
        // note: 'digit' # 0 in the three registers is the plus/minus sign
        for (int digitPosition = 0; digitPosition < 6; digitPosition++) {
            delay(200);
            setDigits(row, digitPosition, lampTestDigitValue);
        }
    }

    delay(durationInMilliseconds);

    // reset all lamps
    for (int index = 0; index < 4; index++) {
        delay(200);
        turnOffLampNumber(index);
    }
    for (int index = 4; index < 11; index++) {
        delay(200);
        turnOffLampNumber(index);
    }
    for (int index = 11; index < 18; index++) {
        delay(200);
        turnOffLampNumber(index);
    }
    for (int index = 0; index < 4; index++) {
        delay(200);
        ledControl.clearDisplay(index);
    }

    // restore previously-displayed values for Verb and Noun
    setLamp(green, lampVerb);
    setLamp(green, lampNoun);
    setLamp(green, lampProg);
    /*Serial.println("Lamptest V35");
    Serial.print("verb_old2 : "); Serial.println(verb_old2);
    Serial.print("verb_old  : "); Serial.println(verb_old);
    Serial.print("verb      : "); Serial.println(verb);
    Serial.print("noun_old2 : "); Serial.println(noun_old2);
    Serial.print("noun_old  : "); Serial.println(noun_old);
    Serial.print("noun      : "); Serial.println(noun); */
    // blank Verb readout if needed
    //verb = ((verbOld[0] * 10) + verbOld[1]);
    verb = verb_old;
    /*Serial.println("Lamptest V35 verb = verb_old");
    Serial.print("verb_old2 : "); Serial.println(verb_old2);
    Serial.print("verb_old  : "); Serial.println(verb_old);
    Serial.print("verb      : "); Serial.println(verb);
    Serial.print("noun_old2 : "); Serial.println(noun_old2);
    Serial.print("noun_old  : "); Serial.println(noun_old);
    Serial.print("noun      : "); Serial.println(noun);*/
    printVerb(verb);
    
    // blank Prog readout if needed
    printProg(prog);
    noun = noun_old;
    /*Serial.println("Lamptest V35 noun = noun_old");
    Serial.print("verb_old2 : "); Serial.println(verb_old2);
    Serial.print("verb_old  : "); Serial.println(verb_old);
    Serial.print("verb      : "); Serial.println(verb);
    Serial.print("noun_old2 : "); Serial.println(noun_old2);
    Serial.print("noun_old  : "); Serial.println(noun_old);
    Serial.print("noun      : "); Serial.println(noun); */
    printNoun(noun);
    keyValue = keyNone;
    mode = modeIdle;
    validateAction();
}

void startupsequence(int durationInMilliseconds)
{
    for (int index = 11; index < 18; index++) {
        // Uplink Acty, No Att, Stby, Key Rel, Opr Err, --, --
        delay(50);
        illuminateWithRGBAndLampNumber(100, 100, 60, index);    // less blue = more white
    }

    for (int index = 4; index < 11; index++) {
        // Temp, Gimbal Loc, Prog, Restart, Tracker, Alt, Vel
        delay(50);
        illuminateWithRGBAndLampNumber(120, 110, 0, index);     // more yellow
    }

    for (int lampNumber = 0; lampNumber < 4; lampNumber++) {
        // Comp Acty, Prog, Verb, Noun
        delay(50);
        illuminateWithRGBAndLampNumber(0, 150, 0, lampNumber);
    }

    int lampTestDigitValue = 8;
    // passes number "8" to all the 7-segment numeric displays
    for (int row = 0; row < 4; row++) {
        // row 0 = Prog/Verb/Noun
        // row 1 = Register 1
        // row 2 = Register 2
        // row 3 = Register 3
        // ... each has six positions
        // note: 'digit' # 0 in the three registers is the plus/minus sign
        for (int digitPosition = 0; digitPosition < 6; digitPosition++) {
            delay(50);
            setDigits(row, digitPosition, lampTestDigitValue);
        }
    }

    delay(durationInMilliseconds);

    // reset all lamps
    for (int index = 0; index < 4; index++) {
        delay(50);
        turnOffLampNumber(index);
    }
    for (int index = 4; index < 11; index++) {
        delay(50);
        turnOffLampNumber(index);
    }
    for (int index = 11; index < 18; index++) {
        delay(50);
        turnOffLampNumber(index);
    }
    for (int index = 0; index < 4; index++) {
        delay(50);
        ledControl.clearDisplay(index);
    }

    // restore previously-displayed values for Verb and Noun
    setLamp(green, lampVerb);
    setLamp(green, lampNoun);
    setLamp(green, lampProg);
    keyValue = keyNone;
    mode = modeIdle;
    
}

void actionReadTime()
{
    // read time from real-time clock (RTC)
    DateTime now = realTimeClock.now();
    // 10th and hundreds of seconds
    if( oldSecond < now.second() )
    {
        oldSecond = now.second();
        previousMillis = millis();
    }
    int hundreds = ( ( millis()-previousMillis )/10 )%100;
    int tenth = hundreds - (hundreds % 10);
    //valueForDisplay[register1Position] = (now.hour());
    //valueForDisplay[register2Position] = (now.minute());
    //valueForDisplay[register3Position] = ((now.second() * 100) + tenth);
    //setDigits();
    printRegister(1,(now.hour()));
    printRegister(2,(now.minute()));
    printRegister(3,((now.second() * 100) + tenth));
}


void actionReadGPS()
{ // Read GPS
  if (toggle == true && gpsread == true)
  {
    //ALT_light_on();
    if (gpsfix == false)
    {
        setLamp(yellow, lampPosition);
    }
    else if (gpsfix == true)
    {
        setLamp(off, lampPosition);
    }
    
    digitalWrite(7,HIGH);
    delay(100);
    gpsread = false;
    // int index = 0;
    Serial.begin(9600);
    //delay(200);
    while((Serial.available()) && (GPS_READ_STARTED == true))
    {
      setLamp(white, lampAlt);
      if (gps.encode(Serial.read()))
      {
         //setLamp(orange, lampPosition);
         setLamp(orange, lampVel);
         GPS_READ_STARTED = false;
      }
    }
    digitalWrite(7,LOW);
    
    setLamp(off, lampAlt);
    setLamp(off, lampVel);
    //if (gps.location.lat() != 0)
    if (gps.location.isValid() == 1)
    {
        gpsfix = true;
    } 
    //else if (gps.location.lat() == 0)
    else if (gps.location.isValid() != 1)
    {
        gpsfix = false;
    }
    printRegister(1,gps.location.lat()*100);
    printRegister(2,gps.location.lng()*100);
    printRegister(3,gps.altitude.meters());
  }
  if (toggle == false)
  {
     gpsread = true;
     GPS_READ_STARTED = true;
  }
}

void actionSetTime()
{   // read & display time from hardware real-time clock (RTC)
    DateTime now = realTimeClock.now();
    int nowYear = now.year();
    int nowMonth = now.month();
    int nowDay = now.day();
    int nowHour = now.hour();
    int nowMinute = now.minute();
    int nowSecond = now.second();

    while (keyValue == keyEnter) {
        keyValue = readKeyboard();
    }

    while (keyValue != keyEnter) {
        keyValue = readKeyboard();
        if (keyValue != oldKey) {
            oldKey = keyValue;
            if (keyValue == keyPlus) {
                nowHour++;
            }
            if (keyValue == keyMinus) {
                nowHour--;
            }
            if (nowHour > 23) {
                nowHour = 0;
            }
            if (nowHour < 0) {
                nowHour = 23;
            }
        }
        printRegister(1,nowHour);
        printRegister(2,nowMinute);
        printRegister(3,(nowSecond * 100)); // emulate milliseconds
    }

    while (keyValue == keyEnter) {
        keyValue = readKeyboard();
    }

    while (keyValue != keyEnter) {
        keyValue = readKeyboard();
        if (keyValue != oldKey) {
            oldKey = keyValue;
            if (keyValue == keyPlus) {
                nowMinute++;
            }
            if (keyValue == keyMinus) {
                nowMinute--;
            }
            if (nowMinute > 59) {
                nowMinute = 0;
            }
            if (nowMinute < 0) {
                nowMinute = 59;
            }
        }
        printRegister(1,nowHour);
        printRegister(2,nowMinute);
        printRegister(3,(nowSecond * 100));
    }

    while (keyValue == keyEnter) {
        keyValue = readKeyboard();
    }

    while (keyValue != keyEnter) {
        keyValue = readKeyboard();
        if (keyValue != oldKey) {
            oldKey = keyValue;
            if (keyValue == keyPlus) {
                nowSecond++;
            }
            if (keyValue == keyMinus) {
                nowSecond--;
            }
            if (nowSecond > 59) {
                nowSecond = 0;
            }
            if (nowSecond < 0) {
                nowSecond = 59;
            }
        }
        printRegister(1,nowHour);
        printRegister(2,nowMinute);
        printRegister(3,(nowSecond *100));
    }
    realTimeClock.adjust(DateTime(nowYear, nowMonth, nowDay, nowHour, nowMinute, nowSecond));
    action = displayRealTimeClock;
    setDigits(0, 0, 1);
    setDigits(0, 1, 6);
    verb = verbDisplayDecimal;
    verbOld[0] = 1;
    verbOld[1] = 6;
}

void actionSetDate()
{
    byte yearToSet[4];
    byte monthToSet[2];
    byte dayToSet[2];


    DateTime now = realTimeClock.now();
    int nowYear = now.year();
    int nowMonth = now.month();
    int nowDay = now.day();
    int nowHour = now.hour();
    int nowMinute = now.minute();
    int nowSecond = now.second();

    realTimeClock.adjust(DateTime(
                                  ((yearToSet[0] * 10^3) + (yearToSet[1] * 10^2) + (yearToSet[2] * 10) + yearToSet[3]),
                                  ((monthToSet[0] * 10) + monthToSet[1]),
                                  ((dayToSet[0] * 10) + dayToSet[1]),
                                  nowHour,
                                  nowMinute,
                                  nowSecond)
                         );
}

/*
void mode11() {
    flashUplinkAndComputerActivityRandomly();
}
 */

// V21 N98 read & enter & play the selected Audio Clip
void actionSelectAudioclip()
{   // V21 N98 read & enter & play the selected Audio Clip
    // first print initial clipnum = 1
    printRegister(1,clipnum);
    setLamp(off,lampProg);
    // enter can be pressed several times?
    while (keyValue == keyEnter) {
        keyValue = readKeyboard();
    }

    while (keyValue != keyEnter)
    { // now something else than enter has been pressed
        unsigned long blink_currentMillis = millis();
        if (blink_currentMillis - blink_previousMillis >= blink_interval)
        {
            // save the last time you blinked the LED
            blink_previousMillis = blink_currentMillis;
            // if the LED is off turn it on and vice-versa:

            if (blink == true)
            {
                blink = false;
            } else {
                blink = true;
            }
            printVerb(verb, blink);
            printNoun(noun, blink);
        }
        keyValue = readKeyboard();
        if (keyValue != oldKey)
        {
            oldKey = keyValue;
            if (keyValue == keyPlus) {
                clipnum++;
            }
            if (keyValue == keyMinus) {
                clipnum--;
            }
            if (clipnum > clipcount) {
                clipnum = 1;
            }
            if (clipnum < 1) {
                clipnum = clipcount;
            }
        }
        printRegister(1,clipnum);
    }
    action = PlaySelectedAudioclip;
    verb = verbDisplayDecimal;
    noun = nounSelectAudioclip;
    setLamp(green, lampProg);
    printProg(clipnum);
}

void playTrack(uint8_t track)
{
   myDFPlayer.stop();
   delay(200);
   myDFPlayer.play(track);
   delay(200);
   int file = myDFPlayer.readCurrentFileNumber();
   while (file != track) {
     myDFPlayer.play(track);
     delay(200);
     file = myDFPlayer.readCurrentFileNumber();
   }
}

// V16 N98 play the selected Audio Clip
void actionPlaySelectedAudioclip(int clipnum)
{   // V16 N98 play the selected Audio Clip
    printVerb(verb);
    printNoun(noun);
    playTrack(clipnum);
    action = none;
    verb = verbNone;
    noun = nounNone;
    prog = programNone;
    printVerb(verb);
    printNoun(noun);
    printProg(prog);
    setLamp(green, lampProg);
    clearRegister(1);
    clearRegister(2);
    clearRegister(3);
}

void flashUplinkAndComputerActivityRandomly()
{
    if ((toggle600 == true) && (uplink_compact_toggle == true))
    {
        uplink_compact_toggle = false;
        int randomNumber = random(1, 50);
        if ((randomNumber == 15) || (randomNumber == 25)) {
            setLamp(green,lampCompActy);
        }
        else {
            setLamp(off,lampCompActy);
        }
        if ((randomNumber == 17) || (randomNumber == 25)) {
            setLamp(white,lampUplinkActy);
        }
        else {
            setLamp(off,lampUplinkActy);
        }
    }
    else if ((toggle600 == false) && (uplink_compact_toggle == false))
    {
        uplink_compact_toggle = true;
    }
}







void readIMU(int imumode)
{  // reads the IMU Values mode Gyro or Accel
    /* 
    https://elektro.turanis.de/html/prj075/index.html
    https://github.com/griegerc/arduino-gy521/blob/master/gy521-read-angle/gy521-read-angle.ino
    const int ACCEL_OFFSET   = 200;
    const int GYRO_OFFSET    = 151;  // 151
    const int GYRO_SENSITITY = 131;  // 131 is sensivity of gyro from data sheet
    const float GYRO_SCALE   = 2; //  0.02 by default - tweak as required
    const float LOOP_TIME    = 0.15; // 0.1 = 100ms
    */
    Wire.beginTransmission(MPU_addr);
    Wire.write(0x3B);  // starting with register 0x3B (ACCEL_XOUT_H)
    Wire.endTransmission(false);
    Wire.requestFrom(MPU_addr,14,true);  // request a total of 14 registers
    int accValueX = 0;
    int accValueY = 0;
    int accValueZ = 0;
    int accCorrX = 0;
    int accCorrY = 0;
    int accCorrZ = 0;
    float accAngleX = 0.0;
    float accAngleY = 0.0;
    float accAngleZ = 0.0;
    int temp = 0;
    int gyroValueX = 0;
    int gyroValueY = 0;
    int gyroValueZ = 0;
    float gyroAngleX = 0.0;
    float gyroAngleY = 0.0;
    float gyroAngleZ = 0.0; 
    float gyroCorrX = 0.0;
    float gyroCorrY = 0.0;
    float gyroCorrZ = 0.0;
  
    accValueX = (Wire.read() << 8) | Wire.read();  // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)
    accValueY = (Wire.read() << 8) | Wire.read();  // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
    accValueZ = (Wire.read() << 8) | Wire.read();  // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
    temp = (Wire.read() << 8) | Wire.read();  // 0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)
    gyroValueX = (Wire.read() << 8) | Wire.read();  // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
    gyroValueY = (Wire.read() << 8) | Wire.read();  // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
    gyroValueZ = (Wire.read() << 8) | Wire.read();  // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)
        
    temp = (temp / 340.00 + 36.53); //equation for temperature in degrees C from datasheet
        
        
    accCorrX = accValueX - ACCEL_OFFSET;
    accCorrX = map(accCorrX, -ACCEL_SCALE, ACCEL_SCALE, -90, 90);
    accAngleX = constrain(accCorrX, -90, 90);
    // our IMU sits upside down in the DSKY, so we have to flip the angle
    accAngleX = -accAngleX;
    accAngleX = accAngleX + ACC_OFFSET_X;

    accCorrY = accValueY - ACCEL_OFFSET;
    accCorrY = map(accCorrY, -ACCEL_SCALE, ACCEL_SCALE, -90, 90);

    accAngleY = constrain(accCorrY, -90, 90);
    accAngleY = accAngleY + ACC_OFFSET_Y;

    accCorrZ = accValueZ - ACCEL_OFFSET;
    accCorrZ = map(accCorrZ, -ACCEL_SCALE, ACCEL_SCALE, -90, 90);
    accAngleZ = constrain(accCorrZ, -90, 90);
        // our IMU sits upside down in the DSKY, so we have to flip the angle
    accAngleZ = -accAngleZ;
    accAngleZ = accAngleZ + ACC_OFFSET_Z;

    gyroCorrX = (float)((gyroValueX/GYRO_SENSITITY)+GYRO_OFFSET_X);
    gyroAngleX = (gyroCorrX * GYRO_GRANGE) * -LOOP_TIME;
    gyroCorrY = (float)((gyroValueY/GYRO_SENSITITY)+GYRO_OFFSET_Y);
    gyroAngleY = (gyroCorrY * GYRO_GRANGE) * -LOOP_TIME;
    gyroCorrZ = (float)((gyroValueZ/GYRO_SENSITITY)+GYRO_OFFSET_Z);
    gyroAngleZ = (gyroCorrZ * GYRO_GRANGE) * -LOOP_TIME;
    setLamp(off, lampNoAtt);
    if (imumode == Gyro)
        {
            printRegister(1,int(gyroAngleX*100));
            printRegister(2,int(gyroAngleY*100));
            printRegister(3,int(gyroAngleZ*100));
        }
    else if (imumode == Accel)
        {
            printRegister(1,int(accAngleX*100));
            printRegister(2,int(accAngleY*100));
            printRegister(3,int(accAngleZ*100));
    }
    
}

void actionReadIMU(int imumode)
{
    if ((toggle600 == true) && (imutoggle == true))
    {   // only every 600ms an imuupdate to avoid flickering
        imutoggle = false;
        flashUplinkAndComputerActivityRandomly();
        readIMU(imumode);
        
        
    }
    else if ((toggle600 == false) && (imutoggle == false))
    {
        flashUplinkAndComputerActivityRandomly();
        imutoggle = true;
        readIMU(imumode);
    }
}


void jfk(byte jfk)
{
    if (audioTrack > 3) {
        audioTrack = 1;
    }

    while (audioTrack != jfk) {
        pinMode(9, OUTPUT);
        delay(100);
        pinMode(9, INPUT);
        delay(100);
        audioTrack++;
        if (audioTrack > 3) {
            audioTrack = 1;
        }
    }

    pinMode(9, OUTPUT);
    delay(100);
    pinMode(9, INPUT);
    audioTrack++;
    currentProgram = programNone;
    if (currentProgram == 0)
    {
      ledControl.setRow(0, 2, 0);
      ledControl.setRow(0, 3, 0);
    }
}

void setup()
{
    pinMode(A0, INPUT);
    pinMode(A1, INPUT);
    pinMode(A2, INPUT);
    pinMode(A7, INPUT);
    pinMode(7, OUTPUT);
    digitalWrite(7, LOW);
    randomSeed(analogRead(A7));
    neoPixels.begin();

    for (int index = 0; index < 4; index++) {
        ledControl.shutdown(index,false);
        ledControl.setIntensity(index, 8);
        ledControl.clearDisplay(index);
    }

    Wire.begin();
    Wire.beginTransmission(MPU_addr);
    Wire.write(0x6B);  // PWR_MGMT_1 register
    Wire.write(0);     // set to zero (wakes up the MPU-6050)
    Wire.endTransmission(true);

    realTimeClock.begin();

    // Toggle 
    timer.every(1000, toggle_timer);
    timer.every(600, toggle_timer_600);
    timer.every(100, toggle_timer_250);
    
    Serial.begin(9600);
    // DFPlayerMini initialize
    mySoftwareSerial.begin(9600);
    Serial.println();
    Serial.println(F("DFRobot DFPlayer Mini Demo"));
    Serial.println(F("Initializing DFPlayer ... (May take 3~5 seconds)"));
    if (!myDFPlayer.begin(mySoftwareSerial))
    {  //Use softwareSerial to communicate with mp3.
        Serial.println(F("Unable to begin:"));
        Serial.println(F("1.Please recheck the connection!"));
        Serial.println(F("2.Please insert the SD card!"));
    while(true);
    }
    Serial.println(F("DFPlayer Mini online."));
    myDFPlayer.volume(20);  //Set volume value. From 0 to 30
    clipcount = myDFPlayer.readFileCounts();
    Serial.println(clipcount); //read all file counts in SD card
    //startupsequence(100);
    delay(100);
    clearRegister(1);
    delay(100);
    clearRegister(2);
    delay(100);
    clearRegister(3);
    delay(100);

    setLamp(white, lampNoAtt);
    delay(100);
    setLamp(white, lampPosition);
    delay(100);
    setLamp(green, lampNoun);
    delay(100);
    setLamp(green, lampVerb);
    delay(100);
    setLamp(green, lampProg);
    delay(100);
}

void loop()
{
    timer.tick(); // toggle on / off
    if (toggle == true)
    {
        if ((toggle250 == true) && (toggled250 == false))
        {
            setLamp(white, lampClk);
            toggled250 = true;
        }
        else if ((toggle250 == false) && (toggled250 == true))
        {
            setLamp(off, lampClk);
        }
    }
    else
    {
        //setLamp(off, lampClk);
        if ((toggle250 == true) && (toggled250 == true))
        {
            setLamp(white, lampClk);
            toggled250 = false;
        }
        else if ((toggle250 == false) && (toggled250 == false))
        {
            setLamp(off, lampClk);
        }
    }
    
    if (currentProgram == programJFKAudio) {
        //jfk(1);
        playTrack(19);
        currentProgram = programNone;
        action = none;
        verb = verbNone;
        noun = nounNone;
        prog = programNone;
        printVerb(verb);
        printNoun(noun);
        printProg(prog);
        clearRegister(1);
        clearRegister(2);
        clearRegister(3);
    }
    else if (currentProgram == programApollo11Audio) {
        //jfk(2);
        playTrack(20);
        currentProgram = programNone;
        action = none;
        verb = verbNone;
        noun = nounNone;
        prog = programNone;
        printVerb(verb);
        printNoun(noun);
        printProg(prog);
        clearRegister(1);
        clearRegister(2);
        clearRegister(3);
    }
    else if (currentProgram == programApollo13Audio) {
        //jfk(3);
        playTrack(21);
        currentProgram = programNone;
        action = none;
        verb = verbNone;
        noun = nounNone;
        prog = programNone;
        printVerb(verb);
        printNoun(noun);
        printProg(prog);
        clearRegister(1);
        clearRegister(2);
        clearRegister(3);
    }

    if (mode == modeIdle) {
        executeIdleMode();
        if (action == none)
        {
            setLamp(white, lampSTBY);
        }
        else if (action != none)
        {
            setLamp(off, lampSTBY);
        }
    }
    else if (mode == modeInputVerb) {
        executeVerbInputMode();
        setLamp(off, lampSTBY);

    }
    else if (mode == modeInputNoun) {
        executeNounInputMode();
        setLamp(off, lampSTBY);
    }
    else if (mode == modeInputProgram) {
        executeProgramInputMode();
        setLamp(off, lampSTBY);
    }
    else if (mode == modeLampTest) {
        setLamp(off, lampSTBY);
        executeLampTestModeWithDuration(2000);
    }
    
    if (action == displayIMUAttitude) {
        actionReadIMU(Accel);  // V16N17 ReadIMU Accel
    }
    if (action == displayIMUGyro) {
        actionReadIMU(Gyro);  // V16N18 ReadIMU Gyro
    }
    else if (action == displayRealTimeClock) {
        actionReadTime();   // V16N36 ReadTime
    }
    else if (action == displayGPS) {
        actionReadGPS();    // V16N43 Read GPS
    }
    else if (action == setTime) {
        actionSetTime();    // V21N36 Set The Time
    }
    else if (action == setDate) {
        actionSetDate();    // V21N37 Set The Date
    }
    else if (action == PlayAudioclip) 
    {   // V21N98 Play Audio Clip
        actionSelectAudioclip();    
    }
    else if (action == PlaySelectedAudioclip) 
    {   // V16N98 Play Audio Clip
        actionPlaySelectedAudioclip(clipnum);    
    }
    //Serial.print(verb);
    //Serial.print("  ");
    //Serial.print(noun);
    //Serial.print("  ");
    //Serial.println(action);
}