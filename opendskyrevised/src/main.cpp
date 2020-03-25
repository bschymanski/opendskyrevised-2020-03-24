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
    off                     = 6
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
{ // Nouns 0,17,36,37,43,68
    nounNone                = 0,
    nounIMUAttitude         = 17,
    nounClockTime           = 36,
    nounDate                = 37,
    nounLatLongAltitude     = 43,
    nounRangeTgoVelocity    = 68
};

enum registerDisplayPositions: int 
{ // Display Register Positions
    register1Position       = 4,
    register2Position       = 5,
    register3Position       = 6
};

long valueForDisplay[7];
byte digitValue[7][7];
byte keyValue = keyNone;
byte oldKey = none;
bool fresh = true;
byte action = none;
byte currentAction = none;
byte verb = verbNone;
byte verbNew[2];
byte verbOld[2];
byte noun = 0;
byte nounNew[2];
byte nounOld[2];
byte currentProgram = programNone;
byte prog = 0;
byte progNew[2];
byte progOld[2];
bool newProg = false;
byte count = 0;
byte mode = modeIdle;
byte oldMode = modeIdle;
bool toggle = false;
byte toggleCount = 0;
bool error = 0;
bool newAction = false;
byte audioTrack = 1;

int lat = 0;
int lon = 0;
int alt = 0;

int globaltimer=0;
bool global_state_1sec=false;

// GPS Definitions
bool GPS_READ_STARTED = true;
bool gpsread = true;
bool gpsfix = false;

// variables for Time display (10th of a second, 100th of a second)
unsigned long previousMillis = 0;
int oldSecond = 0;

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

void setup() {
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
    myDFPlayer.volume(10);  //Set volume value. From 0 to 30
    Serial.println(myDFPlayer.readFileCounts()); //read all file counts in SD card
}

void validateAction() {
    if (verb == verbLampTest) {
        mode = modeLampTest;
        newAction = false;
    }
    else if ((verb == verbDisplayDecimal) && (noun == nounIMUAttitude)) {
        action = displayIMUAttitude;
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
        off                     = 6
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
            neoPixels.setPixelColor(lampNumber, neoPixels.Color(100,0,0));
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

void setDigits(byte maximum, byte digit, byte value)
{//Serial.println("setDigits(byte ...)");
    ledControl.setDigit(maximum, digit, value, false);
}

void flasher() {
    if (toggle == false) {
        illuminateWithRGBAndLampNumber(100, 100, 100, lampOprErr);
    } else {
        turnOffLampNumber(lampOprErr);
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


void processIdleMode() {
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

void executeIdleMode() {
    // no action set just reading the kb
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
        illuminateWithRGBAndLampNumber(100, 100, 100, lampKeyRelease);
    }
    else {
        turnOffLampNumber(lampKeyRelease);
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
        if ((error == 1) && (keyValue == keyReset) && (fresh == true)) {
            error = 0; turnOffLampNumber(lampOprErr); fresh = false;
        } //resrt reeor
        if ((keyValue == keyEnter) && (fresh == true)) {
            fresh = false;
            verb = ((verbNew[0] * 10) + (verbNew[1]));
            if ((verb != verbDisplayDecimal)
                && (verb != verbSetComponent)
                && (verb != verbLampTest)
                && (verb != verbNone)) {
                error = 1;
                verb = ((verbOld[0] * 10) + verbOld[1]);    // restore prior verb
            }
            else {
                turnOffLampNumber(lampOprErr);
                turnOffLampNumber(lampKeyRelease);
                turnOffLampNumber(lampVerb);
                mode = modeIdle;
                count = 0;
                fresh = false;
                error = 0;
                newAction = true;
            }
        }

        if (fresh == true) {
            if (keyValue == keyRelease) {
                mode = oldMode;
                turnOffLampNumber(lampKeyRelease);
                turnOffLampNumber(lampVerb);
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
                turnOffLampNumber(lampVerb);
                count = 0;
                fresh = false;
            }
            else if (keyValue == keyProceed) {
                //program
                mode = modeInputProgram;
                turnOffLampNumber(lampVerb);
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
    illuminateWithRGBAndLampNumber(0, 150, 0, lampVerb);
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
            turnOffLampNumber(lampOprErr);
            fresh = false;
        } //resrt reeor

        if ((keyValue == keyEnter) && (fresh == true)) {
            fresh = false;
            noun = ((nounNew[0] * 10) + (nounNew[1]));
            fresh = false;
            if ((noun != nounIMUAttitude)
                && (noun != nounClockTime)
                && (noun != nounLatLongAltitude)
                && (noun != nounRangeTgoVelocity)
                && (noun != nounNone)) {
                noun = ((nounOld[0] * 10) + nounOld[1]);    // restore prior noun
                error = 1;
            }
            else {
                turnOffLampNumber(lampOprErr);
                turnOffLampNumber(lampKeyRelease);
                turnOffLampNumber(lampNoun);
                mode = modeIdle;
                count = 0;
                fresh = false;
                error = 0;
                newAction = true;
            }
        }

        if ((keyValue == keyRelease) && (fresh == true)) {
            mode = oldMode;
            turnOffLampNumber(lampKeyRelease);
            turnOffLampNumber(lampNoun);
            count = 0;
            fresh = false;
            if (noun == 0) {
                //verb
                ledControl.setRow(0, 4, 0);
                ledControl.setRow(0, 5, 0);
            }
            else {
                setDigits(0, 4, nounOld[0]);
                setDigits(0, 5, nounOld[1]);
            }
        }
        if ((keyValue == keyVerb) && (fresh == true)) {
            //verb
            mode = modeInputVerb;
            turnOffLampNumber(lampNoun);
            count = 0;
            fresh = false;
        }
        if ((keyValue == keyProceed) && (fresh == true)) {
            mode = modeInputProgram;
            turnOffLampNumber(lampNoun);
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
    illuminateWithRGBAndLampNumber(0, 150, 0, lampNoun);
    toggleKeyReleaseLamp();
    if (error == 1) {
        flasher();
    }
    keyValue = readKeyboard();
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
        //Serial.print("neuer Prog Key wurde eingegeben: ");
        //Serial.println(keyValue);
        //Serial.print("prog   : "); Serial.println(currentProgram);
        if ((error == 1) && (keyValue == keyReset) && (fresh == true))
        {
            error = 0;
            turnOffLampNumber(lampOprErr);
            fresh = false;
            //Serial.print("Error = 1, Keyreset wurde gedrückt, ");
            //Serial.print("neuer Prog Key wurde eingegeben: ");
            //Serial.println(keyValue);
            //Serial.print("prog   : "); Serial.println(currentProgram);
        }
        if ((keyValue == keyEnter) && (fresh == true)) 
        {
            fresh = false;
            currentProgram = ((progNew[0] * 10) + (progNew[1]));
            fresh = false;
            if ((currentProgram != programNone)
                && (currentProgram != programJFKAudio)
                && (currentProgram != programApollo11Audio)
                && (currentProgram != programApollo13Audio))
            {
                currentProgram = ((progOld[0] * 10) + progOld[1]);    // restore prior noun
                error = 1;
            }
            else
            {
                turnOffLampNumber(lampOprErr);
                turnOffLampNumber(lampKeyRelease);
                turnOffLampNumber(lampProg);
                mode = modeIdle;
                count = 0;
                fresh = false;
                error = 0;
                newProg = true;
            }
            //Serial.print("KeyEnter wurde gedrückt, ");
            //Serial.print("neuer Prog Key wurde eingegeben: ");
            //Serial.println(keyValue);
            //Serial.print("prog   : "); Serial.println(currentProgram);
        }
        if ((keyValue == keyRelease) && (fresh == true))
        {
            mode = oldMode;
            turnOffLampNumber(lampKeyRelease);
            turnOffLampNumber(lampProg);
            count = 0;
            fresh = false;
            if (currentProgram == 0) {
                ledControl.setRow(0, 2, 0);
                ledControl.setRow(0, 3, 0);
            }
            else {
                setDigits(0, 2, progOld[0]);
                setDigits(0, 3, progOld[1]);
            }
            //Serial.print("KeyRelease wurde gedrückt");
            //Serial.print("neuer Prog Key wurde eingegeben: ");
            //Serial.println(keyValue);
            //Serial.print("prog   : "); Serial.println(currentProgram);
        }
        if ((keyValue <= keyNumber9) && (count < 2))
        { // hier wird das neue Prog eingegeben
            progNew[count] = keyValue;
            setDigits(0, count+2, keyValue);
            count++;
            fresh = false;
            //Serial.print("0-9 wurde gedrückt, ");
            //Serial.print("neuer Prog Key wurde eingegeben: ");
            //Serial.println(keyValue);
            //Serial.print("prog   : "); Serial.println(currentProgram);
        }
    }
}

void executeProgramInputMode()
{ // inputting the program
    illuminateWithRGBAndLampNumber(0, 150, 0, lampProg);
    toggleKeyReleaseLamp();
    if (error == 1) {
        flasher();
    }
    keyValue = readKeyboard();
    processProgramInputMode();
}

//void processkeytime() {
//}

void executeLampTestModeWithDuration(int durationInMilliseconds) {
    for (int index = 11; index < 18; index++) {
        // Uplink Acty, No Att, Stby, Key Rel, Opr Err, --, --
        illuminateWithRGBAndLampNumber(100, 100, 60, index);    // less blue = more white
    }

    for (int index = 4; index < 11; index++) {
        // Temp, Gimbal Loc, Prog, Restart, Tracker, Alt, Vel
        illuminateWithRGBAndLampNumber(120, 110, 0, index);     // more yellow
    }

    for (int lampNumber = 0; lampNumber < 4; lampNumber++) {
        // Comp Acty, Prog, Verb, Noun
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
            setDigits(row, digitPosition, lampTestDigitValue);
        }
    }

    delay(durationInMilliseconds);

    // reset all lamps
    for (int index = 0; index < 4; index++) {
        turnOffLampNumber(index);
    }
    for (int index = 4; index < 11; index++) {
        turnOffLampNumber(index);
    }
    for (int index = 11; index < 18; index++) {
        turnOffLampNumber(index);
    }
    for (int index = 0; index < 4; index++) {
        ledControl.clearDisplay(index);
    }

    // restore previously-displayed values for Verb and Noun
    verbNew[0] = verbOld[0];
    verbNew[1] = verbOld[1];

    // blank Verb readout if needed
    verb = ((verbOld[0] * 10) + verbOld[1]);
    if (verb == verbNone) {
        ledControl.setRow(0, 0, 0); ledControl.setRow(0, 1, 0);
    }
    else {
        setDigits(0, 0, verbOld[0]);
        setDigits(0, 1, verbOld[1]);
    }

    // blank Prog readout if needed
    if (currentProgram == programNone) {
        ledControl.setRow(0, 2, 0);
        ledControl.setRow(0, 3, 0);
    }
    else {
        setDigits(0, 0, progNew[0]);
        setDigits(0, 1, progNew[1]);
    }

    // blank Noun readout if needed
    if (noun == 0) {
        ledControl.setRow(0, 4, 0);
        ledControl.setRow(0, 5, 0);
    }
    else {
        setDigits(0, 4, nounNew[0]);
        setDigits(0, 5, nounNew[1]);
    }
    keyValue = keyNone;
    mode = modeIdle;
    validateAction();
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
    valueForDisplay[register1Position] = (now.hour());
    valueForDisplay[register2Position] = (now.minute());
    valueForDisplay[register3Position] = ((now.second() * 100) + tenth);
    setDigits();
}

void actionReadGPS()
{
//PrintMode();
  //PrintAction();
  if (toggle == true && gpsread == true)
  {
    //ALT_light_on();
    if (gpsfix == false)
    {
        setLamp(yellow, lampPosition);
    }
    else if (gpsfix == true)
    {
        setLamp(white, lampPosition);
    }
    
    digitalWrite(7,HIGH);
    delay(20);
    gpsread = false;
    // int index = 0;
    Serial.begin(9600);
    delay(30);
    while((Serial.available()) && (GPS_READ_STARTED == true))
    {
      setLamp(white, lampAlt);
      if (gps.encode(Serial.read()))
      {
         //setLamp(orange, lampPosition);
         setLamp(orange, lampVel);
         GPS_READ_STARTED = false;
         //Serial.print("2 GPS_READ_STARTED ");Serial.println(GPS_READ_STARTED);  
      }
    }
    digitalWrite(7,LOW);
    Serial.print("lat        ");Serial.println(gps.location.lat(), 6);
    Serial.print("lon        ");Serial.println(gps.location.lng(), 6);
    Serial.print("alt        ");Serial.println(gps.altitude.meters(), 6);
    Serial.print("age        ");Serial.println(gps.altitude.age(), 6);
    Serial.print("is updated ");Serial.println(gps.location.isUpdated(), 6);
    Serial.print("is valid   ");Serial.println(gps.location.isValid(), 6);
    
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
    valueForDisplay[register1Position] = gps.location.lat()*100;
    valueForDisplay[register2Position] = gps.location.lng()*100;
    valueForDisplay[register3Position] = gps.altitude.meters();
    Serial.print("gpsfix ");Serial.println(gpsfix);  
    setDigits();
  }
  if (toggle == false)
  {
     gpsread = true;
     GPS_READ_STARTED = true;
  }
    

//    valueForDisplay[register1Position] = latitude;
//    valueForDisplay[register2Position] = longitude;
//    valueForDisplay[register3Position] = altitude;
//    digitalWrite(7, LOW);
//    setDigits();
}

void actionSetTime() {
    // read & display time from hardware real-time clock (RTC)
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
        Serial.println(keyValue);
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
        valueForDisplay[register1Position] = nowHour;
        valueForDisplay[register2Position] = nowMinute;
        valueForDisplay[register3Position] = (nowSecond * 100); // emulate milliseconds
        setDigits();
        delay(200);
        ledControl.clearDisplay(1);
        delay(50);
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
        valueForDisplay[register1Position] = nowHour;
        valueForDisplay[register2Position] = nowMinute;
        valueForDisplay[register3Position] = (nowSecond * 100);
        setDigits();
        delay(200);
        ledControl.clearDisplay(2);
        delay(50);
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

        valueForDisplay[register1Position] = nowHour;
        valueForDisplay[register2Position] = nowMinute;
        valueForDisplay[register3Position] = (nowSecond *100);
        setDigits();
        delay(200);
        ledControl.clearDisplay(3);
        delay(50);
    }
    realTimeClock.adjust(DateTime(nowYear, nowMonth, nowDay, nowHour, nowMinute, nowSecond));
    action = displayRealTimeClock;
    setDigits(0, 0, 1);
    setDigits(0, 1, 6);
    verb = verbDisplayDecimal;
    verbOld[0] = 1;
    verbOld[1] = 6;
}

void actionSetDate() {
    byte yearToSet[4];
    byte monthToSet[2];
    byte dayToSet[2];
    byte hourToSet[2];
    byte minuteToSet[2];
    byte secondToSet[2];

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


void flashUplinkAndComputerActivityRandomly() {
    int randomNumber = random(10, 30);

    if ((randomNumber == 15) || (randomNumber == 25)) {
        illuminateWithRGBAndLampNumber(0, 150, 0, lampCompActy);
    }
    else {
        turnOffLampNumber(lampCompActy);
    }

    if ((randomNumber == 17) || (randomNumber == 25)) {
        illuminateWithRGBAndLampNumber(90, 90, 90, lampUplinkActy);
    }
    else {
        turnOffLampNumber(lampUplinkActy);
    }
}







void readIMU() {
    flashUplinkAndComputerActivityRandomly();

    Wire.beginTransmission(MPU_addr);
    Wire.write(0x3B);  // starting with register 0x3B (ACCEL_XOUT_H)
    Wire.endTransmission(false);
    Wire.requestFrom(MPU_addr,14,true);  // request a total of 14 registers

    valueForDisplay[0] = (Wire.read() << 8) | Wire.read();  // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)
    valueForDisplay[1] = (Wire.read() << 8) | Wire.read();  // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
    valueForDisplay[2] = (Wire.read() << 8) | Wire.read();  // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
    valueForDisplay[3] = (Wire.read() << 8) | Wire.read();  // 0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)
    valueForDisplay[register1Position] = (Wire.read() << 8) | Wire.read();  // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
    valueForDisplay[register2Position] = (Wire.read() << 8) | Wire.read();  // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
    valueForDisplay[register3Position] = (Wire.read() << 8) | Wire.read();  // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)
    valueForDisplay[3] = (valueForDisplay[3] / 340.00 + 36.53); //equation for temperature in degrees C from datasheet

    /* Serial.print("AcX = "); Serial.print(valueForDisplay[0]);
     Serial.print(" | AcY = "); Serial.print(valueForDisplay[1]);
     Serial.print(" | AcZ = "); Serial.print(valueForDisplay[2]);
     Serial.print(" | Tmp = "); Serial.print(valueForDisplay[3]);
     Serial.print(" | GyX = "); Serial.print(valueForDisplay[4]);
     Serial.print(" | GyY = "); Serial.print(valueForDisplay[5]);
     Serial.print(" | GyZ = "); Serial.println(valueForDisplay[6]);
     */

    setDigits();
}

void actionReadIMU() {
    readIMU();
}


void playTrack(uint8_t track) {
   myDFPlayer.stop();
   delay(200);
   myDFPlayer.play(track);
   delay(200);
   int file = myDFPlayer.readCurrentFileNumber();

   Serial.print("Track:");Serial.println(track);
   Serial.print("File:");Serial.println(file);

   while (file != track) {
     myDFPlayer.play(track);
     delay(200);
     file = myDFPlayer.readCurrentFileNumber();
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

void loop() {
    timer.tick(); // toggle on / off
    if (toggle == true) {
        setLamp(white, lampClk);
    }
    else
    {
        setLamp(off, lampClk);
    }
    
    if (currentProgram == programJFKAudio) {
        //jfk(1);
        playTrack(19);
        currentProgram = programNone;
        if (currentProgram == 0)
        {
            ledControl.setRow(0, 2, 0);
            ledControl.setRow(0, 3, 0);
        }
    }
    else if (currentProgram == programApollo11Audio) {
        //jfk(2);
        playTrack(20);
        currentProgram = programNone;
        if (currentProgram == 0)
        {
            ledControl.setRow(0, 2, 0);
            ledControl.setRow(0, 3, 0);
        }
    }
    else if (currentProgram == programApollo13Audio) {
        //jfk(3);
        playTrack(21);
        currentProgram = programNone;
        if (currentProgram == 0)
        {
            ledControl.setRow(0, 2, 0);
            ledControl.setRow(0, 3, 0);
        }
    }

    if (mode == modeIdle) {
        executeIdleMode();
        setLamp(white, lampSTBY);
        //setLamp(green, lampVerb);
        //setLamp(green, lampNoun);
        //setLamp(green, lampProg);
    }
    else if (mode == modeInputVerb) {
        executeVerbInputMode();
        setLamp(off, lampSTBY);
        //setLamp(orange, lampVerb);

    }
    else if (mode == modeInputNoun) {
        executeNounInputMode();
        setLamp(off, lampSTBY);
        //setLamp(orange, lampNoun);
    }
    else if (mode == modeInputProgram) {
        executeProgramInputMode();
        setLamp(off, lampSTBY);
        //setLamp(orange, lampProg);
    }
    else if (mode == modeLampTest) {
        executeLampTestModeWithDuration(5000);
    }
    
    if (action == displayIMUAttitude) {
        actionReadIMU();  // V16N17 ReadIMU
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

    //Serial.print(verb);
    //Serial.print("  ");
    //Serial.print(noun);
    //Serial.print("  ");
    //Serial.println(action);
}