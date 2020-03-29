# opendskyrevised-2020-03-24
Based on rondiamonds openDSKY (https://github.com/rondiamond/OpenDSKY)

Open DSKY is a Kickstarter project from S&T Geotronics.
It offers a 3D-printed replica of the Display & Keyboard (DSKY) used in the Apollo Command and Lunar Modules.
The design is custom hardware, based on an Arduino Nano. It does not run the original Apollo flight code. Instead, it implements a small subset of functions, such as displaying the time, GPS coordinates, etc., and can play several selected sound bites through an internal speaker.

This project is a fork of rondiamonds openDSKY (https://github.com/rondiamond/OpenDSKY) which is based on S&T's original Arduino code

Refactoring of the original code, to make it more readable and extensible
Possible additional future functionality
Minimal external dependencies

The following Verbs and Nouns are currentliy implemented:

V16 N36 - Display current Time
V16 N43 - Read and Display the GPS Coordinates
V16 N17 - Read and Display Accellerometer Angles
V16 N18 - Read and Display Gyro Angles
v16 N98 - Play Selceted Audio Clip
V35     - Lamptest

V21 N36 - Set Time
v21 N37 - Set Date
V21 N98 - Select the Audioclip to play

Note that the Audio only works if you connect D4 & D5 of the Arduino to the RX and TX Pin of the DF Audoplayer
(https://wiki.dfrobot.com/DFPlayer_Mini_SKU_DFR0299)

P69     - Play JFK

This project is NOT affiliated with S&T Geotronics. It is offered as open-source under the MIT License.

For questions regarding the OpenDSKY project, hardware and its original software, please contact: https://www.stgeotronics.com

I use PlatformIO and Visual Studio Code, but the files should flash when using the Arduino IDE if the Plugins are present:
  Adafruit NeoPixel
  LedControl
  RTClib
  arduino-timer
  DFRobotDFPlayerMini
  TinyGPSPlus
  
 
