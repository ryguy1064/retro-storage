#pragma once

#if defined(__AVR_ATmega328P__)

// -------------------------------  Pin assignments for Arduino UNO/Nano/Pro Mini (Atmega328p)  ------------------------------

#define PIN_STEP       2  // can be changed to different pin
#define PIN_STEPDIR    3  // can be changed to different pin
#define PIN_MOTORA     4  // can be changed to different pin
#define PIN_SELECTA    5  // can be changed to different pin
#define PIN_SIDE       6  // can be changed to different pin
#define PIN_INDEX      7  // accesses via IDXPORT/IDXBIT #defines below
#define PIN_READDATA   8  // must be pin 8 (ICP for timer1)
#define PIN_WRITEDATA  9  // must be pin 9 (OCP for timer1)
#define PIN_WRITEGATE 10  // accessed via WGPORT/WGBIT #defines below
#define PIN_TRACK0    11  // can be changed to different pin
#define PIN_WRITEPROT 12  // can be changed to different pin or commented out
#define PIN_DENSITY   13  // can be changed to different pin or commented out
// #define PIN_MOTORB    A0  // can be changed to different pin or commented out (together with PIN_SELECTB)
// #define PIN_SELECTB   A1  // can be changed to different pin or commented out (together with PIN_MOTORB)

#elif defined(__AVR_ATmega32U4__)

// -----------------------  Pin assignments for Arduino Leonardo/Micro (Atmega32U4)  --------------------------

#define PIN_STEP       2  // can be changed to different pin
#define PIN_STEPDIR    3  // can be changed to different pin
#define PIN_READDATA   4  // must be pin 4 (ICP for timer1)
#define PIN_MOTORA     5  // can be changed to different pin
#define PIN_SELECTA    6  // can be changed to different pin
#define PIN_SIDE       7  // can be changed to different pin
#define PIN_INDEX      8  // accesses via IDXPORT/IDXBIT #defines below
#define PIN_WRITEDATA  9  // must be pin 9 (OCP for timer1)
#define PIN_WRITEGATE 10  // accessed via WGPORT/WGBIT #defines below
#if defined(ARDUINO_AVR_LEONARDO)
#define PIN_TRACK0    11  // can be changed to different pin
#define PIN_WRITEPROT 12  // can be changed to different pin or commented out
#define PIN_DENSITY   13  // can be changed to different pin or commented out
#else
#define PIN_TRACK0    14  // can be changed to different pin
#define PIN_WRITEPROT 15  // can be changed to different pin or commented out
#define PIN_DENSITY   16  // can be changed to different pin or commented out
#endif
#define PIN_MOTORB    A0  // can be changed to different pin or commented out (together with PIN_SELECTB)
#define PIN_SELECTB   A1  // can be changed to different pin or commented out (together with PIN_MOTORB)

#elif defined(__AVR_ATmega2560__)

// ------------------------------  Pin assignments for Arduino Mega (Atmega2560)  -----------------------------

#define PIN_STEP      8   // can be changed to different pin
#define PIN_STEPDIR   7   // can be changed to different pin
#define PIN_MOTORA    6   // can be changed to different pin
#define PIN_SELECTA   5   // can be changed to different pin
#define PIN_SIDE      49  // can be changed to different pin
#define PIN_INDEX     47  // accessed via IDXPORT/IDXBIT #defines below
#define PIN_READDATA  48  // must be pin 48 (ICP for timer5)
#define PIN_WRITEDATA 46  // must be pin 46 (OCP for timer5)
#define PIN_WRITEGATE 45  // accessed via WGPORT/WGBIT #defines below
#define PIN_TRACK0    44  // can be changed to different pin
#define PIN_WRITEPROT 43  // can be changed to different pin or commented out
#define PIN_DENSITY   42  // can be changed to different pin or commented out
#define PIN_MOTORB    41  // can be changed to different pin or commented out (together with PIN_SELECTB)
#define PIN_SELECTB   40  // can be changed to different pin or commented out (together with PIN_MOTORB)

#else

#error "ArduinoFDC library requires either an ATMega328P, Atmega32U4 or ATMega2560 processor (Arduino UNO, Leonardo or MEGA)"

#endif