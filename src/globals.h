#ifndef _GLOBALS_
#define _GLOBALS_

/*
 * 
 *  2020 By Paolo Malaspina,  https://www.linkedin.com/in/paolomalaspina/
 *
 *
 *  Permission is hereby granted, free of charge, to any person obtaining a copy
 *  of this software and associated documentation files (the "Software"), to deal
 *  in the Software without restriction, including without limitation the rights
 *  to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 *  copies of the Software, and to permit persons to whom the Software is
 *  furnished to do so, subject to the following conditions:
 *
 *  The above copyright notice and this permission notice shall be included in
 *  all copies or substantial portions of the Software.
 *
 *  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 *  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 *  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 *  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 *  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 *  OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 *  THE SOFTWARE.
 *
 */

#include <Arduino.h>
#include <nrf_soc.h>
#include <Time.h>
#include <HardwareSerial.h>
#include <Adafruit_Microbit.h>
#include <DS3232RTC.h>
#include <AT24CX.h>
#include "utils.h"


#define PROGRAM_VERSION "1.5.3"

#define         die_temp  _temperatures[2]
#define         rtc_temp  _temperatures[1]
#define         ds_temp   _temperatures[0]

// actuator finite state machine
#define         FSM_IDLE       0
#define         FSM_START     10
#define         FSM_WATERING  11
#define         FSM_STOP      12

// Microbit instance
extern Adafruit_Microbit    microbit;

// EEPROM MAP
extern char  s_DeviceName[20];   //   0 ...  19 Device Name
extern char  s_LocalName[20];    //  20 ...  39 Local Name
extern char  s_Welcome[40];      //  40 ...  79 Welcome message
extern time_t    start_ON;       //  80 ...  83 Start time (unsigned long)
extern uint16_t  min_ON;         //  84 ...  85 Min time (int)
extern uint16_t  max_ON;         //  86 ...  87 Max time (int)
extern int16_t   minTemp;        //  88 ...  91 min Temperature (float)
extern int16_t   maxTemp;        //  92 ...  95 max Temperature (float)
extern uint16_t  my_disp_on;     //  96 ...  97 display status

                          // .... reserved
                          // 100 ... 772   Store last week hourly temperature (7 * 24 * sizeof(float))

//temperatures

extern int16_t         _temperatures[3];
extern int16_t         ds_temp_24[24];


// remaining time in a watering cycle
extern uint16_t  elapse_ON;

// state for FSM
extern int       fsm_watering;

// EEPROM Instance
extern AT24C32  storage;
// RTC instance
extern DS3232RTC RTC;

extern char     s_Halt[];

#endif
