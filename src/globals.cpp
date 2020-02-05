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
#include <HardwareSerial.h>
#include <Time.h>
#include <Adafruit_Microbit.h>
#include <DS3232RTC.h>
#include <AT24CX.h>
#include "utils.h"
#include "globals.h"

#define PROGRAM_VERSION "1.5.3"
#define BUFFER_DISPLAY_LENGHT 40
#define REFRESH 200
#define TRG 120

// Microbit instance
Adafruit_Microbit microbit;

// EEPROM MAP
char  s_DeviceName[20];   //   0 ...  19 Device Name
char  s_LocalName[20];    //  20 ...  39 Local Name
char  s_Welcome[40];      //  40 ...  79 Welcome message
time_t    start_ON;       //  80 ...  83 Start time (unsigned long)
uint16_t  min_ON;         //  84 ...  85 Min time (int)
uint16_t  max_ON;         //  86 ...  87 Max time (int)
int16_t   minTemp;        //  88 ...  91 min Temperature (float)
int16_t   maxTemp;        //  92 ...  95 max Temperature (float)
uint16_t  my_disp_on;     //  96 ...  97 display status

                          // .... reserved
                          // 100 ... 772   Store last week hourly temperature (7 * 24 * sizeof(float))

//temperatures

int16_t         _temperatures[3];
int16_t         ds_temp_24[24];


// remaining time in a watering cycle
uint16_t        elapse_ON = 0;

// state for FSM
int             fsm_watering;

// EEPROM Instance
AT24C32         storage(0x07);

// RTC instance
DS3232RTC RTC;


// string for showing the halt condition
char s_Halt[] = "----";
