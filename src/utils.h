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

#ifndef _PC_UTILS_
#define _PC_UTILS_



#include <Arduino.h>
#include <nrf_soc.h>
#include <HardwareSerial.h>
#include <Wire.h>
#include <Time.h>
#include <Adafruit_Microbit.h>
#include "menu.h"



/*
 * system timer function
 */
void sys_one_second_run();  // to be called every loop cycle
bool sys_one_second();      // active for ony one loop cycle
bool sys_one_minute();      // active for ony one loop cycle
bool sys_one_hour();      // active for ony one loop cycle

/*
 * watchdog setup function - it is a sw emulation because of a platform issue
 */
void watchdog_setup();
void watchdog_reset();


/*
 * scan the I2C bus
 */
void I2C_Scan(byte address_from, byte address_to);

/*
 * Scan for 1 wire devices
 */
void One_Wire_Scan(int pin);


/*
 * print functions
 */
void print_100( int16_t val);
void print_timestamp();
void print_timestamp(time_t my_t);
void print_datestamp();
void print_datestamp(time_t my_t);


void print_timestamp(Adafruit_Microbit_BLESerial &ble_serial);
void print_timestamp(Adafruit_Microbit_BLESerial &ble_serial, time_t my_t);
/*
 * read command line from Serial
 */
int get_Command(char *input_command); // 0 success, -1 overflow, everything else running ...
int get_Command(char *input_command, Adafruit_Microbit_BLESerial &ble_serial);




#endif
