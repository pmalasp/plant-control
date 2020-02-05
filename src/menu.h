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
 
#ifndef _PC_MENU_
#define _PC_MENU_


#include <Arduino.h>
#include <nrf_soc.h>
#include <HardwareSerial.h>
#include <Adafruit_Microbit.h>
#include "globals.h"


#define SERIAL_CMD_LENGHT     80


typedef enum S_CODE {
CMD_ERROR,
NO_CMD,
CMD_PROG_START,
CMD_PROG_MAX_TIME,
CMD_PROG_MIN_TIME,
CMD_PROG_MAX_TEMP,
CMD_PROG_MIN_TEMP,
CMD_PROG_NAME,
CMD_PROG_LNAME,
CMD_PROG_WELCOME,
CMD_SET_TIME,
CMD_SET_DATE,
CMD_LOG_ON,
CMD_LOG_OFF,
CMD_LOG_24H,
CMD_I2C_SCAN,
CMD_1W_SCAN,
CMD_SYS_DSP_ON,
CMD_SYS_DSP_OFF,
CMD_SYS_START,
CMD_SYS_STOP,
CMD_HALT,
CMD_VERSION,
CMD_HELP,
CMD_STATUS_PRINT,
} S_CODE_T;


// D_TK(aaa) expanded in const char TK_aaa[] = "aaa";
#define D_TK(S)  const char TK_##S[] = #S
#define TK(S)               TK_##S


typedef struct S_MENU {
  const char*                 token;
  const S_CODE_T              code;
  const struct S_MENU* const  left;
} S_MENU_T;

class parse_Command {
  public:
    
    void  help();    
    void  help(Adafruit_Microbit_BLESerial &ble_serial);
    int   parse();
    int   parse_int   (const char *separator);
    float parse_float (const char *separator);
    char *parse_string(const char *separator);

    char  input_command[SERIAL_CMD_LENGHT];
    
  private:
    void  print_menu(const S_MENU_T* menu, int indent );
    void  print_menu(const S_MENU_T* menu, int indent, Adafruit_Microbit_BLESerial &ble_serial );
    char *input_token;
    
};


#endif
