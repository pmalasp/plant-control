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
#include <Adafruit_Microbit.h>
#include "menu.h"

/*
 * COMMAND tokens definition
 */
D_TK(i2c);
D_TK(1w);
D_TK(scan);
D_TK(status);
D_TK(prog);
D_TK(start);
D_TK(stop);
D_TK(halt);
D_TK(min);
D_TK(max);
D_TK(time);
D_TK(date);
D_TK(temp);
D_TK(name);
D_TK(lname);
D_TK(welcome);
D_TK(sys);
D_TK(log);
D_TK(off);
D_TK(on);
D_TK(24h);
D_TK(disp);
D_TK(version);
D_TK(help);
// D_TK(aaa) expanded in const char TK_aaa[] = "aaa";


const S_MENU_T m_scan[] = {
  {TK(1w),  CMD_1W_SCAN, NULL},
  {TK(i2c), CMD_I2C_SCAN, NULL},
  {NULL,     NO_CMD,    NULL},
};

const S_MENU_T m_log[] = {
  {TK(on),  CMD_LOG_ON, NULL},
  {TK(off), CMD_LOG_OFF, NULL},
  {TK(24h), CMD_LOG_24H, NULL},
  {NULL,     NO_CMD,    NULL},
};

const S_MENU_T m_disp[] = {
  {TK(on),  CMD_SYS_DSP_ON, NULL},
  {TK(off), CMD_SYS_DSP_OFF, NULL},
  {NULL,     NO_CMD,    NULL},
};

const S_MENU_T m_sys[] = {
  {TK(time),  CMD_SET_TIME,   NULL},
  {TK(date),  CMD_SET_DATE,   NULL},
  {TK(start), CMD_SYS_START,  NULL},
  {TK(stop),  CMD_SYS_STOP,   NULL},
  {TK(halt),  CMD_HALT,       NULL},
  {TK(log),   NO_CMD,         m_log},
  {TK(disp),  NO_CMD,         m_disp},
  {TK(scan),  NO_CMD,         m_scan},
  {NULL,     NO_CMD,    NULL},

};

const S_MENU_T m_prog_time[] = {
  {TK(min),     CMD_PROG_MIN_TIME,    NULL},
  {TK(max),     CMD_PROG_MAX_TIME,    NULL},
  {NULL,     NO_CMD,    NULL},  
};

const S_MENU_T m_prog_temp[] = {
  {TK(min),     CMD_PROG_MIN_TEMP,    NULL},
  {TK(max),     CMD_PROG_MAX_TEMP,    NULL},
  {NULL,     NO_CMD,    NULL},  
};

const S_MENU_T m_prog[] = {
  {TK(name),     CMD_PROG_NAME,    NULL},
  {TK(lname),    CMD_PROG_LNAME,   NULL},
  {TK(welcome),  CMD_PROG_WELCOME, NULL},
  {TK(start),    CMD_PROG_START,   NULL},
  {TK(temp),     NO_CMD,          m_prog_temp},  
  {TK(time),     NO_CMD,          m_prog_time},  
  {NULL,     NO_CMD,    NULL},  
};

const S_MENU_T m_main[] = {
  {TK(version),  CMD_VERSION,       NULL},
  {TK(status),   CMD_STATUS_PRINT,  NULL},
  {TK(help),     CMD_HELP,          NULL},
  {TK(sys),      NO_CMD,            m_sys},
  {TK(prog),     NO_CMD,            m_prog},
  {NULL,         NO_CMD,            NULL},
  
};


void parse_Command::print_menu(const S_MENU_T* menu, int indent ){
  
  // go through the list of commands
  while(menu->token != NULL) {
    for (int i = 0; i < indent ; i++) Serial.print("  "); // indent with double space 
    Serial.println(menu->token);
    
    if (menu->left != NULL ) {  // recursively check if there is a submenu
      print_menu(menu->left, (indent + 1 ) );
    }
    menu++;                     // scan the array
  }

}

void parse_Command::print_menu(const S_MENU_T* menu, int indent, Adafruit_Microbit_BLESerial &ble_serial ){
  
  // go through the list of commands
  while(menu->token != NULL) {
    ble_serial.poll();
    
    for (int i = 0; i < indent ; i++) ble_serial.print("  "); // indent with double space 
    ble_serial.println(menu->token);
    
    if (menu->left != NULL ) {  // recursively check if there is a submenu
      print_menu(menu->left, (indent + 1 ), ble_serial );
    }
    menu++;                     // scan the array
  }

}



/*
 * Print the command structure
 */
void parse_Command::help()
{
  print_menu( m_main, 0);
}

void parse_Command::help(Adafruit_Microbit_BLESerial &ble_serial)
{
  print_menu( m_main, 0, ble_serial);
}


/*
 * process the serial communicatioon
 */
int parse_Command::parse()
{
  const S_MENU_T* menu_ptr = m_main;
  
  // get the first token of the command
  input_token = strtok(input_command, " ");
  
  // go through the list of commands
  while(menu_ptr->token != NULL) {
    if (!strcmp(input_token, menu_ptr->token) ) {  // check if the token text is equal to the input token
      if ( menu_ptr->code != NO_CMD) {             // command found, no submenu available
         return ( menu_ptr->code );
      } else {                                     // token found, submenu available
        input_token = strtok(NULL, " ");           // get the next input token
        menu_ptr = menu_ptr->left;                 // go to submenu
      }
      
    } else {                                       // token not found, try the next element
      menu_ptr++;
    }
  }
  return ( CMD_ERROR );                            // no command found, return an error
}


int   parse_Command::parse_int(const char *separator)
{
  input_token = strtok(NULL, separator);
  return (atoi(input_token));
}

float   parse_Command::parse_float(const char *separator)
{
  input_token = strtok(NULL, separator);
  return (atof(input_token));
}

char*   parse_Command::parse_string(const char *separator)
{
  input_token = strtok(NULL, separator);
  return (input_token);
}
