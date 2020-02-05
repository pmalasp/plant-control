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
 
#include <OneWire.h>
#include "utils.h"



/*
 * Watchdog setup
 */
 
void watchdog_setup() 
{

  NRF_WDT->CONFIG = (WDT_CONFIG_HALT_Pause << WDT_CONFIG_HALT_Pos) | ( WDT_CONFIG_SLEEP_Run << WDT_CONFIG_SLEEP_Pos);   //Configure Watchdog. a) Pause watchdog while the CPU is halted by the debugger.  b) Keep the watchdog running while the CPU is sleeping.
  NRF_WDT->CRV = 10*32768;             //ca 3 sek. timout
  NRF_WDT->RREN |= WDT_RREN_RR0_Msk;  //Enable reload register 0
  NRF_WDT->TASKS_START = 1;           //Start the Watchdog timer
}


/*
 * Watchdog reset
 */
 
void watchdog_reset() 
{
  NRF_WDT->RR[0] = WDT_RR_RR_Reload;  //Reload watchdog register 0
}




/*
 * System wide 1 second, minute, hour timer
 */
unsigned long one_ts = 0;
bool          one_second_elapsed  = false;
bool          one_minute_elapsed  = false;
bool          one_hour_elapsed    = false;
int           one_second_tick     = 0;
int           one_minute_tick     = 0;

void sys_one_second_run(){

  if (one_second_elapsed) {                 // active only for 1 loop cycle
    one_second_elapsed = false;
  
    if (one_minute_elapsed) {                 // active only for 1 loop cycle
      one_minute_elapsed = false;

      if (one_hour_elapsed) {                 // active only for 1 loop cycle
        one_hour_elapsed = false;
      }  
    }  
  }  
  
  if ((millis() - one_ts) >= 1000) { // time for the next run
    one_ts = millis();
    one_second_elapsed = true;  

    one_second_tick++;
    if (one_second_tick == 60){           // one minute gone
      one_second_tick = 0;
      one_minute_elapsed = true;

      one_minute_tick++;
      if (one_minute_tick == 60){         // one hour gone
        one_minute_tick = 0;
        one_hour_elapsed = true;
        
      }
    }
  }

}

bool sys_one_second(){  return one_second_elapsed; }
bool sys_one_minute(){  return one_minute_elapsed; }
bool sys_one_hour()  {  return one_hour_elapsed;   }


/*
 * Scan for 1 wire devices
 */
void One_Wire_Scan(int pin)
{

  OneWire ow(pin);

  uint8_t address[8];
  uint8_t count = 0;


  if (ow.search(address))
  {
    Serial.print("\nuint8_t pin");
    Serial.print(pin, DEC);
    Serial.println("[][8] = {");
    do {
      count++;
      Serial.println("  {");
      for (uint8_t i = 0; i < 8; i++)
      {
        Serial.print("0x");
        if (address[i] < 0x10) Serial.print("0");
        Serial.print(address[i], HEX);
        if (i < 7) Serial.print(", ");
      }
      Serial.println("  },");
    } while (ow.search(address));

    Serial.println("};");
    Serial.print("// nr devices found: ");
    Serial.println(count);
  }

}



/*
 * Scan I2C for connected devices
 */
void I2C_Scan(byte address_from, byte address_to)
{
  byte error, address;
  int nDevices;

  //Wire.begin();

  Serial.println("Scanning...");

  nDevices = 0;
  for(address = address_from; address < address_to; address++ )
  {
    // The i2c_scanner uses the return value of
    // the Write.endTransmisstion to see if
    // a device did acknowledge to the address.
    Wire.beginTransmission(address);
    error = Wire.endTransmission();

    if (error == 0)
    {
      Serial.print("I2C device found at address 0x");
      if (address<16)
        Serial.print("0");
      Serial.print(address,HEX);
      Serial.println("  !");

      nDevices++;
    }
    else if (error==4)
    {
      Serial.print("Unknown error at address 0x");
      if (address<16)
        Serial.print("0");
      Serial.println(address,HEX);
    }
  }
  if (nDevices == 0)
    Serial.println("No I2C devices found\n");
  else
    Serial.println("done\n");

  //Wire.end();

  // delay(5000);           // wait 5 seconds for next scan
}


/*
 * print value divided by 100
 */
void print_100( int16_t val){
  int16_t val_int = val / 100;
  
  Serial.print(val_int);                // print the integer part
  Serial.print(".");

  val_int = abs(val - val_int * 100);   // get the fractional part
  if (val_int < 10) Serial.print('0');  // insert a 0 if needed
  Serial.print(val_int);                // print the fractional part

}

/*
 * print timestamp
 */
void print_timestamp(){
  print_timestamp(now());
}

/*
 * print start time 
 */
void print_timestamp(time_t my_t){
  
  if (hour(my_t) < 10) Serial.print('0');
  Serial.print( hour(my_t) );
  Serial.print(":");
  if (minute(my_t) < 10) Serial.print('0');
  Serial.print(minute(my_t));
  Serial.print(":");
  if (second(my_t) < 10) Serial.print('0'); 
  Serial.print(second(my_t));


}

void print_timestamp(Adafruit_Microbit_BLESerial &ble_serial){
  print_timestamp(ble_serial, now());
}

/*
 * print start time 
 */
void print_timestamp(Adafruit_Microbit_BLESerial &ble_serial, time_t my_t){
  
  if (hour(my_t) < 10) ble_serial.print('0');
  ble_serial.print( hour(my_t) );
  ble_serial.print(":");
  if (minute(my_t) < 10) ble_serial.print('0');
  ble_serial.print(minute(my_t));
  ble_serial.print(":");
  if (second(my_t) < 10) ble_serial.print('0'); 
  ble_serial.print(second(my_t));


}


/*
 * print date
 */
void print_datestamp(){
  print_datestamp(now());
}

/*
 * print date 
 */
void print_datestamp(time_t my_t){
  

  if (day(my_t) < 10) Serial.print('0');
  Serial.print(day(my_t));
  Serial.print( "/");
  
  if (month(my_t) < 10) Serial.print('0');
  Serial.print(month(my_t));
  Serial.print( "/");

  if (year(my_t) < 1000) Serial.print('0');
  if (year(my_t) < 100 ) Serial.print('0');
  if (year(my_t) < 10  ) Serial.print('0');
  Serial.print(year(my_t) );
 
}


/*
 * read command line from BLE
 */
int get_Command(char *input_command, Adafruit_Microbit_BLESerial &ble_serial)
{
  static int  index_command = 0;
  int         return_code;
  
  // parse the serial input and perform actions
  if (ble_serial.available() )  {
    
    // read the first character from the serial buffer
    input_command[index_command] = ble_serial.read();
    //ble_serial.println(input_command[index_command]);   // DEBUG

    if ((input_command[index_command] == '\r' ) | (input_command[index_command] == '\n' ) )
    {
      input_command[index_command] = '\0'; // terminate the string
      ble_serial.println(input_command);   // ECHO
      index_command = 0;                   // restart from scratch
      return_code =  (0);                          // return reading success
    } else {
      // increase the index
      index_command++;

      // check the buffer overflow
      if (index_command == SERIAL_CMD_LENGHT)
      {
        index_command = 0;                  // restart from scratch
        return_code =  (-1);                        // mark the overflow error ....         Serial.println("Buffer Overflow, please restart");
      } else {
        return_code =  (-2);             // command read ongoing
      }
    }
  } else {
    return_code =  (-3) ; // nothing to read ...
  }
  
  return (return_code);  
}

/*
 * read command line from Serial
 */
int get_Command(char *input_command)
{
  static int  index_command  = 0;
  int         return_code;

  // parse the serial input and perform actions
  if (Serial.available() )  {
    
    // read the first character from the serial buffer
    input_command[index_command] = Serial.read();

    if ((input_command[index_command] == '\r' ) | (input_command[index_command] == '\n' ) )
    {
      input_command[index_command] = '\0'; // terminate the string
      Serial.println(input_command);
      index_command = 0;                   // restart from scratch
      return_code = (0);                   // return reading success
    } else {
      // Serial.println(input_command[index_command]); // DEBUG
      // increase the index
      index_command++;

      // check the buffer overflow
      if (index_command == SERIAL_CMD_LENGHT)
      {
        index_command = 0;               // restart from scratch
        return_code =  (-1);             // mark the overflow error ....         Serial.println("Buffer Overflow, please restart");
      } else {
        return_code =  (-2);             // command read ongoing
      }
    }
  } else {
    return_code = (-3) ; // nothing to read ...
  }

  return (return_code);
}
