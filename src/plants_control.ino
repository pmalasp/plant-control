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



#include <HardwareSerial.h>
#include <Wire.h>
#include <OneWire.h>
#include <Time.h>
#include <DS3232RTC.h>
#include <MMA8653.h>
#include <nrf_soc.h>
#include <Adafruit_Microbit.h>
#include <AT24CX.h>
#include <DallasTemperature.h>
#include "utils.h"
#include "globals.h"
#include "ble_comm.h"


#define PROGRAM_VERSION "1.5.1"
#define BUFFER_DISPLAY_LENGHT 40
#define REFRESH 200
#define TRG 120


// string for the LED output
char  buffer_LED_Display[BUFFER_DISPLAY_LENGHT]; 
int   pointer_LED_Display = 4;


// Temperature Sensor
#define ONE_WIRE_PIN        16

OneWire           OW_bus(ONE_WIRE_PIN);
DallasTemperature MyDs18b20(&OW_bus);
DeviceAddress     AddrDs18b20;


// Output controller
#define WATERING_PIN  1




bool my_read_on = false;



// command parsers for serial and BLE
parse_Command    serial_cmd;

  
void setup() {
  
  Serial.begin(57600);
  Wire.begin();

  RTC.begin();

  time_t my_t = RTC.get();
  
  if (my_t != 0) {
    Serial.println("RTC has set the system time");
    setTime(my_t);
  } else {
    Serial.println("Unable to sync with the RTC");
  }

  init_Storage();  
  init_BLE();
  init_LED_Display();
  init_Button_in();

  init_Sensors();
  init_Actuators();
  
 
  watchdog_setup(); // start the watchdog
}



void loop() {
  
  sys_one_second_run();   // run the system wide 1 second timer

  process_Serial();       // check the serial command

  process_BLE();          // Bluetooth
  
  process_LED_Display();  // Update LED display
  
  process_Button_in();    // Check Button display

  process_Sensors();      // Sensors
  process_Actuators();    // Actuators

  // RESET the watchdog
  watchdog_reset();


}





/*
 * retrieve stored variables
 */
void init_Storage()
{
  // Wire.begin();

  
  
  storage.readChars(  0, s_DeviceName, 20);
  storage.readChars( 20, s_LocalName,   20);
  storage.readChars( 40, s_Welcome, BUFFER_DISPLAY_LENGHT);
  strcpy(buffer_LED_Display, s_Welcome);    // display the message

  start_ON = (time_t) storage.readLong( 80);        // 80 ... 83 Start time (unsigned long)
  min_ON  = (uint16_t)storage.readInt(84);          // 84 ... 85 Min time (int)
  max_ON  = (uint16_t)storage.readInt(86);          // 86 ... 87 Max time (int)
  minTemp = (int16_t) storage.readInt(88);          //  88 ...  91 min Temperature * 100
  maxTemp = (int16_t) storage.readInt(92);          //  92 ...  95 max Temperature * 100
  my_disp_on  = (uint16_t)storage.readInt(96);      //  96 ...  97 display status

  int h, i, j;
  h = hour();
  for(i = 0; i < 24; i++){
    j = h - i;
    if ( j < 0 ) j +=  24;    // loop through the array
    
    ds_temp_24[j] = temp_read_Storage(now() - i * SECS_PER_HOUR); // 100 .... historical values, 1 week
  }
  
  Serial.println(buffer_LED_Display);


}

/*
 * temperature_Storage
 * store the hourly value in EEPROM
 * 100 ... xxx   Store last week hourly temperature (7 * 24 * sizeof(int16_t))
 */
void temp_write_Storage(time_t my_t, int16_t my_temperature){
  unsigned int my_addr = 100 + (dayOfWeek(my_t)*24 + hour(my_t)  ) * sizeof(int16_t);

  /*/
  Serial.print("storage addr:");
  Serial.print(sizeof(float));
  Serial.print(", ");
  Serial.print(my_addr);
  Serial.print(", ");
  /*/

  // DEBUG --- do not write EEPROM
  storage.writeInt(my_addr, my_temperature);


}

/*
 * temperature_Storage
 * read the hourly value in EEPROM
 * 100 ... 772   Store last week hourly temperature (7 * 24 * sizeof(float))
 */
int16_t temp_read_Storage(time_t my_t){
  unsigned int my_addr = 100 + (dayOfWeek(my_t)*24 + hour(my_t)  ) * sizeof(int16_t) ;

  /*/
  Serial.print(dayOfWeek(my_t));
  Serial.print(", ");
  Serial.print("storage addr:");
  Serial.print(my_addr);
  Serial.print(", ");
  /*/

  return( (int16_t)storage.readInt(my_addr));


}



/*
 * INITIALIZE SENSORS
 */
void init_Sensors()
{

  MyDs18b20.begin();                      // Initialize

  MyDs18b20.getAddress(AddrDs18b20, 0);   // Retrieve the HW address of the sensor

  MyDs18b20.setResolution(AddrDs18b20, 12); // set the conversion resolution

  MyDs18b20.setWaitForConversion(false);  // makes it async
}

/*
 * process SENSORS
 */
void process_Sensors()
{
  static bool           die_read   = false;
  static long           ds_temp_buffer = 0;
  static long           ds_temp_counter = 0;
  
  int                   my_second; // second actually calls second(now()) and now() is a reasonably complex function 
  int                   my_hour;
  
  // try and read the die temperature, must be repeated a number of times before getting the results

  if (!die_read)
  {
    int32_t  temp;
    uint32_t err_code = sd_temp_get(&temp); 
    
  
    // Update the temperature
    if (err_code == 0) 
    {
          die_temp = (die_temp + (int16_t)temp * 100) / 5 ; // sd_temp_get(...) return the temperature  * 4, so to do the everage divide by 5 ... it is an IIR
          die_read = true; // lock the reading until the next cycle
    }
      
  }


  if (sys_one_second() ) //  1 second tick
  {
    my_second = second(); // second actually calls second(now()) and now() is a reasonably complex function
    
    if ( my_second  == 55)                        // request the start of the temperature readings 
    {  
      MyDs18b20.requestTemperatures();            // trigger the next measurement cycle
      die_read = false;                           // activate the die reading
    } 
    else if ( my_second  == 58)                        // read the temperatures 
    {  
      rtc_temp =  (int16_t) RTC.temperature() * 25 ;                // RTC.temperature() return the temperature  * 4 temperature measurement in DS3231 happen every 64 seconds
      ds_temp =   (100 * MyDs18b20.getTemp(AddrDs18b20) ) / 128;    // get the temperature*128 from the sensor, DEVICE_DISCONNECTED_RAW in case of error 
      
    } 
    else if (my_second == 0 )                           // set aside the temperature every minute
    { 
      ds_temp_buffer  += (long) ds_temp;                // add the read temperature to the buffer
      ds_temp_counter++;                                // increment the counter of the buffer (needed in case of a restart)
      
      // Serial.println("update ds_temp average"); 

      sprintf(buffer_LED_Display, "%2d.%2d *C", ds_temp/100, ds_temp%100); // Update display

      
      if (minute() == 0) // every hour evaluate the average of the buffer and store the temperatures 
      {
        my_hour = (int) hour();
        
        ds_temp_24[my_hour] = (int16_t) (ds_temp_buffer / ds_temp_counter);

        Serial.print("Store hourly average - samples: ");
        Serial.print(ds_temp_counter);
        Serial.print(" value: ");
        print_100( ds_temp_24[my_hour] );
        Serial.println(" *C ");

        temp_write_Storage(now(), ds_temp_24[my_hour]);// store the value in the EEPROM, keeping a 1 week history
        ds_temp_buffer  = 0;// reset the average buffer and the counter
        ds_temp_counter = 0;
         
      }
    }
  }

    
}



/*
 * INITIALIZE the output system
 */
void init_Actuators()
{
  // configure the output PIN
  pinMode(WATERING_PIN, OUTPUT);
  
  fsm_watering = FSM_IDLE;
}

/*
 * process the output system
 */
void process_Actuators()
{

  // print the values
  if (sys_one_second() ) 
  {  
     
    /* 
     * FSM states: 
     * - IDLE
     * - START
     * - WATERING
     */
  
     switch (fsm_watering){
      case FSM_IDLE:
        // check the start timer start_ON
        if (elapsedSecsToday(now() ) == start_ON )
        {
          Serial.println("Starting the watering cycle");

          fsm_watering = FSM_START;
        }
        break;
      case FSM_START: // the manual start (sent via BLE, serial or button) set thenstatus to FSM_START, so we jump here
  
        // evaluate temperature history
        int16_t my_temp[6];
        int16_t r_temp, t_temp;

        int i, j; // idexes for the sorting

        for( i = 0; i < 6; i++) my_temp[i] = -12800; // variable initialization to an out of sensor range value 
        for( i = 0; i < 24; i++){
          r_temp = ds_temp_24[i]; // get the temperature from the store
          //Serial.println(r_temp);

          for (j = 0; (j < i) && (j < 6); j++){ // scan the values already read to sorT the array
            
            if (r_temp > my_temp[j]){ // swap in case the new value is higher
              t_temp      = my_temp[j];
              my_temp[j]  = r_temp;
              r_temp      = t_temp;
            } 
          }

        }

        //
        Serial.println("Sorted array");
        for (j = 0; j < 6; j++) {
          Serial.print(my_temp[j]);
          Serial.print(", ");
        }
        Serial.println();
        Serial.print("75% percentile; ");
        Serial.print(my_temp[5]); // this correspond to the 75% percentile of the array
        //
        
        elapse_ON = constrain( (int) map(my_temp[5], minTemp, maxTemp, min_ON, max_ON  ), min_ON, max_ON); // linear evaluation with limits

        //
        Serial.print(", ON time; ");
        Serial.println(elapse_ON);
        //

        fsm_watering = FSM_WATERING;

        Serial.println("power ON the valve");// start watering
        digitalWrite(WATERING_PIN, HIGH);   // activate the actuator
  
        break;
      case FSM_WATERING:
        // check the end timer
        if ( elapse_ON == 0 )
        {
          Serial.println("Stop the watering cycle");

          fsm_watering = FSM_STOP;
        }
        else
          elapse_ON--; // function is running every second, so just decrease by 1
        break;
      case FSM_STOP: // manual or BLE stop will send here

        // stop watering
        Serial.println("power off the valve");

        // reset the on counter
        elapse_ON = 0;
        
        fsm_watering = FSM_IDLE;

        digitalWrite(WATERING_PIN, LOW);   // de-activate the actuator

        break;
     }

  }
}



/*
 * initialize the LED desplay update
 */
void init_LED_Display()
{

  // init the display
 microbit.matrix.begin();


  // Fill screen
  microbit.matrix.fillScreen(LED_ON);
  delay(1000);

  /* draw a yes check
  microbit.show(microbit.YES);
  delay(1000);

  microbit.print("Welcome");
  */
  
  microbit.matrix.clear();



}


/*
 * process the LED desplay update
 */
void process_LED_Display()
{
  static unsigned long ts_last = 0;
  
 
  if (my_disp_on) {
    if ((millis() - ts_last) >= 200)   // run every ... milliseconds
    {
      // increase the timestamp
      ts_last = millis();
  
      // Increment and rotate the position index
      pointer_LED_Display--;
      if ( pointer_LED_Display < ((int)strlen(buffer_LED_Display) * -5))
      {
        pointer_LED_Display = 4;
      }
  
      microbit.matrix.setFont(&TomThumb);
      microbit.matrix.setTextWrap(false);
      microbit.matrix.setTextColor(LED_ON);
      
      microbit.matrix.setCursor(pointer_LED_Display, 5);
      microbit.matrix.clear();
      microbit.matrix.Adafruit_GFX::print(buffer_LED_Display);
    }
    
  }
}


/*
 * initialize the buttons
 */
void init_Button_in()
{

  // se the button pins
  pinMode(PIN_BUTTON_A, INPUT);
  pinMode(PIN_BUTTON_B, INPUT);



}


/*
 * process the Button input
 */
void process_Button_in()
{
  static unsigned long a_ts = 0;
  static unsigned long b_ts = 0;

  static bool A_pressed = false;
  static bool B_pressed = false;

  /*
   * work on btn A
   */
  if (! A_pressed) {   
    if (! digitalRead(PIN_BUTTON_A)) {
      // Serial.println("Button A pressed");
      
      A_pressed = true;
      a_ts = millis();
    }
  } else {
    if (digitalRead(PIN_BUTTON_A)) {
      Serial.print("Button A released after ");
      Serial.print( millis()- a_ts);
      Serial.println(" ms.");
        
      A_pressed = false;

      // toggle the watering cycle
      if (fsm_watering == FSM_IDLE) 
        fsm_watering = FSM_START; // Start the watering cycle
      else 
        fsm_watering = FSM_STOP; // stop the watering cycle
      
    }
     
  }

  /*
   * work on btn B
   */
  if (! B_pressed) {   
    if (! digitalRead(PIN_BUTTON_B)) {
      // Serial.println("Button B pressed");
      
      B_pressed = true;
      b_ts = millis();
    }
  } else {
    if (digitalRead(PIN_BUTTON_B)) {
      Serial.print("Button B released after ");
      Serial.print( millis()- b_ts);
      Serial.println(" ms.");
        
      B_pressed = false;

      
    }
     
  }

  
}




/*
 * process the serial communication
 */
void process_Serial()
{
  int         i, j, result, cmd;
  tmElements_t my_t;
  time_t t ;

  // skip everything if serial is not connected
  if (!Serial) return;
  
  // run every 1000 milliseconds
  if (sys_one_second())
  {
        
    // print timestamp & data
    print_datestamp();
    Serial.print(" - ");
    print_timestamp();
    Serial.print(" - "); 
    
    if (my_read_on) 
    {
          // Serial.print("Chip temperature: ");
          print_100(die_temp);
          Serial.print( " *C [die], ");
  
          //Serial.print("RTC temperature: ");
          print_100(rtc_temp );
          Serial.print( " *C [rtc], ");
  
          //Serial.print("BMP temperature: ");
          print_100(ds_temp );
          Serial.print( " *C [ds]");
  
          if (fsm_watering == FSM_WATERING )
          {
            Serial.print( ", ON for ");
            Serial.print(elapse_ON );
            Serial.print( " s");
          }
      
    }
    // close the line
    Serial.println();
  
  }
  
  
  result = get_Command(serial_cmd.input_command); // get the command line, 0 is line completed
  if (result == 0 )
  {
    cmd = serial_cmd.parse();

    switch (cmd) 
    {
      case CMD_ERROR:
        Serial.println("command error");
        break;

      case CMD_VERSION:
        Serial.print("Version: ");
        Serial.println(PROGRAM_VERSION);
        Serial.print("Compile date: ");
        Serial.println( __DATE__);
        Serial.print("Compile time: ");
        Serial.println( __TIME__ );
        break;

      case CMD_HELP:
        serial_cmd.help();
        break;
      
      case CMD_I2C_SCAN:
        Serial.println("Scan I2C bus");
        I2C_Scan(0, 127);// check the I2C bus
        break;

      case CMD_1W_SCAN:
        Serial.println("Scan 1 Wire bus");
        One_Wire_Scan(ONE_WIRE_PIN);// check the I2C bus
        break;

      case CMD_STATUS_PRINT:
        /*/
        Serial.print("Now: ");
        print_datestamp();
        Serial.print(" - ");
        print_timestamp();
        Serial.println( );
        /*/

        Serial.print("Device Name: ");
        Serial.print(s_DeviceName);
        Serial.println( );

        Serial.print("Local Name: ");
        Serial.print(s_LocalName);
        Serial.println( );

        Serial.print("Message: ");
        Serial.print(buffer_LED_Display);
        Serial.println( );

        Serial.print("Start at: ");
        print_timestamp(start_ON);
        Serial.println( );

        Serial.print(" -   min: ");
        Serial.print(min_ON);
        Serial.print(" s - ");
        print_100(minTemp );
        Serial.println(" *C" );

        Serial.print(" -   MAX: ");
        Serial.print(max_ON);
        Serial.print(" s - ");
        print_100(maxTemp );
        Serial.println(" *C" );

        /*/
        Serial.print("Chip temperature: ");
        Serial.print(die_temp);
        Serial.println( " *C");

        Serial.print("RTC temperature: ");
        Serial.print(rtc_temp);
        Serial.println( " *C");

        Serial.print("BMP temperature: ");
        Serial.print(bmp_temp);
        Serial.println( " *C");

        Serial.print("BMP pressure : ");
        Serial.print(bmp_press);
        Serial.println( " Pa");
        /*/
        break;

      case CMD_PROG_START:
        // read the time
        my_t.Hour   = (uint8_t) serial_cmd.parse_int(":");
        my_t.Minute = (uint8_t) serial_cmd.parse_int(":");
        my_t.Second = (uint8_t) 0;
        // my_t.Second = (uint8_t) serial_cmd.parse_int("#");
        
        start_ON = my_t.Hour * SECS_PER_HOUR + my_t.Minute * SECS_PER_MIN + my_t.Second;
        
        //Serial.println(my_t.Hour);
        //Serial.println(my_t.Minute);
        //Serial.println(my_t.Second);
        storage.writeLong( 80, (long) start_ON);        // 80 ... 83 Start time (unsigned long)
        break;

      case CMD_PROG_MAX_TIME:
        max_ON  = (uint16_t) serial_cmd.parse_int("\0"); // read the min number of seconds
        //Serial.println(max_ON);
        storage.writeInt(86, max_ON);                   // 86 ... 87 Max time (int)
      break;

      case CMD_PROG_MIN_TIME:
        min_ON  = (uint16_t) serial_cmd.parse_int("\0");  // read the min number of seconds
        // Serial.println(min_ON);
        storage.writeInt(84, min_ON);                    // 84 ... 85 Min time (int) store in eeprom
        break;

      case CMD_PROG_MAX_TEMP:
        maxTemp  = 100 * serial_cmd.parse_int("\0");  // read the temperature value
        Serial.println(maxTemp);
        storage.writeInt(92, maxTemp);                  //  92 ...  95 max Temperature (float)      break;
        break;
  
      case CMD_PROG_MIN_TEMP:
        minTemp  =  100 * serial_cmd.parse_int("\0");  // read the temperature value
        Serial.println(minTemp);
        storage.writeInt(88, minTemp);                  //  88 ...  91 min Temperature (float)
        break;

      case CMD_PROG_NAME:
        strcpy(s_DeviceName, serial_cmd.parse_string("\0"));
        storage.writeChars(  0, s_DeviceName, 20);
        break;

      case CMD_PROG_LNAME:
        strcpy(s_LocalName, serial_cmd.parse_string("\0"));
        storage.writeChars(  20, s_DeviceName, 20);
        break;

      case CMD_PROG_WELCOME:
        strcpy(s_Welcome, serial_cmd.parse_string("\0"));
        //Serial.println(s_Welcome);
        storage.writeChars(  40, s_Welcome, 40);
        break;

      case CMD_LOG_OFF:
        my_read_on = false;
        break;

      case CMD_LOG_ON:
        my_read_on = true;
        //Serial.println("ChipT, RTC_T, BMP_T, BMP_P");
        break;

      case CMD_LOG_24H:
        int h, i, j;
        
        h = hour();
        // ts_tmp = ts_tmp - minute(ts_tmp)*60 - second(ts_tmp) - SECS_PER_HOUR *23; // get the timestamp from yesterday

        for(i = 0; i < 24; i++){
          j = h - i;
          if ( j < 0 ) j +=  24;    // loop through the array

          if (j < 10) Serial.print("0");
          Serial.print(j);
          Serial.print(":00 ");

          print_100(ds_temp_24[j]);
          Serial.println( " *C");

        }
        break;

      case CMD_SET_TIME:
        // get the date
        my_t.Year  = year() - 1970; 
        my_t.Month = month(); 
        my_t.Day   = day(); 

        // read the time
        my_t.Hour =   (uint8_t) serial_cmd.parse_int(":");
        my_t.Minute = (uint8_t) serial_cmd.parse_int(":");
        my_t.Second = (uint8_t) serial_cmd.parse_int("\0");
        
        t = makeTime(my_t);

        // tell the new set
        Serial.print( "will set to: ");
        print_timestamp(t);
        Serial.print( " - ");
        print_timestamp(t);
        Serial.println();

        RTC.set(t);        // use the time_t value to ensure correct weekday is set
        setTime(t);             
        break;

      case CMD_SET_DATE:
        // read the time
        my_t.Day    = (uint8_t) serial_cmd.parse_int("/");
        my_t.Month  = (uint8_t) serial_cmd.parse_int("/");
        my_t.Year   = (uint8_t) (serial_cmd.parse_int("\0") - 1970);

        my_t.Hour   = hour();
        my_t.Minute = minute();
        my_t.Second = second();
        
        t = makeTime(my_t);

        // tell the new set
        Serial.print( "will set to: ");
        print_timestamp(t);
        Serial.print( " - ");
        print_timestamp(t);
        Serial.println();


        RTC.set(t);        // use the time_t value to ensure correct weekday is set
        setTime(t);             
        break;

      case CMD_SYS_START:
        if (fsm_watering == FSM_IDLE) fsm_watering = FSM_START;
        break;
        
      case CMD_SYS_STOP:
        if (fsm_watering != FSM_IDLE) fsm_watering = FSM_STOP;
        break;

      case CMD_SYS_DSP_ON:
        my_disp_on = 1;  // activate the flag
        storage.writeInt(96, (uint16_t) my_disp_on ); // store the display status
        break;
      
      case CMD_SYS_DSP_OFF:
        my_disp_on = 0;       // de-activate the flag
        microbit.matrix.clear();  // clear the display
        storage.writeInt(96, (uint16_t) my_disp_on ); // store the display status
        break;

      case CMD_HALT:
        while (true) microbit.matrix.print(s_Halt);
        break;
    }

  } 
  else if (result == -1) Serial.println("Buffer Overflow, please restart");

}
