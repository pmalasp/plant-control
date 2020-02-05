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
#include <Adafruit_Microbit.h>
#include "utils.h"
#include "globals.h"
#include "ble_comm.h"

bool                ble_read_on = false;
parse_Command       BLE_cmd;



/*
 * This code implements the following BLE (Bluetooth Low Energy) objects:
 * 
 *      https://www.bluetooth.com/specifications/assigned-numbers/
 *      
 *   User define services
 *     https://www.guidgenerator.com/online-guid-generator.aspx
 *     https://www.uuidgenerator.net/ 
 *      
 *      The BLE Environmental Sensing Service
 *        (see https://www.bluetooth.com/specifications/gatt/services/)
 *      The Temperature Characteristics
 *        https://www.bluetooth.com/wp-content/uploads/Sitecore-Media-Library/Gatt/Xml/Services/org.bluetooth.service.environmental_sensing.xml
 *        https://www.bluetooth.com/wp-content/uploads/Sitecore-Media-Library/Gatt/Xml/Characteristics/org.bluetooth.characteristic.date_time.xml
 *        
 *      Time service  
 *        https://www.bluetooth.com/specifications/gatt/viewer?attributeXmlFile=org.bluetooth.characteristic.date_time.xml
 *        
 *        
 *   Environmental service UUID:0000181A-0000-1000-8000-00805F9B34FB
 *     Standard GATT UUID for temperature, UUID:00002A6E-0000-1000-8000-00805F9B34FB
 *     Custom   GATT UUID 24h temperature, UUID:2fb27321-ae84-49ec-be7a-e0bc7e94858a 
 *     
 *   CurrentTimeService    UUID:00001805-0000-1000-8000-00805F9B34FB
 *     Standard GATT Standard GATT DateTimeCharacteristic UUID:00002A08-0000-1000-8000-00805F9B34FB 
 *     
 *   Custom service        UUID:     
 *     Custom attribute    UUID:
 *     Custom attribute    UUID:
 *     Custom attribute    UUID: 
 *     Custom attribute    UUID: 
 *     Custom attribute    UUID:d2b525ea-bd81-4b8c-84de-87cdd545567e 
 *          
 *     
 *        
 * BLE objects presented in the program:       
 *   Programming data:
 *     Start Time HH:MM                         (RW)   time_t    start_ON
 *     Min and Max duration                     (RW)   uint16_t  min_ON,  max_ON
 *     Min and Max reference °C                 (RW)   int16_t   minTemp, maxTemp; 
 *     Display (ON/OFF)                         (RW)   uint16_t  my_disp_on
 *     
 *   Status data:     
 *  -   Current Ds18b20 temperature sensor value (RO)
 *  -   Current RTC     temperature sensor value (RO)
 *  -   Current proc.   temperature sensor value (RO)
 *  -   Last 24h average temperature             (RO)
 *  -   Current time                             (RO)
 *  -   Remaining time                           (RO)   uint16_t  elapse_ON = 0;

 *  
 *   Control data:
 *  -   Manual Start                             (-W)   uint8_t 
 *  -   Manual Stop                              (-W)   uint8_t 
 *  -   Reboot                                   (-W) - uint8_t go on a loop and trigger the watchdog
 *     
 *     
        const unsigned char DateTimeCharArray[7] = { 226, 7, 5, 8, 9, 3, 0 };   // 2018 (0x07E2 - 7 226) May 8th , 09:03:00
        uint8_t DateTimeCharArray[7];
        DateTimeCharArray[0] = now.year() & 0x00FF; // LSB (0xE2) 2018 = 0x07E2
        DateTimeCharArray[1] = now.year() >> 8;     // MSB (0x07)
        DateTimeCharArray[2] = now.month();  
        DateTimeCharArray[3] = now.day();
        DateTimeCharArray[4] = now.hour();
        DateTimeCharArray[5] = now.minute();  
        DateTimeCharArray[6] = now.second();    
      
        DateTimeCharacteristic.setValue(DateTimeCharArray, 7);  // and update date time characteristic
 *   
 *   
 *   
 *   
 */


/***********************************************************************/ 
BLEService              tempService =             BLEService("181A");    // Environmental service UUID:0000181A-0000-1000-8000-00805F9B34FB

BLECharacteristic       tempDsCharacteristic =      BLECharacteristic("2A6E", BLERead | BLENotify, 6);    // Standard GATT UUID for temperature, UUID:00002A6E-0000-1000-8000-00805F9B34FB
BLEDescriptor           tempDsDescriptor =            BLEDescriptor("2901", "Sensors temperature");

BLECharacteristic       tempDs24_1Characteristic =  BLECharacteristic("2fb17321-ae84-49ec-be7a-e0bc7e94858a", BLERead | BLENotify , 48); 
BLEDescriptor           tempDs24_1Descriptor =        BLEDescriptor("2901", "00 - 07 Temperatures");
BLECharacteristic       tempDs24_2Characteristic =  BLECharacteristic("c55ba99f-0d64-4f4a-a069-7c60471a038d", BLERead | BLENotify , 48); 
BLEDescriptor           tempDs24_2Descriptor =        BLEDescriptor("2901", "08 - 15 Temperatures");
BLECharacteristic       tempDs24_3Characteristic =  BLECharacteristic("8bf012bb-4ba9-4e6b-918b-02aebdc02661", BLERead | BLENotify , 48); 
BLEDescriptor           tempDs24_3Descriptor =        BLEDescriptor("2901", "16 - 23 Temperatures");

/***********************************************************************/ 
BLEService              CurrentTimeService =      BLEService("1805");  // Time service UUID:00001805-0000-1000-8000-00805F9B34FB
BLECharacteristic       DateTimeCharacteristic =    BLECharacteristic("2A08", BLERead | BLENotify | BLEWrite , 7); // Standard GATT UUID for current time, UUID:00002A08-0000-1000-8000-00805F9B34FB
BLEDescriptor           DateTimeDescriptor =          BLEDescriptor("2901", "Current Time");

/***********************************************************************/ 
BLEService              controlService =          BLEService("7f98e438-8a72-492f-9b5b-2f5d30be37b8");  // 
BLECharacteristic       controlCharacteristic =     BLECharacteristic("11ea4d30-89b9-4fbe-8ef2-d627395fe69b", BLEWrite , 3); // 
BLEDescriptor           controlDescriptor =           BLEDescriptor("2901", "Start, Stop, Halt");
BLECharacteristic       remainingTimeCharacteristic=BLECharacteristic("2a970a8d-b841-4b94-b204-75ca16fb50f9", BLERead | BLENotify, 2); // 
BLEDescriptor           remainingTimeDescriptor =     BLEDescriptor("2901", "Remaining Water Time");


/***********************************************************************/ 
BLEService              programService =          BLEService("b03ee4b6-2c83-4473-a5a7-59171486880b");  // 
BLECharacteristic       programCharacteristic =     BLECharacteristic("5a676f6d-be38-4330-8fa8-b6859ac5c79f", BLERead | BLENotify | BLEWrite , 14); // 
BLEDescriptor           programDescriptor =           BLEDescriptor("2901", "Programming data");


/***********************************************************************/ 
//BLEService              xxxService =    BLEService("");  // 
//BLECharacteristic       yyyCharacteristic = BLECharacteristic("", BLERead | BLENotify | BLEWrite , n); // 
//BLEDescriptor           xxxDescriptor =        BLEDescriptor("2901", "");




/*
 * initialize the bluetooth channel
 */
void init_BLE()
{
  // clears bond data on every boot
  //bleBondStore.clearData();
  //microbit.BTLESerial.setBondStore(bleBondStore);

  
  // Assign the name
  microbit.BTLESerial.setLocalName(s_LocalName);
  microbit.BTLESerial.setDeviceName(s_DeviceName);

  // Environmental service
  microbit.BTLESerial.setAdvertisedServiceUuid(tempService.uuid());
  microbit.BTLESerial.addAttribute(tempService);
  microbit.BTLESerial.addAttribute(tempDsCharacteristic);
  microbit.BTLESerial.addAttribute(tempDsDescriptor);
  microbit.BTLESerial.addAttribute(tempDs24_1Characteristic);
  microbit.BTLESerial.addAttribute(tempDs24_1Descriptor);
  microbit.BTLESerial.addAttribute(tempDs24_2Characteristic);
  microbit.BTLESerial.addAttribute(tempDs24_2Descriptor);
  microbit.BTLESerial.addAttribute(tempDs24_3Characteristic);
  microbit.BTLESerial.addAttribute(tempDs24_3Descriptor);


  // Time service
  microbit.BTLESerial.setAdvertisedServiceUuid(CurrentTimeService.uuid());
  microbit.BTLESerial.addAttribute(CurrentTimeService);
  microbit.BTLESerial.addAttribute(DateTimeCharacteristic);
  microbit.BTLESerial.addAttribute(DateTimeDescriptor);

  DateTimeCharacteristic.setEventHandler(BLEWritten, bleDateTimeReceived);

  
  // Control Service
  microbit.BTLESerial.setAdvertisedServiceUuid(controlService.uuid());  // 
  microbit.BTLESerial.addAttribute(controlService);
  microbit.BTLESerial.addAttribute(controlCharacteristic); // 
  microbit.BTLESerial.addAttribute(controlDescriptor);
  microbit.BTLESerial.addAttribute(remainingTimeCharacteristic); // 
  microbit.BTLESerial.addAttribute(remainingTimeDescriptor);

  controlCharacteristic.setEventHandler(BLEWritten, bleControlReceived);

  // Programming service
  microbit.BTLESerial.setAdvertisedServiceUuid(programService.uuid());  // 
  microbit.BTLESerial.addAttribute(programService);
  microbit.BTLESerial.addAttribute(programCharacteristic); // 
  microbit.BTLESerial.addAttribute(programDescriptor);

  programCharacteristic.setEventHandler(BLEWritten, bleProgramReceived);
  
  // register handlers
  microbit.BTLESerial.setEventHandler(BLEConnected,     blePeripheralConnectHandler);
  microbit.BTLESerial.setEventHandler(BLEDisconnected,  blePeripheralDisconnectHandler);
  //microbit.BTLESerial.setEventHandler(BLEBonded,        blePeripheralBondedHandler);

  // custom services and characteristics can be added as well
  microbit.BTLESerial.begin();

  // increase tx power for longer range
  microbit.BTLESerial.setTxPower(4);


}


/*
 * process the Bluetooth communication
 */
void set_characteristics_BLE() { set_characteristics_BLE(false ); }

void set_characteristics_BLE(bool force_init){
  uint8_t         buffArray[20];
  const uint8_t*  valArray; 
  bool            bUpdate;

  
/*   
 *    Status data:     
 *  -   Current Ds18b20 temperature sensor value (RO)
 *  -   Current RTC     temperature sensor value (RO)
 *  -   Current proc.   temperature sensor value (RO)
 *  -   Last 24h average temperature             (RO)
 *  -   Current time                             (RO)
 *  
 */

  // Time update
  valArray = DateTimeCharacteristic.value(); 
  buffArray[0] = year() & 0x00FF; // LSB (0xE2) 2018 = 0x07E2
  buffArray[1] = year() >> 8;     // MSB (0x07)
  buffArray[2] = month();  
  buffArray[3] = day();
  buffArray[4] = hour();
  buffArray[5] = minute();  
  buffArray[6] = second();    
  if (force_init || (memcmp(buffArray, valArray, 7) ) ){
    DateTimeCharacteristic.setValue(buffArray, 7);  // and update date time characteristic
  }

  // Temperature update
  valArray = tempDsCharacteristic.value();
  if (force_init || (memcmp(_temperatures, valArray, 6) ) ){
    tempDsCharacteristic.setValue( (uint8_t*)(  _temperatures), 6 );
  }
  
  valArray = tempDs24_1Characteristic.value(); // for(int i = 0; i < 16; i++ ) Serial.print( valArray[i]);
  if (force_init || (memcmp(ds_temp_24, valArray, 16) ) ){
    tempDs24_1Characteristic.setValue( (uint8_t*)ds_temp_24, 16); // 16, 32, 48 .... max length is 20 chars
  }
  
  valArray = tempDs24_2Characteristic.value(); // for(int  i = 0; i < 16; i++ ) Serial.print( valArray[i]);
  if (force_init || (memcmp(&(ds_temp_24[8]), valArray, 16) ) ){
    tempDs24_2Characteristic.setValue( (uint8_t*)&(ds_temp_24[8]), 16); // 16, 32, 48 .... max length is 20 chars
  }
  
  valArray = tempDs24_3Characteristic.value(); // for(int i = 0; i < 16; i++ ) Serial.print( valArray[i]);
  if (force_init || (memcmp(&(ds_temp_24[16]), valArray, 16) ) ){
    tempDs24_3Characteristic.setValue( (uint8_t*)&(ds_temp_24[16]), 16); // 16, 32, 48 .... max length is 20 chars
  }
  
/*
 *   Programming data:
 *     Start Time HH:MM                         (RW)   time_t    start_ON
 *     Min and Max duration                     (RW)   uint16_t  min_ON,  max_ON
 *     Min and Max reference °C                 (RW)   int16_t   minTemp, maxTemp; 
 *     Display (ON/OFF)                         (RW)   uint16_t  my_disp_on
 *     
 */
  // programCharacteristic
  valArray = programCharacteristic.value(); 
  buffArray[0] = hour(start_ON);
  buffArray[1] = 0;
  buffArray[2] = minute(start_ON);
  buffArray[3] = 0;
  buffArray[4] = min_ON & 0xFF;
  buffArray[5] = min_ON >> 8;
  buffArray[6] = max_ON & 0xFF;
  buffArray[7] = max_ON >> 8;
  buffArray[8] = minTemp& 0xFF;
  buffArray[9] = minTemp >> 8;
  buffArray[10] = maxTemp & 0xFF;
  buffArray[11] = maxTemp >> 8;
  buffArray[12] = my_disp_on & 0xFF;
  buffArray[13] = my_disp_on >> 8;
  if (force_init || (memcmp(buffArray, valArray, 14) ) ){
    programCharacteristic.setValue(buffArray, 14);
  }



  // remainingTimeCharacteristic //  uint16_t  elapse_ON = 0;
  valArray = remainingTimeCharacteristic.value(); 
    
  buffArray[0] = elapse_ON & 0xFF;
  buffArray[1] = elapse_ON >> 8;

  if ( force_init || (memcmp(buffArray, valArray, 2) )  ){
    remainingTimeCharacteristic.setValue(buffArray, 2);
  }
  
 
 
}

/*
 * process the Bluetooth events
 */
void blePeripheralConnectHandler(BLECentral& central) {
  Serial.print(F("Connected event, central: "));
  Serial.println(central.address());

  set_characteristics_BLE(true);
}

void blePeripheralDisconnectHandler(BLECentral& central) {
  Serial.print(F("Disconnected event, central: "));
  Serial.println(central.address());
}
        

/*   Control data:
 *  -   Manual Start                             (-W)   uint8_t 
 *  -   Manual Stop                              (-W)   uint8_t 
 *  -   Reboot                                   (-W) - uint8_t go on a loop and trigger the watchdog
 *  
 */  
void bleControlReceived(BLECentral& central, BLECharacteristic& characteristic) {
  const uint8_t*  valArray; 

  valArray = controlCharacteristic.value();  // Serial.print("Control: "); for(int i = 0; i < 3; i++ ) Serial.print( valArray[i]); Serial.println();

  if        (valArray[0] == 1) { //Manual start
      if (fsm_watering == FSM_IDLE) fsm_watering = FSM_START;
        
  } else if (valArray[1] == 1) { //Manual stop
      if (fsm_watering != FSM_IDLE) fsm_watering = FSM_STOP;
    
  } else if (valArray[2] == 1) { //system halting, forcing the watchdog to reboot
      while (true) microbit.matrix.print(s_Halt);
  } 
 
  
  
}

/*
 *   Programming data:
 *     Start Time HH:MM                         (RW)   time_t    start_ON
 *     Min and Max duration                     (RW)   uint16_t  min_ON,  max_ON
 *     Min and Max reference °C                 (RW)   int16_t   minTemp, maxTemp; 
 *     Display (ON/OFF)                         (RW)   uint16_t  my_disp_on
 *     
 */
void bleProgramReceived(BLECentral& central, BLECharacteristic& characteristic) {
  const uint8_t*  valArray; 
  time_t          new_ON;  
  int             new_VAL;

  valArray = programCharacteristic.value();  Serial.print("Program: "); for(int i = 0; i < 14; i++ ) {Serial.print( valArray[i]); Serial.print("," );  }Serial.println();

  new_ON = ((int) valArray[0]) * SECS_PER_HOUR + ((int) valArray[2]) * SECS_PER_MIN;
  if ( start_ON != new_ON){
    start_ON = new_ON;
    storage.writeLong( 80, (long) start_ON);        // 80 ... 83 Start time (unsigned long)
  }

  new_VAL = (int) valArray[4] | (int) ( ((uint16_t) valArray[5]) << 8);
  if (new_VAL != min_ON) {
    min_ON = new_VAL;
    storage.writeInt(84, min_ON);                    // 84 ... 85 Min time (int) store in eeprom
  }


  new_VAL = (int) valArray[6] | (int) ( ((uint16_t) valArray[7]) << 8);
  if (new_VAL != max_ON) {
    max_ON = new_VAL;
    storage.writeInt(86, max_ON);                   // 86 ... 87 Max time (int)
  }

  new_VAL = (int) valArray[8] | (int) ( ((uint16_t) valArray[9]) << 8);
  if (new_VAL != minTemp) {
    minTemp = new_VAL;
    storage.writeInt(88, minTemp);                  //  88 ...  91 min Temperature (float)
  }

  new_VAL = (int) valArray[10] | (int) ( ((uint16_t) valArray[11]) << 8);
  if (new_VAL != maxTemp) {
    maxTemp = new_VAL;
    storage.writeInt(92, maxTemp );                 //  92 ...  95 max Temperature (float)
  }

  if ( my_disp_on != (int) valArray[12]) {
    if (valArray[12] == 1) {
      my_disp_on = 1;
    } else {
      my_disp_on = 0;
      microbit.matrix.clear();  // clear the display
    }
    storage.writeInt(96, (uint16_t) my_disp_on );   // update the my_disp_flag
  }


// Debug -----------------------
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
// Debug -----------------------


    
}

void bleDateTimeReceived(BLECentral& central, BLECharacteristic& characteristic) {
  const uint8_t*  valArray; 
  tmElements_t    my_t;
  time_t          t;  


  valArray = DateTimeCharacteristic.value();  // Serial.print("DateTime: "); for(int i = 0; i < 7; i++ ) Serial.print( valArray[i]); Serial.println();

  // get the date
  my_t.Year  = ((int) valArray[0] | (int) ( (uint16_t) valArray[1] <<  8 ))  - 1970; 
  my_t.Month = (int) valArray[2]; 
  my_t.Day   = (int) valArray[3]; 

  // read the time
  my_t.Hour =   (int) valArray[4];
  my_t.Minute = (int) valArray[5];
  my_t.Second = (int) valArray[6];
  
  t = makeTime(my_t);

  RTC.set(t);        // use the time_t value to ensure correct weekday is set
  setTime(t);             

}



/*
 * process the Bluetooth communication
 */
void process_BLE()
{
  uint8_t       DateTimeCharArray[7];
  int           result, cmd;
  tmElements_t  my_t;
  
  // skip everything if not connected
  if (!microbit.BTLESerial.connected() ) return;
  
 
  // print the values
  if (sys_one_second() ) 
  {

    //
    microbit.BTLESerial.poll();

    // Update values
    set_characteristics_BLE();
    
    microbit.BTLESerial.poll();

    /*
    if (second() == 1 ){  // update the characteristics every minute 
      Serial.println(F("Updated characteristics values"));
      
      tempDsCharacteristic.setValue( (uint8_t*)(  _temperatures), 6 );
      
      if (minute() == 0 ){  // update the 24h array characteristics every hour
        tempDs24_1Characteristic.setValue( (uint8_t*)ds_temp_24, 48);
      }
    }
    */
    

    if (ble_read_on){ // on ble constant logging can be annoying
      print_timestamp(microbit.BTLESerial);
      
  
      microbit.BTLESerial.print(" - ");
      
      microbit.BTLESerial.print(die_temp); // MPU chip temperature
      microbit.BTLESerial.print(" *C d, ");
      
      microbit.BTLESerial.print(rtc_temp); // rtc
      microbit.BTLESerial.print(" *C rt, ");
      
      microbit.BTLESerial.print(ds_temp); // ds
      microbit.BTLESerial.print(" *C ds, ");
      
      if (fsm_watering == FSM_WATERING )
      {
        microbit.BTLESerial.print( ", ON for ");
        microbit.BTLESerial.print(elapse_ON );
        microbit.BTLESerial.print( " s");
      }
  
    
      microbit.BTLESerial.println();
      microbit.BTLESerial.flush();
      
    } else {  // just to show it is still alive
      microbit.BTLESerial.print( ".");
    }

  }

  result = get_Command(BLE_cmd.input_command, microbit.BTLESerial); // get the command line, 0 is line completed
  if (result == 0 )
  {
    cmd = BLE_cmd.parse();

    switch (cmd) 
    {
      case CMD_ERROR:
        microbit.BTLESerial.println("command error");
        break;

      case CMD_VERSION:
        microbit.BTLESerial.print("Version: ");
        microbit.BTLESerial.println(PROGRAM_VERSION);
        microbit.BTLESerial.print("Compile date: ");
        microbit.BTLESerial.println( __DATE__);
        microbit.BTLESerial.print("Compile time: ");
        microbit.BTLESerial.println( __TIME__ );
        break;
      
      case CMD_STATUS_PRINT:

        microbit.BTLESerial.print("ON: ");
        
        print_timestamp(microbit.BTLESerial, start_ON);
        microbit.BTLESerial.print(", ");

        microbit.BTLESerial.print(min_ON);
        microbit.BTLESerial.print(" s ");
        microbit.BTLESerial.print(minTemp / 100);
        microbit.BTLESerial.print(" *C [min], " );

        microbit.BTLESerial.print(max_ON);
        microbit.BTLESerial.print(" s ");
        microbit.BTLESerial.print(maxTemp / 100);
        microbit.BTLESerial.println(" *C [MAX]" );

        break;

      case CMD_PROG_START:
        // microbit.BTLESerial.println("decode start time");
        // read the time
        my_t.Hour   = (uint8_t) BLE_cmd.parse_int(":");
        my_t.Minute = (uint8_t) BLE_cmd.parse_int(":");
        my_t.Second = (uint8_t) 0;
        // my_t.Second = (uint8_t) BLE_cmd.parse_int("#");
        
        start_ON = my_t.Hour * SECS_PER_HOUR + my_t.Minute * SECS_PER_MIN + my_t.Second;
        
        // microbit.BTLESerial.println(my_t.Hour);
        // microbit.BTLESerial.println(my_t.Minute);
        // microbit.BTLESerial.println(my_t.Second);
        storage.writeLong( 80, (long) start_ON);        // 80 ... 83 Start time (unsigned long)
        break;

      case CMD_PROG_MAX_TIME:
        max_ON  = (uint16_t) BLE_cmd.parse_int("\0"); // read the min number of seconds
        storage.writeInt(86, max_ON);                   // 86 ... 87 Max time (int)
      break;

      case CMD_PROG_MIN_TIME:
        min_ON  = (uint16_t) BLE_cmd.parse_int("\0");  // read the min number of seconds
        storage.writeInt(84, min_ON);                    // 84 ... 85 Min time (int) store in eeprom
        break;

      case CMD_PROG_MAX_TEMP:
        maxTemp  =  100 * BLE_cmd.parse_int("\0");            // read the temperature value
        storage.writeInt(92, maxTemp );          //  92 ...  95 max Temperature (float)      break;
        break;
  
      case CMD_PROG_MIN_TEMP:
        minTemp  =  100 * BLE_cmd.parse_int("\0");  // read the temperature value
        storage.writeInt(88, minTemp);                  //  88 ...  91 min Temperature (float)
        break;


      case CMD_LOG_OFF:
        ble_read_on = false;
        break;

      case CMD_LOG_ON:
        ble_read_on = true;
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
        
      case CMD_HELP:
        BLE_cmd.help(microbit.BTLESerial);
        break;

      case CMD_HALT:
        while (true) microbit.matrix.print(s_Halt);
        break;

      case CMD_I2C_SCAN:
      case CMD_1W_SCAN:
      case CMD_PROG_NAME:
      case CMD_PROG_LNAME:
      case CMD_PROG_WELCOME:
      case CMD_SET_TIME:
      case CMD_SET_DATE:
      case CMD_LOG_24H:
        microbit.BTLESerial.println("cmd not supported over BLE");
        break;
    }

    microbit.BTLESerial.flush();


  } 
  else if (result == -1) Serial.println("Buffer Overflow, please restart");

    

}

/*/
void blePeripheralBondedHandler(BLECentral& central) {
  // central bonded event handler
  Serial.print(F("Remote bonded event, central: "));
  Serial.println(central.address());

}

/*/
