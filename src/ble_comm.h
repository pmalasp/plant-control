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


/*
 * manage the bluetooth channel
 */
void init_BLE();
void set_characteristics_BLE(bool force_initialization);
void set_characteristics_BLE();
void process_BLE();

/*
 * process the Bluetooth events
 */
void blePeripheralConnectHandler(BLECentral& central) ;
void blePeripheralDisconnectHandler(BLECentral& central);
void bleControlReceived(BLECentral& central, BLECharacteristic& characteristic);
void bleProgramReceived(BLECentral& central, BLECharacteristic& characteristic);
void bleDateTimeReceived(BLECentral& central, BLECharacteristic& characteristic);
