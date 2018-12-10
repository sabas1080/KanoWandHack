/************************************************************

Kano Wand Hack - DIY Wand

This example demonstrates how emulate an Kano Wand

Development environment specifics:
  IDE: Arduino 1.8.4
  Hardware Platform:
  - Micro:Bit

Distributed as-is; no warranty is given.
************************************************************************/
#include <BLEPeripheral.h>
#include <MadgwickAHRS.h>

// create peripheral instance, see pinouts above
BLEPeripheral            blePeripheral;


#define INFO_SERVICE_UUID    "64a70010-f691-4b93-a6f4-0968f5b648f8"
#define IO_SERVICE_UUID      "64A70012-F691-4B93-A6F4-0968F5B648F8"
#define SENSOR_SERVICE_UUID  "64A70011-F691-4B93-A6F4-0968F5B648F8"


#define BleUUIDInformationOrganisationChar "64A7000B-F691-4B93-A6F4-0968F5B648F8"
#define BleUUIDInformationSwChar           "64A70013-F691-4B93-A6F4-0968F5B648F8"
#define BleUUIDInformationHwChar           "64A70001-F691-4B93-A6F4-0968F5B648F8"

#define BleUUIDIOBatteryChar               "64A70007-F691-4B93-A6F4-0968F5B648F8"
#define BleUUIDIOUserButtonChar            "64A7000D-F691-4B93-A6F4-0968F5B648F8"
#define BleUUIDIOVibratorChar              "64A70008-F691-4B93-A6F4-0968F5B648F8"
#define BleUUIDIOLedChar                   "64A70009-F691-4B93-A6F4-0968F5B648F8" //LED RGB
#define BleUUIDIOKeepAliveChar             "64A7000F-F691-4B93-A6F4-0968F5B648F8"

                                         
#define BleUUIDSensorQuaternionsChar       "64A70002-F691-4B93-A6F4-0968F5B648F8"
#define BleUUIDSensorRawChar               "64A7000A-F691-4B93-A6F4-0968F5B648F8"
#define BleUUIDSensorMotionChar            "64A7000C-F691-4B93-A6F4-0968F5B648F8"
#define BleUUIDSensorMagnCalibrateChar     "64A70021-F691-4B93-A6F4-0968F5B648F8"
#define BleUUIDSensorQuaternionsResetChar  "64A70004-F691-4B93-A6F4-0968F5B648F8"
#define BleUUIDSensorTempChar              "64A70014-F691-4B93-A6F4-0968F5B648F8"

 // BLE INFO Service
BLEService infoService = BLEService(INFO_SERVICE_UUID);

 // BLE IO Service
BLEService ioService = BLEService(IO_SERVICE_UUID);

 // BLE Sensor Service
BLEService sensorService = BLEService(SENSOR_SERVICE_UUID);

// BLE Organisation Characteristic
BLECharacteristic organisationChar(BleUUIDInformationOrganisationChar,  // standard 16-bit characteristic UUID
    BLERead | BLENotify, 4); // remote clients will be able to get notifications if this characteristic changes
// BLE Information Software Characteristic
BLECharacteristic informationSwChar(BleUUIDInformationSwChar,  // standard 16-bit characteristic UUID
    BLERead | BLENotify,3); // remote clients will be able to get notifications if this characteristic changes
    // BLE Information Hardware Characteristic
BLECharacteristic informationHwChar(BleUUIDInformationHwChar,  // standard 16-bit characteristic UUID
    BLERead | BLENotify,3); // remote clients will be able to get notifications if this characteristic changes


    // BLE Battery Level Characteristic
BLEUnsignedCharCharacteristic ioBatteryChar(BleUUIDIOBatteryChar,  // standard 16-bit characteristic UUID
    BLERead | BLENotify); // remote clients will be able to get notifications if this characteristic changes
    // BLE User Button Characteristic
BLEUnsignedCharCharacteristic ioUserButtonChar(BleUUIDIOUserButtonChar,  // standard 16-bit characteristic UUID
    BLERead | BLENotify); // remote clients will be able to get notifications if this characteristic changes
    // BLE Vibrator Characteristic
BLEUnsignedCharCharacteristic ioVibratorChar(BleUUIDIOVibratorChar,  // standard 16-bit characteristic UUID
    BLERead | BLENotify); // remote clients will be able to get notifications if this characteristic changes
    // BLE Led Characteristic
BLECharacteristic LedChar(BleUUIDIOLedChar,  // standard 16-bit characteristic UUID
    BLEWrite | BLENotify,3); // remote clients will be able to get notifications if this characteristic changes
    // BLE Keep Alive Characteristic
BLEUnsignedCharCharacteristic ioKeepAliveChar(BleUUIDIOKeepAliveChar,  // standard 16-bit characteristic UUID
    BLERead | BLENotify); // remote clients will be able to get notifications if this characteristic changes


    // BLE Quaternions Characteristic
BLEUnsignedCharCharacteristic sensorQuaternionsChar(BleUUIDSensorQuaternionsChar,  // standard 16-bit characteristic UUID
    BLERead | BLENotify); // remote clients will be able to get notifications if this characteristic changes
    // BLE idSensorRaw Characteristic
BLEUnsignedCharCharacteristic idSensorRawChar(BleUUIDSensorRawChar,  // standard 16-bit characteristic UUID
    BLERead | BLENotify); // remote clients will be able to get notifications if this characteristic changes
    // BLE sensorMotion Characteristic
BLEUnsignedCharCharacteristic sensorMotionChar(BleUUIDSensorMotionChar,  // standard 16-bit characteristic UUID
    BLERead | BLENotify); // remote clients will be able to get notifications if this characteristic changes
    // BLE MagnCalibrate Characteristic
BLEUnsignedCharCharacteristic sensorMagnCalibrateChar(BleUUIDSensorMagnCalibrateChar,  // standard 16-bit characteristic UUID
    BLERead | BLENotify); // remote clients will be able to get notifications if this characteristic changes
    // BLE QuaternionsReset Characteristic
BLEUnsignedCharCharacteristic sensorQuaternionsResetChar(BleUUIDSensorQuaternionsResetChar,  // standard 16-bit characteristic UUID
    BLERead | BLENotify); // remote clients will be able to get notifications if this characteristic changes
    // BLE Temperature Characteristic
BLEUnsignedCharCharacteristic sensorTempChar(BleUUIDSensorTempChar,  // standard 16-bit characteristic UUID
    BLERead | BLENotify); // remote clients will be able to get notifications if this characteristic changes

const int buttonA = 5;     // the number of the pushbutton pin
const int buttonB = 11;     // the number of the pushbutton pin

long previousMillis = 0;  // last time checked, in ms
int oldBatteryLevel = 0;  // last battery level reading from analog input

Madgwick filter;
unsigned long microsPerReading, microsPrevious;
float accelScale, gyroScale;

void setup() {
  
  Serial.begin(9600);
  //while (!Serial);
  Serial.println("Processor came out of reset.\n");
  
  pinMode(buttonA, INPUT);  
  pinMode(buttonB, INPUT);   
  
  /* Set a local name for the BLE device
     This name will appear in advertising packets
     and can be used by remote devices to identify this BLE device
     The name can be changed but maybe be truncated based on space left in advertisement packet
  */
  blePeripheral.setLocalName("Kano-Wand-75-80-89");
  
  blePeripheral.setAdvertisedServiceUuid(infoService.uuid()); // add the service UUID
  blePeripheral.addAttribute(infoService); // Add the info service
  blePeripheral.addAttribute(organisationChar); // add the organisationChar characteristic
  blePeripheral.addAttribute(informationSwChar); // add the organisationChar characteristic
  blePeripheral.addAttribute(informationHwChar); // add the organisationChar characteristic


  blePeripheral.setAdvertisedServiceUuid(ioService.uuid()); // add the service UUID
  blePeripheral.addAttribute(ioService); // Add the info service
  blePeripheral.addAttribute(ioBatteryChar); // add the stepsTaken characteristic
  blePeripheral.addAttribute(ioUserButtonChar); // add the organisationChar characteristic
  blePeripheral.addAttribute(ioVibratorChar); // add the organisationChar characteristic
  blePeripheral.addAttribute(LedChar); // add the organisationChar characteristic
  blePeripheral.addAttribute(ioKeepAliveChar); // add the organisationChar characteristic


  blePeripheral.setAdvertisedServiceUuid(sensorService.uuid()); // add the service UUID
  blePeripheral.addAttribute(sensorService); // Add the info service
  blePeripheral.addAttribute(sensorQuaternionsChar); // add the stepsTaken characteristic
  blePeripheral.addAttribute(idSensorRawChar); // add the organisationChar characteristic
  blePeripheral.addAttribute(sensorMotionChar); // add the organisationChar characteristic
  blePeripheral.addAttribute(sensorMagnCalibrateChar); // add the organisationChar characteristic
  blePeripheral.addAttribute(sensorQuaternionsResetChar); // add the organisationChar characteristic
  blePeripheral.addAttribute(sensorTempChar); // add the organisationChar characteristic

  // assign event handlers for connected, disconnected to peripheral
  blePeripheral.setEventHandler(BLEConnected, blePeripheralConnectHandler);
  blePeripheral.setEventHandler(BLEDisconnected, blePeripheralDisconnectHandler);

  // assign event handlers for characteristic
  LedChar.setEventHandler(BLEWritten, ledCharacteristicWritten);

  // assign event handlers for characteristic
  //sensorQuaternionsChar.setEventHandler(BLERead, MotionCharacteristicWritten);
  // set an initial value for the characteristic
  //sensorQuaternionsChar.setValue(0);

  //Defualt Factory
  organisationChar.setValue("Kano");
  informationSwChar.setValue("1.1");
  informationHwChar.setValue("1");

  /* Start advertising BLE.  It will start continuously transmitting BLE
     advertising packets and will be visible to remote BLE central devices
     until it receives a new connection */

  // begin initialization
  blePeripheral.begin();

  Serial.println("Bluetooth device active, waiting for connections...");
}

void loop() {

  // poll peripheral
  blePeripheral.poll();

  // read the current button pin state
  char buttonValue = digitalRead(buttonA);

  // has the value changed since the last read
  bool buttonChanged = (ioUserButtonChar.value() != buttonValue);

  if (buttonChanged) {
    // button state changed, update characteristics
    ioUserButtonChar.setValue(buttonValue);
    Serial.println(F("Pressed Button"));
  }
  
}

void blePeripheralConnectHandler(BLECentral& central) {
  Serial.print(F("Connected event, central: "));
  Serial.println(central.address());
}

void blePeripheralDisconnectHandler(BLECentral& central) {
  Serial.print(F("Disconnected event, central: "));
  Serial.println(central.address());
}

void updateBatteryLevel() {
  /* Read the current voltage level on the A0 analog input pin.
     This is used here to simulate the charge level of a battery.
  */
  int battery = analogRead(A0);
  int batteryLevel = map(battery, 0, 1023, 0, 100);

  if (batteryLevel != oldBatteryLevel) {      // if the battery level has changed
    Serial.print("Battery Level % is now: "); // print it
    Serial.println(batteryLevel);
    ioBatteryChar.setValue(batteryLevel);  // and update the battery level characteristic
    oldBatteryLevel = batteryLevel;           // save the level for next comparison
  }
}

void buttonCharacteristicWritten() {
  // set an value for the characteristic
  ioUserButtonChar.setValue(0);
}

void ledCharacteristicWritten(BLECentral& central, BLECharacteristic& characteristic) {
  // central wrote new value to characteristic, update LED
  Serial.print("Characteristic event, written: ");
  if (LedChar.value()) {
    Serial.write(LedChar.value(), LedChar.valueLength());
  } else {
    Serial.write(LedChar.value(), LedChar.valueLength());
  }
}
