/************************************************************

Kano Wand Hack - DIY Wand

This example demonstrates how emulate an Kano Wand

Development environment specifics:
  IDE: Arduino 1.8.4
  Hardware Platform:
  - Arduino WiFi Rev2

Distributed as-is; no warranty is given.
************************************************************************/
#include <ArduinoBLE.h>
#include <SPI.h>
#include <WiFiNINA.h> //Only for control led RGB
#include <utility/wifi_drv.h> //Only for control led RGB
#include "SparkFunLSM6DS3.h"
#include "Wire.h"

LSM6DS3Core myIMU( SPI_MODE, SPIIMU_SS ); //SPIIMU_SS is the CS pin for Arduino WiFi Rev2

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
BLEService infoService(INFO_SERVICE_UUID);

 // BLE IO Service
BLEService ioService(IO_SERVICE_UUID);

 // BLE Sensor Service
BLEService sensorService(SENSOR_SERVICE_UUID);


// BLE Organisation Characteristic
BLEUnsignedCharCharacteristic organisationChar(BleUUIDInformationOrganisationChar,  // standard 16-bit characteristic UUID
    BLERead | BLENotify); // remote clients will be able to get notifications if this characteristic changes
// BLE Information Software Characteristic
BLEUnsignedCharCharacteristic informationSwChar(BleUUIDInformationSwChar,  // standard 16-bit characteristic UUID
    BLERead | BLENotify); // remote clients will be able to get notifications if this characteristic changes
    // BLE Information Hardware Characteristic
BLEUnsignedCharCharacteristic informationHwChar(BleUUIDInformationHwChar,  // standard 16-bit characteristic UUID
    BLERead | BLENotify); // remote clients will be able to get notifications if this characteristic changes


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
BLEUnsignedCharCharacteristic LedChar(BleUUIDIOLedChar,  // standard 16-bit characteristic UUID
    BLERead | BLENotify); // remote clients will be able to get notifications if this characteristic changes
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

long previousMillis = 0;  // last time checked, in ms
    
void setup() {
  Serial.begin(9600);
  //while (!Serial);
  Serial.println("Processor came out of reset.\n");

  WiFiDrv::pinMode(25, OUTPUT);  //GREEN
  WiFiDrv::pinMode(26, OUTPUT);  //RED
  WiFiDrv::pinMode(27, OUTPUT);  //BLUE
  WiFiDrv::analogWrite(25, 128);  // for configurable brightness
  WiFiDrv::analogWrite(26, 128);  // for configurable brightness
  WiFiDrv::analogWrite(27, 128);  // for configurable brightness

   // begin initialization
  if (!BLE.begin()) {
    Serial.println("starting BLE failed!");

    while (1);
  }
  
  //Call .beginCore() to configure the IMU
  if( myIMU.beginCore() != 0 )
  {
    Serial.print("Error at beginCore().\n");
  }
  else
  {
    Serial.print("\nbeginCore() passed.\n");
  }

  /* Set a local name for the BLE device
     This name will appear in advertising packets
     and can be used by remote devices to identify this BLE device
     The name can be changed but maybe be truncated based on space left in advertisement packet
  */
  BLE.setLocalName("Kano-Wand-75-80-89");
  
  BLE.setAdvertisedService(infoService); // add the service UUID
  infoService.addCharacteristic(organisationChar); // add the organisationChar characteristic
  infoService.addCharacteristic(informationSwChar); // add the organisationChar characteristic
  infoService.addCharacteristic(informationHwChar); // add the organisationChar characteristic
  BLE.addService(infoService); // Add the info service

  BLE.setAdvertisedService(ioService); // add the service UUID
  ioService.addCharacteristic(ioBatteryChar); // add the stepsTaken characteristic
  ioService.addCharacteristic(ioUserButtonChar); // add the organisationChar characteristic
  ioService.addCharacteristic(ioVibratorChar); // add the organisationChar characteristic
  ioService.addCharacteristic(LedChar); // add the organisationChar characteristic
  ioService.addCharacteristic(ioKeepAliveChar); // add the organisationChar characteristic
  BLE.addService(ioService); // Add the info service

  BLE.setAdvertisedService(sensorService); // add the service UUID
  sensorService.addCharacteristic(sensorQuaternionsChar); // add the stepsTaken characteristic
  sensorService.addCharacteristic(idSensorRawChar); // add the organisationChar characteristic
  sensorService.addCharacteristic(sensorMotionChar); // add the organisationChar characteristic
  sensorService.addCharacteristic(sensorMagnCalibrateChar); // add the organisationChar characteristic
  sensorService.addCharacteristic(sensorQuaternionsResetChar); // add the organisationChar characteristic
  sensorService.addCharacteristic(sensorTempChar); // add the organisationChar characteristic
  BLE.addService(sensorService); // Add the info service

  // assign event handlers for connected, disconnected to peripheral
  BLE.setEventHandler(BLEConnected, blePeripheralConnectHandler);
  BLE.setEventHandler(BLEDisconnected, blePeripheralDisconnectHandler);

  // assign event handlers for characteristic
  LedChar.setEventHandler(BLEWritten, ledCharacteristicWritten);
  // set an initial value for the characteristic
  LedChar.setValue(0);

  /* Start advertising BLE.  It will start continuously transmitting BLE
     advertising packets and will be visible to remote BLE central devices
     until it receives a new connection */

  // start advertising
  BLE.advertise();

  Serial.println("Bluetooth device active, waiting for connections...");
}

void loop() {
    // wait for a BLE central
  BLEDevice central = BLE.central();

  // if a central is connected to the peripheral:
  if (central) {
    Serial.print("Connected to central: ");
    // print the central's BT address:
    Serial.println(central.address());
    // turn on the LED to indicate the connection:
    digitalWrite(LED_BUILTIN, HIGH);

    // check every 200ms
    // while the central is connected:
    while (central.connected()) {
      long currentMillis = millis();
      // if 200ms have passed:
      if (currentMillis - previousMillis >= 200) {
        previousMillis = currentMillis;
      }
    }
    // when the central disconnects, turn off the LED:
    digitalWrite(LED_BUILTIN, LOW);
    Serial.print("Disconnected from central: ");
    Serial.println(central.address());
  } 
}

void blePeripheralConnectHandler(BLEDevice central) {
  // central connected event handler
  Serial.print("Connected event, central: ");
  Serial.println(central.address());
}

void blePeripheralDisconnectHandler(BLEDevice central) {
  // central disconnected event handler
  Serial.print("Disconnected event, central: ");
  Serial.println(central.address());
}

void ledCharacteristicWritten(BLEDevice central, BLECharacteristic characteristic) {
  // central wrote new value to characteristic, update LED
  Serial.print("Characteristic event, written: ");
  if (LedChar.value()) {
    Serial.println("LED on");
    WiFiDrv::analogWrite(25, 128);  // for configurable brightness
    WiFiDrv::analogWrite(26, 128);  // for configurable brightness
    WiFiDrv::analogWrite(27, 128);  // for configurable brightness
  } else {
    Serial.println("LED off");
    WiFiDrv::analogWrite(25, 0);  // for configurable brightness
    WiFiDrv::analogWrite(26, 0);  // for configurable brightness
    WiFiDrv::analogWrite(27, 0);  // for configurable brightness
  }
}
