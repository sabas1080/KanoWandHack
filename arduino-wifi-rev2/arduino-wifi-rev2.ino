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
#include "SparkFunLSM6DS3.h"
#include <MadgwickAHRS.h>

LSM6DS3 myIMU( SPI_MODE, SPIIMU_SS );  //SPIIMU_SS is the CS pin for Arduino WiFi Rev2

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

long previousMillis = 0;  // last time checked, in ms
int oldBatteryLevel = 0;  // last battery level reading from analog input

Madgwick filter;
unsigned long microsPerReading, microsPrevious;
float accelScale, gyroScale;
    
void setup() {
  Serial.begin(9600);
  //while (!Serial);
  Serial.println("Processor came out of reset.\n");

  // start the IMU and filter
  myIMU.settings.gyroSampleRate = 26;   //Hz.  Can be: 13, 26, 52, 104, 208, 416, 833, 1666
  myIMU.settings.accelSampleRate = 26;  //Hz.  Can be: 13, 26, 52, 104, 208, 416, 833, 1666, 3332, 6664, 13330
  
  filter.begin(26);

  // Set the accelerometer range to 2G
  myIMU.settings.accelRange = 2;      //Max G force readable.  Can be: 2, 4, 8, 16
  // Set the gyroscope range to 250 degrees/second
  myIMU.settings.gyroRange = 245;   //Max deg/s.  Can be: 125, 245, 500, 1000, 2000

  pinMode(buttonA, INPUT_PULLUP);

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

  //Call .begin() to configure the IMU
  myIMU.begin();

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

  // assign event handlers for characteristic
  sensorQuaternionsChar.setEventHandler(BLERead, MotionCharacteristicWritten);
  // set an initial value for the characteristic
  sensorQuaternionsChar.setValue(0);

  //Defualt Factory
  organisationChar.setValue("Kano Computing");
  informationSwChar.setValue("1.0.4");
  informationHwChar.setValue("3");

  /* Start advertising BLE.  It will start continuously transmitting BLE
     advertising packets and will be visible to remote BLE central devices
     until it receives a new connection */

  // start advertising
  BLE.advertise();

  Serial.println("Bluetooth device active, waiting for connections...");

  //initialize variables to pace updates to correct rate
  microsPerReading = 1000000 / 26;
  microsPrevious = micros();
}

void loop() {
  // poll for BLE events
  BLE.poll();
  
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
  } else {
    Serial.println("LED off");
  }
}

void MotionCharacteristicWritten(BLEDevice central, BLECharacteristic characteristic)  {
  int aix, aiy, aiz;
  int gix, giy, giz;
  float ax, ay, az;
  float gx, gy, gz;
  float roll, pitch, heading;
  unsigned long microsNow;

  // check if it's time to read data and update the filter
  microsNow = micros();
  if (microsNow - microsPrevious >= microsPerReading) {
    
    // read raw data from IMU and convert from raw data to gravity and degrees/second units
    //Get all parameters
    ax = myIMU.readFloatAccelX();
    ay = myIMU.readFloatAccelY();
    az = myIMU.readFloatAccelZ();
    gx = myIMU.readFloatGyroX();
    gy = myIMU.readFloatGyroY();
    gz = myIMU.readFloatGyroZ();
    
    // update the filter, which computes orientation
    filter.updateIMU(gx, gy, gz, ax, ay, az);

    // print the heading, pitch and roll
    roll = filter.getRoll();
    pitch = filter.getPitch();
    heading = filter.getYaw();
    Serial.print("Orientation: ");
    Serial.print(heading);
    Serial.print(" ");
    Serial.print(pitch);
    Serial.print(" ");
    Serial.println(roll);

    // increment previous time, so we keep proper pace
    microsPrevious = microsPrevious + microsPerReading;
  }
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
    ioBatteryChar.writeValue(batteryLevel);  // and update the battery level characteristic
    oldBatteryLevel = batteryLevel;           // save the level for next comparison
  }
}
