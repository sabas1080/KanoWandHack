/************************************************************

Kano Wand Hack - DIY Wand

This example demonstrates how emulate an Kano Wand

Development environment specifics:
  IDE: Arduino 1.8.4
  Hardware Platform:
  - ESP32

Distributed as-is; no warranty is given.
************************************************************************/
//Library BLE
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>
#include <BLE2902.h>

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

BLEServer* pServer = NULL;

BLECharacteristic *pCharacteristicInformationOrganisationChar = NULL;
BLECharacteristic *pCharacteristicInformationSwChar = NULL;
BLECharacteristic *pCharacteristicInformationHwChar = NULL;
BLECharacteristic *pCharacteristicIOBatteryChar = NULL;
BLECharacteristic *pCharacteristicUserIOUserButtonChar = NULL;

BLECharacteristic *pCharacteristicIOVibratorChar = NULL;
BLECharacteristic *pCharacteristicIOLedChar = NULL;
BLECharacteristic *pCharacteristicIOKeepAliveChar = NULL;
BLECharacteristic *pCharacteristicSensorQuaternionsChar = NULL;
BLECharacteristic *pCharacteristicUserSensorRawChar = NULL;
BLECharacteristic *pCharacteristicSensorMotionChar = NULL;
BLECharacteristic *pCharacteristicSensorMagnCalibrateChar = NULL;
BLECharacteristic *pCharacteristicSensorQuaternionsResetChar = NULL;
BLECharacteristic *pCharacteristicSensorTempChar = NULL;


bool deviceConnected = false;
bool oldDeviceConnected = false;
uint8_t button = 0;
int oldBatteryLevel = 0;  // last battery level reading from analog input

class MyServerCallbacks : public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) {
      deviceConnected = true;
    };

    void onDisconnect(BLEServer* pServer) {
      deviceConnected = false;
    }
};

void InitBLE() {
  BLEDevice::init("Kano-Wand-75-78-89");
  // Create the BLE Server
  BLEServer *pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());

  // Create the BLE Service INFO
  BLEService *pServiceInfo = pServer->createService(INFO_SERVICE_UUID);

   pCharacteristicInformationOrganisationChar = pServiceInfo->createCharacteristic(
                                         BleUUIDInformationOrganisationChar,
                                         BLECharacteristic::PROPERTY_READ |
                                         BLECharacteristic::PROPERTY_NOTIFY
                                       );
    pCharacteristicInformationSwChar = pServiceInfo->createCharacteristic(
                                         BleUUIDInformationSwChar,
                                         BLECharacteristic::PROPERTY_READ |
                                         BLECharacteristic::PROPERTY_NOTIFY
                                       );
     pCharacteristicInformationHwChar = pServiceInfo->createCharacteristic(
                                         BleUUIDInformationHwChar,
                                         BLECharacteristic::PROPERTY_READ |
                                         BLECharacteristic::PROPERTY_NOTIFY
                                       );

// Create the BLE Service IO
  BLEService *pServiceIO = pServer->createService(IO_SERVICE_UUID);

  pCharacteristicIOBatteryChar = pServiceIO->createCharacteristic(
                                         BleUUIDIOBatteryChar,
                                         BLECharacteristic::PROPERTY_READ |
                                         BLECharacteristic::PROPERTY_NOTIFY
                                       );
    pCharacteristicUserIOUserButtonChar = pServiceIO->createCharacteristic(
                                         BleUUIDIOUserButtonChar,
                                         BLECharacteristic::PROPERTY_READ |
                                         BLECharacteristic::PROPERTY_NOTIFY
                                       );
     pCharacteristicIOVibratorChar = pServiceIO->createCharacteristic(
                                         BleUUIDIOVibratorChar,
                                         BLECharacteristic::PROPERTY_READ |
                                         BLECharacteristic::PROPERTY_NOTIFY
                                       );
                                       
     pCharacteristicIOLedChar = pServiceIO->createCharacteristic(
                                         BleUUIDIOLedChar,
                                         BLECharacteristic::PROPERTY_READ |
                                         BLECharacteristic::PROPERTY_WRITE
                                       );
     pCharacteristicIOKeepAliveChar = pServiceIO->createCharacteristic(
                                         BleUUIDIOKeepAliveChar,
                                         BLECharacteristic::PROPERTY_READ |
                                         BLECharacteristic::PROPERTY_NOTIFY
                                       );

 // Create the BLE Service SENSOR
  BLEService *pServiceSensor = pServer->createService(SENSOR_SERVICE_UUID);

  pCharacteristicSensorQuaternionsChar = pServiceSensor->createCharacteristic(
                                         BleUUIDSensorQuaternionsChar,
                                         BLECharacteristic::PROPERTY_READ |
                                         BLECharacteristic::PROPERTY_NOTIFY
                                       );
    pCharacteristicUserSensorRawChar = pServiceSensor->createCharacteristic(
                                         BleUUIDSensorRawChar,
                                         BLECharacteristic::PROPERTY_READ |
                                         BLECharacteristic::PROPERTY_NOTIFY
                                       );
     pCharacteristicSensorMotionChar = pServiceSensor->createCharacteristic(
                                         BleUUIDSensorMotionChar,
                                         BLECharacteristic::PROPERTY_READ |
                                         BLECharacteristic::PROPERTY_NOTIFY
                                       );
                                       
     pCharacteristicSensorMagnCalibrateChar = pServiceSensor->createCharacteristic(
                                         BleUUIDSensorMagnCalibrateChar,
                                         BLECharacteristic::PROPERTY_READ |
                                         BLECharacteristic::PROPERTY_NOTIFY
                                       );
     pCharacteristicSensorQuaternionsResetChar = pServiceSensor->createCharacteristic(
                                         BleUUIDSensorQuaternionsResetChar,
                                         BLECharacteristic::PROPERTY_READ |
                                         BLECharacteristic::PROPERTY_NOTIFY
                                       );
     pCharacteristicSensorTempChar = pServiceSensor->createCharacteristic(
                                         BleUUIDSensorTempChar,
                                         BLECharacteristic::PROPERTY_READ |
                                         BLECharacteristic::PROPERTY_NOTIFY
                                       );
    
  //Defualt Factory
  pCharacteristicInformationOrganisationChar->setValue("Kano");
  pCharacteristicInformationSwChar->setValue("1.1.5");
  pCharacteristicInformationHwChar->setValue("1");
  
  pServiceInfo->start();
  pServiceIO->start();
  pServiceSensor->start();
  
  //BLEAdvertising *pAdvertising = pServer->getAdvertising();
   pServer->getAdvertising()->start();
}

void setup () {
  Serial.begin(9600);
  Serial.println("Config...");
  pinMode(5,INPUT_PULLUP);
  
  InitBLE();
  
  Serial.println("Ready Wandhack");
}

void loop () {
    // notify changed value
    if (deviceConnected) {
      updateBatteryLevel();
      buttonstate();
    }
    // disconnecting
    if (!deviceConnected && oldDeviceConnected) {
        delay(500); // give the bluetooth stack the chance to get things ready
        pServer->startAdvertising(); // restart advertising
        Serial.println("start advertising");
        oldDeviceConnected = deviceConnected;
    }
    // connecting
    if (deviceConnected && !oldDeviceConnected) {
        // do stuff here on connecting
        oldDeviceConnected = deviceConnected;
    }
  delay(1000);
}

void buttonstate(){
  if(digitalRead(5)==0){
    button = 51; 
  }
  else{
    button = 0;
  }
  pCharacteristicUserIOUserButtonChar->setValue(&button,1);
  pCharacteristicUserIOUserButtonChar->notify();
}

void updateBatteryLevel() {
  /* Read the current voltage level on the A0 analog input pin.
     This is used here to simulate the charge level of a battery.
  */
  int battery = analogRead(A0);
  uint16_t batteryLevel = map(battery, 0, 1023, 0, 100);

  if (batteryLevel != oldBatteryLevel) {      // if the battery level has changed
    Serial.print("Battery Level % is now: "); // print it
    Serial.println(batteryLevel);

        uint8_t batteryData[2];
        //uint16_t batteryValue = (uint16_t)(10);
        batteryData[1] = 0;
        batteryData[0] = (uint16_t)(batteryLevel);
        pCharacteristicIOBatteryChar->setValue(batteryData, 2);   // and update the battery level characteristic
        pCharacteristicIOBatteryChar->notify();
        oldBatteryLevel = batteryLevel;           // save the level for next comparison
  }
}





