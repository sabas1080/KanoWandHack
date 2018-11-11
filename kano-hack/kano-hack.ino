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

bool _BLEClientConnected = false;

#define   INFO_SERVICE_UUID    "64a70010-f691-4b93-a6f4-0968f5b648f8"
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


class MyServerCallbacks : public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) {
      _BLEClientConnected = true;
    };

    void onDisconnect(BLEServer* pServer) {
      _BLEClientConnected = false;
    }
};

void InitBLE() {
  BLEDevice::init("Kano-Wand-75-78-89");
  // Create the BLE Server
  BLEServer *pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());

  // Create the BLE Service INFO
  BLEService *pServiceInfo = pServer->createService(INFO_SERVICE_UUID);

  BLECharacteristic *pCharacteristicInformationOrganisationChar = pServiceInfo->createCharacteristic(
                                         BleUUIDInformationOrganisationChar,
                                         BLECharacteristic::PROPERTY_READ |
                                         BLECharacteristic::PROPERTY_WRITE
                                       );
    BLECharacteristic *pCharacteristicInformationSwChar = pServiceInfo->createCharacteristic(
                                         BleUUIDInformationSwChar,
                                         BLECharacteristic::PROPERTY_READ |
                                         BLECharacteristic::PROPERTY_WRITE
                                       );
     BLECharacteristic *pCharacteristicInformationHwChar = pServiceInfo->createCharacteristic(
                                         BleUUIDInformationHwChar,
                                         BLECharacteristic::PROPERTY_READ |
                                         BLECharacteristic::PROPERTY_WRITE
                                       );

// Create the BLE Service IO
  BLEService *pServiceIO = pServer->createService(IO_SERVICE_UUID);

  BLECharacteristic *pCharacteristicIOBatteryChar = pServiceIO->createCharacteristic(
                                         BleUUIDIOBatteryChar,
                                         BLECharacteristic::PROPERTY_READ |
                                         BLECharacteristic::PROPERTY_WRITE
                                       );
    BLECharacteristic *pCharacteristicUserIOUserButtonChar = pServiceIO->createCharacteristic(
                                         BleUUIDIOUserButtonChar,
                                         BLECharacteristic::PROPERTY_READ |
                                         BLECharacteristic::PROPERTY_WRITE
                                       );
     BLECharacteristic *pCharacteristicIOVibratorChar = pServiceIO->createCharacteristic(
                                         BleUUIDIOVibratorChar,
                                         BLECharacteristic::PROPERTY_READ |
                                         BLECharacteristic::PROPERTY_WRITE
                                       );
                                       
     BLECharacteristic *pCharacteristicIOLedChar = pServiceIO->createCharacteristic(
                                         BleUUIDIOLedChar,
                                         BLECharacteristic::PROPERTY_READ |
                                         BLECharacteristic::PROPERTY_WRITE
                                       );
     BLECharacteristic *pCharacteristicIOKeepAliveChar = pServiceIO->createCharacteristic(
                                         BleUUIDIOKeepAliveChar,
                                         BLECharacteristic::PROPERTY_READ |
                                         BLECharacteristic::PROPERTY_WRITE
                                       );

 // Create the BLE Service SENSOR
  BLEService *pServiceSensor = pServer->createService(SENSOR_SERVICE_UUID);

  BLECharacteristic *pCharacteristicSensorQuaternionsChar = pServiceSensor->createCharacteristic(
                                         BleUUIDSensorQuaternionsChar,
                                         BLECharacteristic::PROPERTY_READ |
                                         BLECharacteristic::PROPERTY_WRITE
                                       );
    BLECharacteristic *pCharacteristicUserSensorRawChar = pServiceSensor->createCharacteristic(
                                         BleUUIDSensorRawChar,
                                         BLECharacteristic::PROPERTY_READ |
                                         BLECharacteristic::PROPERTY_WRITE
                                       );
     BLECharacteristic *pCharacteristicSensorMotionChar = pServiceSensor->createCharacteristic(
                                         BleUUIDSensorMotionChar,
                                         BLECharacteristic::PROPERTY_READ |
                                         BLECharacteristic::PROPERTY_WRITE
                                       );
                                       
     BLECharacteristic *pCharacteristicSensorMagnCalibrateChar = pServiceSensor->createCharacteristic(
                                         BleUUIDSensorMagnCalibrateChar,
                                         BLECharacteristic::PROPERTY_READ |
                                         BLECharacteristic::PROPERTY_WRITE
                                       );
     BLECharacteristic *pCharacteristicSensorQuaternionsResetChar = pServiceSensor->createCharacteristic(
                                         BleUUIDSensorQuaternionsResetChar,
                                         BLECharacteristic::PROPERTY_READ |
                                         BLECharacteristic::PROPERTY_WRITE
                                       );
     BLECharacteristic *pCharacteristicSensorTempChar = pServiceSensor->createCharacteristic(
                                         BleUUIDSensorTempChar,
                                         BLECharacteristic::PROPERTY_READ |
                                         BLECharacteristic::PROPERTY_WRITE
                                       );

  pServiceInfo->start();
  pServiceIO->start();
  pServiceSensor->start();
  BLEAdvertising *pAdvertising = pServer->getAdvertising();
  //pServer->getAdvertising()->addServiceUUID(SERVICE_UUID);
  //pServer->getAdvertising()->addServiceUUID(pService);
  //pIO->start();
  //pServer->getAdvertising()->addServiceUUID(pSENSOR);
  //pSENSOR->start();

  // Start advertising
  pAdvertising->start();
}

void setup () {
  Serial.begin(9600);
  
  Serial.println("Ready Wandhack");
 
  InitBLE();

}

void loop () {
 

}





