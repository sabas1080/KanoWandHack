/************************************************************

Kano Wand Hack - DIY Wand

This example demonstrates how emulate an Kano Wand

Development environment specifics:
  IDE: Arduino 1.8.4
  Hardware Platform:
  - ESP32

Distributed as-is; no warranty is given.
************************************************************************/

#include <WiFi.h>
//Library BLE
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>
#include <BLE2902.h>

bool _BLEClientConnected = false;

#define WandhackService BLEUUID("64a7000bf6914b93a6f40968f5b648f8")
BLECharacteristic Characteristics1(BLEUUID("64a7000bf6914b93a6f40968f5b648f8"), BLECharacteristic::PROPERTY_READ );
BLECharacteristic Characteristics2(BLEUUID("64a70013f6914b93a6f40968f5b648f8"), BLECharacteristic::PROPERTY_READ);
BLECharacteristic Characteristics3(BLEUUID("64a70001f6914b93a6f40968f5b648f8"), BLECharacteristic::PROPERTY_READ);

//BLEDescriptor Descriptor1(BLEUUID((uint16_t)0x290C));

class MyServerCallbacks : public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) {
      _BLEClientConnected = true;
    };

    void onDisconnect(BLEServer* pServer) {
      _BLEClientConnected = false;
    }
};

void InitBLE() {
  BLEDevice::init("Kano-Wand-34-78-89");
  // Create the BLE Server
  BLEServer *pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());

  // Create the BLE Service
  BLEService *pWandhack = pServer->createService(WandhackService);

//Caracteristic
  pWandhack->addCharacteristic(&Characteristics1);
  //Descriptor.setValue("Position 0 - 6");
  //Characteristics1.addDescriptor(&Descriptor1);
//Caracteristic
  pWandhack->addCharacteristic(&Characteristics2);
  //Characteristics2.addDescriptor(&Descriptor3);
//Caracteristic
 pWandhack->addCharacteristic(&Characteristics3);
 //Characteristics3.addDescriptor(&Descriptor3);

  pServer->getAdvertising()->addServiceUUID(WandhackService);
  pWandhack->start();

  // Start advertising
  pServer->getAdvertising()->start();
}

void setup () {
  Serial.begin(9600);
  
  Serial.println("Ready Wandhack");
 
  InitBLE();

}

void loop () {
 

}





