/*
    Based on Neil Kolban example for IDF: https://github.com/nkolban/esp32-snippets/blob/master/cpp_utils/tests/BLE%20Tests/SampleServer.cpp
    Ported to Arduino ESP32 by Evandro Copercini
    updates by chegewara
*/

#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>

// See the following for generating UUIDs:
// https://www.uuidgenerator.net/

#define SERVICE_UUID        "0001"
#define CHARACTERISTIC_UUID "0002"


struct test {
    uint16_t one;
    uint16_t two;
    uint16_t three;
    uint16_t four;
};

union tun {
    struct test t;
    byte b[sizeof(struct test)];
};

union tun t = {};
BLECharacteristic *pCharacteristic;

void setup() {


t.t.one = 1;
t.t.two = 99;
t.t.three = 100;
t.t.four = 1500;


  Serial.begin(115200);
  Serial.println("Starting BLE work!");

  BLEDevice::init("quad");
  BLEServer *pServer = BLEDevice::createServer();
  BLEService *pService = pServer->createService(SERVICE_UUID);
pCharacteristic = pService->createCharacteristic(
                                         CHARACTERISTIC_UUID,
                                         BLECharacteristic::PROPERTY_READ |
                                         BLECharacteristic::PROPERTY_WRITE |
                                         BLECharacteristic::PROPERTY_NOTIFY
                                       );

  char s[] = "Hello World says Neil testin this is a test just saying - Hello World says Neil testin this is a test just saying";
  Serial.println(sizeof(s));
  pCharacteristic->setValue(t.b, sizeof(t.b));
  pService->start();

  // BLEAdvertising *pAdvertising = pServer->getAdvertising();  // this still is working for backward compatibility
  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pAdvertising->setScanResponse(true);
  pAdvertising->setMinPreferred(0x06);  // functions that help with iPhone connections issue
  pAdvertising->setMinPreferred(0x12);
  BLEDevice::startAdvertising();
  Serial.println("Characteristic defined! Now you can read it in your phone!");
}

void loop() {
  // put your main code here, to run repeatedly:
  delay(50);
  t.t.one = t.t.one + 1;

    pCharacteristic->setValue(t.b, sizeof(t.b));
    pCharacteristic->notify(true);

    Serial.print(t.t.one);
    Serial.println(" - SEND");

}