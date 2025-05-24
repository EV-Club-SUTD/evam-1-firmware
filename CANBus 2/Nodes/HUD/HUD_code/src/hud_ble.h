/* 
    hud_ble.h

    BLE Characteristics and Stuff for the HUD Node, for EVAM

*/

#ifndef HUD_BLE_H
#define HUD_BLE_H

#include <Arduino.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <EVAM.h>

/* BLE Services and Characteristics */
#define CORE_SERVICE_UUID "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define STATUS_SERVICE_UUID "4ee1bbf0-5e71-4d58-9ce4-e3e45cb8d8f9"
#define LIGHTING_SERVICE_UUID "1cbef3f2-12d5-4490-8a80-7f7970b51b54"
#define CORE_CHARACTERISTIC_UUID "beb5483e-36e1-4688-b7f5-ea07361b26a8"
#define STATUS_CHARACTERISTIC_UUID "5d2e6e74-31f0-445e-8088-827c53b71166"
#define BATTERY_CHARACTERISTIC_UUID "3a9a4b03-75a4-48fb-a071-af568425664e"
#define FRONT_LIGHTING_CHARACTERISTIC_UUID "825eef3b-e3d0-4ca6-bef7-6428b7260f35"
#define REAR_LIGHTING_CHARACTERISTIC_UUID "db598591-bffe-46dd-9967-fbf869e88b3f"
#define INTERIOR_LIGHTING_CHARACTERISTIC_UUID "8ecaaefa-f62f-4376-a4c8-32c74f62d950"

extern BLEServer *pServer;           //main BLE server
extern BLEService *pCoreService;     //service for core data
extern BLEService *pStatusService;   //service for CANBus node status data
extern BLEService *pLightingService; //service for lighting data
extern BLECharacteristic *pCoreCharacteristic;
extern BLECharacteristic *pStatusCharacteristic;
extern BLECharacteristic *pBatteryCharacteristic;
extern BLECharacteristic *pFrontLightingCharacteristic;
extern BLECharacteristic *pRearLightingCharacteristic;
extern BLECharacteristic *pInteriorLightingCharacteristic;
// extern BLECharacteristic *pTurnIndicatorCharacteristic;
extern BLECharacteristicCallbacks *lightingCallback;

class ConnectionCallbacks : public BLEServerCallbacks
{
public:
  void onConnect(BLEServer *pServer);
  void onDisconnect(BLEServer *pServer);
};

class LightingUpdateCallback : public BLECharacteristicCallbacks
{
public:
  void onWrite(BLECharacteristic *pCharacteristic);
};

extern bool deviceConnected;
extern bool oldDeviceConnected;

void setCoreCharacteristic();
void setStatusCharacteristic();
void setBatteryCharacteristic();
void setLightingCharacteristic();
void setupBLE();



#endif //HUD_BLE_H