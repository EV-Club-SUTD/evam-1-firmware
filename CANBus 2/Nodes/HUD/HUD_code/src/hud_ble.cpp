#include <hud_ble.h>

BLEServer *pServer;           //main BLE server
BLEService *pCoreService;     //service for core data
BLEService *pStatusService;   //service for CANBus node status data
BLEService *pLightingService; //service for lighting data
BLECharacteristic *pCoreCharacteristic;
BLECharacteristic *pStatusCharacteristic;
BLECharacteristic *pBatteryCharacteristic;
BLECharacteristic *pFrontLightingCharacteristic;
BLECharacteristic *pRearLightingCharacteristic;
BLECharacteristic *pInteriorLightingCharacteristic;
BLECharacteristicCallbacks *lightingCallback;

bool deviceConnected = false;
bool oldDeviceConnected = false;

/********** SETTING CHARACTERISTICS ***********/

/* Sets new data for core characteristic and notifies */
void setCoreCharacteristic()
{
    pCoreCharacteristic->setValue((uint8_t *)coreMessage, sizeof(coreMessage));
    pCoreCharacteristic->notify();
}

/* Sets new data for node status characteristic and notifies  */
void setStatusCharacteristic()
{
    pStatusCharacteristic->setValue((uint8_t *)statusMessage, sizeof(statusMessage));
    pStatusCharacteristic->notify();
  }

/* Sets new data for battery characteristic and notifies*/
void setBatteryCharacteristic()
{
    pBatteryCharacteristic->setValue((uint8_t *)batteryMessage, sizeof(batteryMessage));
    pBatteryCharacteristic->notify();
}

/* Sets new data for all three lighting charactersitics and notifies */
void setLightingCharacteristic()
{
    pFrontLightingCharacteristic->setValue((uint8_t *)frontLightMsg.data.u8, sizeof(frontLightMsg.data.u8));
    pRearLightingCharacteristic->setValue((uint8_t *)rearLightMsg.data.u8, sizeof(rearLightMsg.data.u8));
    pInteriorLightingCharacteristic->setValue((uint8_t *)intLightMsg.data.u8, sizeof(intLightMsg.data.u8));
    pBatteryCharacteristic->notify();
}

/*************** CALLBACKS ***************/

/* Callback functions for connection and disconnection of server */

void ConnectionCallbacks::onConnect(BLEServer *pServer){
    deviceConnected = true;

    #ifdef SERIAL_DEBUG
    Serial.println("Connected");
    #endif
    
    phoneConnectedMsg.data.u8[0]=deviceConnected;
    ESP32Can.CANWriteFrame(&phoneConnectedMsg);
  }

void ConnectionCallbacks::onDisconnect(BLEServer *pServer){
    deviceConnected = false;

    #ifdef SERIAL_DEBUG
    Serial.println("Disconnected");
    #endif

    phoneConnectedMsg.data.u8[0]=deviceConnected;
    ESP32Can.CANWriteFrame(&phoneConnectedMsg);
  }

/* Callback function for updated lighting data written from client */
void LightingUpdateCallback::onWrite(BLECharacteristic *pCharacteristic){
    uint8_t *valPtr = pCharacteristic->getData();

    if (pCharacteristic == pFrontLightingCharacteristic)
    {
      Serial.println("Setting front RGB...");
      setVehicleLights(valPtr, FRONT_LIGHT);
    }
    else if (pCharacteristic == pRearLightingCharacteristic)
    {
      Serial.println("Setting rear RGB...");
      setVehicleLights(valPtr, REAR_LIGHT);
    }
    else if (pCharacteristic == pInteriorLightingCharacteristic)
    {
      Serial.println("Setting interior RGB...");
      setVehicleLights(valPtr, INT_LIGHT);
    }
    else
    {
      Serial.println("No matching characteristic");
    }
}

/******** MAIN SETUP OF BLE**************/
void setupBLE(){
  // Create the BLE Device
  BLEDevice::init("EVAM");

  // Create the BLE Server
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new ConnectionCallbacks());

  // Create the BLE Service
  BLEService *pCoreService = pServer->createService(CORE_SERVICE_UUID);
  BLEService *pStatusService = pServer->createService(STATUS_SERVICE_UUID);
  BLEService *pLightingService = pServer->createService(LIGHTING_SERVICE_UUID);

  // Create BLE Characteristics
  pCoreCharacteristic = pCoreService->createCharacteristic(
      CORE_CHARACTERISTIC_UUID,
      BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY);

  pStatusCharacteristic = pStatusService->createCharacteristic(
      STATUS_CHARACTERISTIC_UUID,
      BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY);

  pBatteryCharacteristic = pStatusService->createCharacteristic(
      BATTERY_CHARACTERISTIC_UUID,
      BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY);

  pFrontLightingCharacteristic = pLightingService->createCharacteristic(
      FRONT_LIGHTING_CHARACTERISTIC_UUID,
      BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_WRITE);
  pRearLightingCharacteristic = pLightingService->createCharacteristic(
      REAR_LIGHTING_CHARACTERISTIC_UUID,
      BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_WRITE);
  pInteriorLightingCharacteristic = pLightingService->createCharacteristic(
      INTERIOR_LIGHTING_CHARACTERISTIC_UUID,
      BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_WRITE);
  // pTurnIndicatorCharacteristic = pLightingService->createCharacteristic(
  //     INTERIOR_LIGHTING_CHARACTERISTIC_UUID,
  //     BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_WRITE);

  lightingCallback = new LightingUpdateCallback();

  pFrontLightingCharacteristic->setCallbacks(lightingCallback);
  pRearLightingCharacteristic->setCallbacks(lightingCallback);
  pInteriorLightingCharacteristic->setCallbacks(lightingCallback);

  // Create BLE Descriptors
  pCoreCharacteristic->addDescriptor(new BLE2902());
  pStatusCharacteristic->addDescriptor(new BLE2902());
  pBatteryCharacteristic->addDescriptor(new BLE2902());
  pFrontLightingCharacteristic->addDescriptor(new BLE2902());
  pRearLightingCharacteristic->addDescriptor(new BLE2902());
  pInteriorLightingCharacteristic->addDescriptor(new BLE2902());

  // Start the service
  pCoreService->start();
  pStatusService->start();
  pLightingService->start();

  // Start advertising
  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(CORE_SERVICE_UUID);
  pAdvertising->setScanResponse(false);
  pAdvertising->setMinPreferred(0x0); // set value to 0x00 to not advertise this parameter
  BLEDevice::startAdvertising();
}