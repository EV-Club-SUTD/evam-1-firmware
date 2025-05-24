/*
 * HUD (Dashboard) NODE FOR EVAM

(Based on Neil Kolban example for IDF: https://github.com/nkolban/esp32-snippets/blob/master/cpp_utils/tests/BLE%20Tests/SampleNotify.cpp
Ported to Arduino ESP32 by Evandro Copercini
updated by chegewara)

FUNCTIONS:
- Reads CAN messages from car and updates BLE charactersitics (see excel)
-- Status of CAN nodes
-- Code data such as speed, battery %, current, etc
- Reads lighting data from BLE and updates car CAN
- Monitors wheel lock out switch and sends relevant CAN message
- Monitors reverse switch and sends out relevant CAN message

TODO / Bugs: 
- Check why battery current is capped at 297A

Designed to run on an ESP32 (ARDUINO_AVR_NANO)


!!THIS CODE HAS NO OVERFLOW PROTECTION!!
(since the car isn't expected to remain on for 50 days consecutively)
*/

#include <EVAM.h>

//#include <utils.h>

/*************** SETUP ***************/
void setup()
{
  #ifdef SERIAL_DEBUG
  Serial.begin(115200);
  Serial.println("EVAM HUD (Dashboard) Node");
  #ifndef ARDUINO_ESP32_DEV
  Serial.println("warning: This code was designed for the ESP32 Dev board.");
  #endif //ndef ARDUINO_ESP32_DEV
  #endif //SERIAL_DEBUG

  /* CAN Setup */
  int res = canSetup();

  #ifdef SERIAL_DEBUG
  if(res == 0){
    Serial.println("CAN Set Up Successful");
  }
  else{
    Serial.println("CAN Set Up Failed!");
  }
  #endif

  /* Message Set up */
  initMessageData();


  /* BLE Setup */
  setupBLE();
  #ifdef SERIAL_DEBUG
  Serial.println("Connect to bluetooth device: 'EVAM'.");
  #endif //SERIAL_DEBUG

  setStatusCharacteristic();
  setLightingCharacteristic();

  #ifdef SERIAL_DEBUG
  Serial.println("Status and lighting characteristics set.");
  #endif //SERIAL_DEBUG

  /* GPIO SETUP */
  setSwitchesGPIO();  
  attachInterrupts();

  #ifdef SERIAL_DEBUG
  Serial.println("GPIO and interrupts set.");
  #endif //SERIAL_DEBUG

  /* SET UP EEPROM */
  //EEPROM.begin(EEPROM_SIZE);
  //lightSwitchOn = EEPROM.read(0); //disabled until we figure out how to save the state just before shutdown. See function shutDown()
  lightSwitchOn = true;  //can eventually be changed to read the EEPROM
  requestCanNodeStatuses(); //initial request to all nodes for their status
  #ifdef SERIAL_DEBUG
  Serial.println("Requesting CAN node statuses");
  #endif //SERIAL_DEBUG
}


/*************** MAIN LOOP ***************/
void loop()
{
  // #ifdef SERIAL_DEBUG
  // Serial.println("Loop ran!");
  // #endif
  unsigned long currentMillis = millis();

  sendButtonCanMessages(&currentMillis);
  checkIncomingCanMessages(); 

  //update BLE server and notify for core data
  if (currentMillis - prevCoreMillis > CORE_DATA_REFRESH_INTERVAL)
  { 
    setBatteryCharacteristic();
    setCoreCharacteristic();
    prevCoreMillis = currentMillis;
  }

  //update and notify for additional low priority data
  if (currentMillis - prevSlowMillis > SLOW_DATA_REFRESH_INTERVAL)
  {
    /* if CANbus is connected, characteristic is updated whenever the data comes in
    updateStatusData();
    setStatusCharacteristicOld();
    */  
    prevSlowMillis = currentMillis;
  }

    //update and notify for additional low priority data
  if (currentMillis - prevMotorLockMillis > MOTOR_LOCK_MSG_REFRESH_INTERVAL)
  {
    checkSendMotorLockout();
    requestCanNodeStatuses(); // force a refresh of node statuses
    prevMotorLockMillis = currentMillis;
  }

  // disconnecting
  if (!deviceConnected && oldDeviceConnected)
  {
    delay(500);                  // give the bluetooth stack the chance to get things ready
    pServer->startAdvertising(); // restart advertising
    Serial.println("Start advertising...");
    oldDeviceConnected = deviceConnected;
  }
  // connecting
  if (deviceConnected && !oldDeviceConnected)
  {
    // do stuff here on connecting
    oldDeviceConnected = deviceConnected;
  }
}


/*  UPDATE WITH RANDOM DATA, FOR TESTING W/O CAN BUS

// Manually update core data for the core characteristic. 
void updateCoreData()
{
  //for now is hardcoded data
  // uint16_t accTemp = analogRead(ACC_PIN);
  // uint8_t accFinal = accTemp / 34.00;
  // vel = accFinal;
  // acc = accFinal;
  // brake = accFinal;
  // vel = rand() % 50 + 50;
  // acc = rand() % 50 + 50;
  // brake = rand() % 50 + 0;
    if (vel < 100)
  {
    vel++;
  }
  else
  {
    vel = 0;
  }
  if (acc < 100)
  {
    acc++;
  }
  else
  {
    acc = 0;
  }
  if (brake > 0)
  {
    brake--;
  }
  else
  {
    brake = 100;
  }
}
// Manually update CAN Bus node status for the status characteristic. 

void updateStatusData()
{
  ecu = 255;
  bms = 1;
  tps = 1;
  sas = 0;
  imu = 255;
  fw = 1;
  rlw = 0;
  rrw = 0;
  fl = 255;
  rl = 255;
  interior = 0;
}

// Manually update battery data for the battery characteristic.
void updateBatteryData()
{
  float battPhysicalCurr = randomFloatRange(300);
  battPercent = randomFloatRange(100) * 10;
  battVolt = randomFloatRange(75) * 10;
  battCurr = (battPhysicalCurr + 320) * 10;
  battTemp = randomFloatRange(50) * 10;
}
*/
/*************** SET CHARACTERISTICS TO NEW VALUES ***************/

/*
void setCoreCharacteristicOld()
{
  if (!(coreMessage[0] == vel && coreMessage[1] == acc && coreMessage[2] == brake) && deviceConnected)
  {

    coreMessage[0] = vel;
    coreMessage[1] = acc;
    coreMessage[2] = brake;
    pCoreCharacteristic->setValue((uint8_t *)coreMessage, sizeof(coreMessage));
    pCoreCharacteristic->notify();
  }
}
*/

/* Sets new data for node status characteristic and notifies
 * Used only for  testing without the canbus 
 */
/*
void setStatusCharacteristicOld()
{
  if (!(statusMessage[0] == ecu &&
        statusMessage[1] == bms &&
        statusMessage[2] == tps &&
        statusMessage[3] == sas &&
        statusMessage[4] == imu &&
        statusMessage[5] == fw &&
        statusMessage[6] == rlw &&
        statusMessage[7] == rrw &&
        statusMessage[8] == fl &&
        statusMessage[9] == rl &&
        statusMessage[10] == interior) &&
      deviceConnected)
  {
    statusMessage[0] = ecu;
    statusMessage[1] = bms;
    statusMessage[2] = tps;
    statusMessage[3] = sas;
    statusMessage[4] = imu;
    statusMessage[5] = fw;
    statusMessage[6] = rlw;
    statusMessage[7] = rrw;
    statusMessage[8] = fl;
    statusMessage[9] = rl;
    statusMessage[10] = interior;
    pStatusCharacteristic->setValue((uint8_t *)statusMessage, sizeof(statusMessage));
    pStatusCharacteristic->notify();
  }
}
*/

/* Sets new data for battery characteristic and notifies
 * Used only for  testing without the canbus 
 */
/*
void setBatteryCharacteristicOld()
{
  uint8_t battCurr_1 = battCurr >> 8;
  uint8_t battCurr_2 = battCurr & 0x00FF;
  if (!(batteryMessage[0] == battPercent &&
        batteryMessage[1] == battVolt &&
        batteryMessage[2] == battCurr_1 &&
        batteryMessage[3] == battCurr_2 &&
        batteryMessage[4] == battTemp) &&
      deviceConnected)
  {
    batteryMessage[0] = battPercent;
    batteryMessage[1] = battVolt;
    batteryMessage[2] = battCurr_1;
    batteryMessage[3] = battCurr_2;
    batteryMessage[4] = battTemp;
    pBatteryCharacteristic->setValue((uint8_t *)batteryMessage, sizeof(batteryMessage));
    pBatteryCharacteristic->notify();
  }
}
*/