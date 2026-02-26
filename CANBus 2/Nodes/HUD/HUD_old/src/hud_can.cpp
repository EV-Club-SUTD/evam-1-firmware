#include <hud_can.h>

CAN_device_t CAN_cfg;               // CAN Config
const int rx_queue_size = 10;       // Receive Queue size


//Sets up CAN Bus
//to be run in the Arduino setup function
int canSetup(){ 
  CAN_cfg.speed = CAN_SPEED_1000KBPS; // HACK: somehow 1000KBPS works with 8-bit micros at 500KBPS for some reason
  CAN_cfg.tx_pin_id = GPIO_NUM_5;
  CAN_cfg.rx_pin_id = GPIO_NUM_14;
  CAN_cfg.rx_queue = xQueueCreate(rx_queue_size, sizeof(CAN_frame_t));
  // Init CAN Module
  int res = ESP32Can.CANInit();
  return res;
}

/*************** UPDATE DATA FROM CAN BUS***************/

//checks for can messages and updates ESP's records of the data
//(or updates the BLE server immediately, in the case of the node statuses)
void checkIncomingCanMessages(){
  CAN_frame_t rx_frame;

  if (xQueueReceive(CAN_cfg.rx_queue, &rx_frame, 3 * portTICK_PERIOD_MS) == pdTRUE) {

    #ifdef SERIAL_DEBUG
    if(rx_frame.MsgID >= 8 && rx_frame.MsgID <= 19) {
      // Status message received, print debug information
      Serial.print(rx_frame.MsgID);
      Serial.print(":");
      Serial.println(rx_frame.data.u8[0]);
      Serial.print("Current status message = ");
      for(int i = 0; i < sizeof(statusMessage); i++)
          Serial.printf("%u ", statusMessage[i]);
      Serial.println("");
    }
    #endif

    //check status messages
    if(rx_frame.MsgID == ECU_STATUS_MSG_ID){
      if(statusMessage[0] != (rx_frame.data.u8[0])){
        statusMessage[0] = (rx_frame.data.u8[0]);
        setStatusCharacteristic();
      } 
    } else if(rx_frame.MsgID == BMS_STATUS_MSG_ID){
      if(statusMessage[1] != (rx_frame.data.u8[0])){
        statusMessage[1] = (rx_frame.data.u8[0]);
        setStatusCharacteristic();
      } 
    }
    else if(rx_frame.MsgID == TPS_STATUS_MSG_ID){
      if(statusMessage[2] != (rx_frame.data.u8[0])){
        statusMessage[2] = (rx_frame.data.u8[0]);
        setStatusCharacteristic();
      } 
    }
    else if(rx_frame.MsgID == SAS_STATUS_MSG_ID){
      if(statusMessage[3] != (rx_frame.data.u8[0])){
        statusMessage[3] = (rx_frame.data.u8[0]);
        setStatusCharacteristic();
      } 
    }
    else if(rx_frame.MsgID == IMU_STATUS_MSG_ID){
      if(statusMessage[4] != (rx_frame.data.u8[0])){
        statusMessage[4] = (rx_frame.data.u8[0]);
        setStatusCharacteristic();
      } 
    }
    else if(rx_frame.MsgID == FW_STATUS_MSG_ID){
      if(statusMessage[5] != (rx_frame.data.u8[0])){
        statusMessage[5] = (rx_frame.data.u8[0]);
        setStatusCharacteristic();
      } 
    }
    else if(rx_frame.MsgID == RLW_STATUS_MSG_ID){
      if(statusMessage[6] != (rx_frame.data.u8[0])){
        statusMessage[6] = (rx_frame.data.u8[0]);
        setStatusCharacteristic();
      } 
    }
    else if(rx_frame.MsgID == RRW_STATUS_MSG_ID){
      if(statusMessage[7] != (rx_frame.data.u8[0])){
        statusMessage[7] = (rx_frame.data.u8[0]);
        setStatusCharacteristic();
      } 
    }
    else if(rx_frame.MsgID == FL_STATUS_MSG_ID){
      if(statusMessage[8] != (rx_frame.data.u8[0])){
        statusMessage[8] = (rx_frame.data.u8[0]);
        setStatusCharacteristic();
      } 
    }
    else if(rx_frame.MsgID == RL_STATUS_MSG_ID){
      if(statusMessage[9] != (rx_frame.data.u8[0])){
        statusMessage[9] = (rx_frame.data.u8[0]);
        setStatusCharacteristic();
      } 
    }
    else if(rx_frame.MsgID == INT_STATUS_MSG_ID){
      if(statusMessage[10] != (rx_frame.data.u8[0])){
        statusMessage[10] = (rx_frame.data.u8[0]);
        setStatusCharacteristic();
      } 
    }

    //check Core Data
    else if(rx_frame.MsgID == THROTTLE_BRAKE_MSG_ID){
      uint16_t throttle = (uint16_t(rx_frame.data.u8[0]) * 4) / 10;
      coreMessage[1] = uint8_t(throttle); // throttle % - attempted non-float
      coreMessage[2] = uint8_t((rx_frame.data.u8[4] * 0.4)); // brake %
    }
    else if(rx_frame.MsgID == BATT_STATS_MSG_ID){
        uint16_t battVolt_16_t = (rx_frame.data.u8[0] + (rx_frame.data.u8[1]<<8))/10;
        batteryMessage[1] = (uint8_t)battVolt_16_t; //battVolt

        //float rawBattCurr = - 320 + (canMsg.data[2] + canMsg.data[3]<<8)*0.1; //the 2 bits are directly passed to the ble server now
        batteryMessage[2] = rx_frame.data.u8[2]; //battCurr_1
        batteryMessage[3] = rx_frame.data.u8[3]; //battCurr_2
        batteryMessage[0] = rx_frame.data.u8[6]; //battPercent
        batteryMessage[4] = rx_frame.data.u8[7]-40; //battTemp

    } 
    else if(rx_frame.MsgID == VEH_SPEED_MSG_ID){
      coreMessage[0] = rx_frame.data.u8[1]; //just take the second byte (MSB) since we only need 8-bit resolution
    }
    else if(rx_frame.MsgID == STEERING_MSG_ID){
      coreMessage[4] = rx_frame.data.u8[0]; // calibrated steering angle from canbus
    }

    //check other data if required
    else if(rx_frame.MsgID == E_STOP_MSG_ID){
      //not sure what to do
      //coreMessage[0] = rx_frame.data.u8[1]; //just take the second byte (MSB) since we only need 8-bit resolution
    }
  } //if(xQueueReceive...
}


/* OUTPUT DATA TO CAN BUS */

/**
 * @brief Sets the vehicle light data (and updates CANBus if light switch is on)
 * @param [in] value (pointer to) the rgb value to be set
 * @param [in] location which light to set (FRONT_LIGHT, REAR_LIGHT, INT_LIGHT)
 */
void setVehicleLights(uint8_t *value, lightLocation_t location)
{
  Serial.print("RGB set to: ");

  //loop to set the individual bytes
  uint8_t lightArr[3];
  for (int i = 0; i < 3; i++)
  {
    lightArr[i] = *(value + i);
    Serial.print(lightArr[i]);
    Serial.print(" ");
  }
  Serial.println();
  switch (location){
    case FRONT_LIGHT:
      for(int i = 0; i < 3; i++){
        frontLightMsg.data.u8[i] = lightArr[i]; //sets the CAN frame data
      }
      
      if(lightSwitchOn){                        //if the light switch is on, send out the CAN Bus frame (otherwise just keep the saved frame)
        ESP32Can.CANWriteFrame(&frontLightMsg);
      }
      break;
    case REAR_LIGHT:
      for(int i = 0; i < 3; i++){
        rearLightMsg.data.u8[i] = lightArr[i];
      }
      if(lightSwitchOn){
        ESP32Can.CANWriteFrame(&rearLightMsg);
      }
      break;
    case INT_LIGHT:
      for(int i = 0; i < 3; i++){
        intLightMsg.data.u8[i] = lightArr[i];
      }
      if(lightSwitchOn){
        ESP32Can.CANWriteFrame(&intLightMsg);
      }
      break;
  } //switch
}

void sendAllLightingMessages(){
    ESP32Can.CANWriteFrame(&frontLightMsg);
    ESP32Can.CANWriteFrame(&rearLightMsg);
    ESP32Can.CANWriteFrame(&intLightMsg);
}

//checks for ISR flags of the various button presses, executes the logic, and sends the CAN messages
void sendButtonCanMessages(unsigned long *_currentMillis){

  //if lighting has just been switched on / off
  if(lightingISRFlag){
    #ifdef SERIAL_DEBUG
    Serial.print("Lights: ");
    Serial.println(lightSwitchOn);
    #endif
    if(!lightSwitchOn){
      //make array of zeros
      uint8_t lightsOffArray[3];
      for(int i = 0; i<3;i++){
        lightsOffArray[i] = 0;
      }
      //set all lights to zeros
      setVehicleLights(&lightsOffArray[0], FRONT_LIGHT);
      setVehicleLights(&lightsOffArray[0], REAR_LIGHT);
      setVehicleLights(&lightsOffArray[0], INT_LIGHT);
      sendAllLightingMessages();
    } else if(lightSwitchOn){
      uint8_t *valPtr = pFrontLightingCharacteristic->getData();
      setVehicleLights(valPtr, FRONT_LIGHT);
      valPtr = pRearLightingCharacteristic->getData();
      setVehicleLights(valPtr, REAR_LIGHT);
      valPtr = pInteriorLightingCharacteristic->getData();
      setVehicleLights(valPtr, INT_LIGHT);
    }
    digitalWrite(LIGHTING_LED_PIN, lightSwitchOn);
    lightingISRFlag = false;
  }

  //reverse
  if(reverseISRFlag && ((*_currentMillis - lastDriveModeSwitchEvent) > DEBOUNCE_INTERVAL)){
    revMode = digitalRead(REVERSE_SWITCH_PIN);
    Serial.print("Reverse: ");
    Serial.println(revMode);
    reverseMsg.data.u8[0] = revMode;
    ESP32Can.CANWriteFrame(&reverseMsg);
    reverseISRFlag = false;
    coreMessage[3] = revMode;   //update BLE message
  }
  
  //boost (template)
  /*
  if(boostISRFlag && ((*_currentMillis - lastDriveModeSwitchEvent) > DEBOUNCE_INTERVAL)){
    boostMode = digitalRead(BOOST_SWITCH_PIN);
    reverseMsg.data.u8[1] = boostMode;
    ESP32Can.CANWriteFrame(&reverseMsg);
    reverseISRFlag = false;
  }
  */
}

void requestCanNodeStatuses(uint8_t node){
  nodeStatusRequestMsg.data.u8[0] = node;
  ESP32Can.CANWriteFrame(&nodeStatusRequestMsg);
}
