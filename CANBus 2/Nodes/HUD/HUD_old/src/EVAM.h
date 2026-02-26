/* 
 * EVAM.h 
 */

#ifndef EVAM_H
#define EVAM_H

#define SERIAL_DEBUG    //flag to turn on/off Serial.print

#ifndef ARDUINO_ESP32_DEV
#warning "this code was designed for the ESP32 Dev board."
#endif //ndef ARDUINO_ESP32_DEV

#include <Arduino.h>
#include <EEPROM.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <hud_ble.h>    //contains all the BLE related code
#include <hud_can.h>    //contains all the CAN Bus related code
#include <messages.h>   //contains all the CAN Bus and BLE messages

//Connected pins
#define MOTOR_LOCK_PIN GPIO_NUM_23
#define LIGHTING_SWITCH_PIN GPIO_NUM_26
#define LIGHTING_LED_PIN GPIO_NUM_27
#define BUZZER_OUT_PIN GPIO_NUM_34
#define REVERSE_SWITCH_PIN GPIO_NUM_21          //p.s. accidentally labelled as "ECO" on the PCB
#define BOOST_SWITCH_PIN GPIO_NUM_3
#define NORMAL_DRIVE_SWITCH_PIN GPIO_NUM_1
//#define POWER_SENSE_PIN GPIO_NUM_22           //designed to connect to an ISOLATED circuit that opens when the main power swtich is opened. DO NOT CONNECT TO THE 100V battery circuit or you will see magic smoke

//Extra GPIOs in case we want to add (2) more buttons later
#define B1_LED_PIN GPIO_NUM_25                
#define B1_SWITCH_PIN GPIO_NUM_33
#define B2_LED_PIN GPIO_NUM_32
#define B2_SWITCH_PIN GPIO_NUM_35

//Timings
#define CORE_DATA_REFRESH_INTERVAL      16
#define SLOW_DATA_REFRESH_INTERVAL      100
#define MOTOR_LOCK_MSG_REFRESH_INTERVAL 1000
#define DEBOUNCE_INTERVAL               100      //interval to wait before accepting a new reading from a switch
#define LIGHT_SWITCH_DEBOUNCE_INTERVAL  1000      //interval to wait before accepting a new reading from a switch

extern unsigned long prevCoreMillis; //timer for the important data service
extern unsigned long prevSlowMillis; //timer for the less important data service
extern unsigned long prevMotorLockMillis; //timer for the less important data service

//Switch values

//motorLock is saved directly to the can bus message frame
extern volatile bool lightSwitchOn; //whether the light switch is on or off
extern volatile bool lightingISRFlag;
extern volatile unsigned long lastLightSwitchEvent;  //last time the light switch ISR was triggered

//EEPROM
#define EEPROM_SIZE 1




extern volatile bool boostMode;                                         //boost mode. Maybe change to ECO
extern volatile bool revMode;                                           //reverse mode
extern volatile bool reverseISRFlag;
extern volatile bool boostISRFlag;
extern volatile unsigned long lastDriveModeSwitchEvent; //last time the reverse or boost/eco ISR was triggered

void setSwitchesGPIO();
void attachInterrupts();
//void checkReEnableInterrupts(unsigned long *_currentMillis);
void checkSendMotorLockout();
void shutDown();

void IRAM_ATTR lightingISR();
void IRAM_ATTR reverseISR();
void IRAM_ATTR boostISR();


#endif //EVAM_H