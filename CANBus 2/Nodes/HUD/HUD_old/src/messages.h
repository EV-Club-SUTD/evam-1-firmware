/*
 * messages.h
 * 
 * CONTAINS ALL THE MESSAGES (BOTH CANBUS AND BLE) FOR THE HUD NODE
 * 
 */
#ifndef MESSAGES_H
#define MESSAGES_H

#include <hud_ble.h>    
#include <hud_can.h>

/********** MESSAGES FOR BLE CHARACTERISTICS **********/

/* Core message to be notified 
    coreMessage[0] = vel        //Car velocity, 0-255km/h
    coreMessage[1] = acc        //Accelerator percentage, 0-100%
    coreMessage[2] = brake      //Brake percentage, 0-100%
    coreMessage[3] = reverse    //Reverse: 0 or 1
    coreMessage[4] = steering   //Center: 128
*/
extern uint8_t coreMessage[5];

/* Node status message to be notified:
    statusMessage[0] = ecu          //engine control unit
    statusMessage[1] = bms          //battery management system
    statusMessage[2] = tps          //throttle (and brake) position sensor
    statusMessage[3] = sas          //steering angle sensor
    statusMessage[4] = imu          //inertial measurment unit
    statusMessage[5] = fw           //Front wheels
    statusMessage[6] = rlw          //Rear left wheel
    statusMessage[7] = rrw          //Rear right wheel
    statusMessage[8] = fl           //Front lights
    statusMessage[9] = rl           //Rear lights
    statusMessage[10] = interior    //interior lights
    0: error, 1: ok, 255: offline
*/
extern uint8_t statusMessage[11];


/* Battery status message to be notified 
    batteryMessage[0] =  battPercent    //Battery percentage
    batteryMessage[1] =  battVolt       //Battery voltage
    batteryMessage[2] =  battCurr_1     //Battery current part 1
    batteryMessage[3] =  battCurr_2     //Battery current part 2
    batteryMessage[4] =  battTemp       //Battery Temperature
*/
extern uint8_t batteryMessage[5];

/*  Lighting messages, removed
 *  Uses the lighting CAN message directly instead
 *  format: red, green, blue
 */


/********** MESSAGES FOR CAN BUS **********/
extern CAN_frame_t frontLightMsg;           //front light CAN message
extern CAN_frame_t rearLightMsg;            //rear light CAN message
extern CAN_frame_t intLightMsg;             //interior light CAN message
extern CAN_frame_t motorLockMsg;            //motor lock status
extern CAN_frame_t phoneConnectedMsg;       //phone connection status
extern CAN_frame_t reverseMsg;              //drive mode: reverse, normal, eco, boost
extern CAN_frame_t nodeStatusRequestMsg;    //CAN frame for HUD to request all nodes to report their statuses


void initMessageData();

void initCoreMsg();
void initBattMsg();
void initStatusMsg();

void initLightingMsg();
void initMotorLockMsg();
void initPhoneConnectedMsg();
void initReverseMsg();
void initNodeStatusRequestMessage();



#endif //MESSAGES_H