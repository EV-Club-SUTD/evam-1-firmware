#ifndef HUD_CAN_H
#define HUD_CAN_H

#include <ESP32CAN.h>
#include <CAN_config.h>
#include <arduino.h>
#include <can_ids.h>
#include <EVAM.h>



//used for setVehicleLights
enum lightLocation_t{
    FRONT_LIGHT = 1,
    REAR_LIGHT  = 2,
    INT_LIGHT   = 3
};

int  canSetup();
void checkIncomingCanMessages();
void setVehicleLights(uint8_t *value, lightLocation_t location);
void sendButtonCanMessages(unsigned long *_currentMillis);
void sendAllLightingMessages();
void requestCanNodeStatuses(uint8_t node = 0);




#endif