// ===== can_data_parser.h =====
#ifndef __CAN_DATA_PARSER_H
#define __CAN_DATA_PARSER_H

#include <Arduino.h>
#include "driver/twai.h"

// ===== CAN Message IDs =====
#define CAN_ID_ECU_STATUS       0x00000008
#define CAN_ID_BATTERY_STATS    0x00000024
#define CAN_ID_FL_Wheel_Speed   0x00000034
#define CAN_ID_FR_Wheel_Speed   0x00000035
#define CAN_ID_RL_Wheel_Speed   0x00000036
#define CAN_ID_RR_Wheel_Speed   0x00000037
#define CAN_ID_VEHICLE_Speed    0x00000038

// ===== Vehicle Data Structure =====
typedef struct {
    // --- 0x08 - ECU Status ---
    uint8_t ecu_byte0;
    uint8_t ecu_byte1;
    bool ecu_valid;
    float arc_value;
    
    // --- 0x24 - Battery Data ---
    uint8_t data_0x24[8];
    bool data_0x24_valid;
    float battery_voltage;      
    float battery_current;      
    uint8_t highest_cell_temp;  
    float SOC;
    
    // --- 0x34 - Front Left Wheel ---
    uint8_t data_0x34[8];
    bool data_0x34_valid;
    float wheel_fl_rpm;
    float wheel_fl_km;        
    
    // --- 0x35 - Front Right Wheel ---
    uint8_t data_0x35[8];
    bool data_0x35_valid;
    float wheel_fr_rpm; 
    float wheel_fr_km;          
    
    // --- 0x36 - Rear Left Wheel (Back Left) ---
    uint8_t data_0x36[8];
    bool data_0x36_valid;
    float wheel_bl_rpm;    
    float wheel_bl_km;       
    
    // --- 0x37 - Rear Right Wheel (Back Right) ---
    uint8_t data_0x37[8];
    bool data_0x37_valid;
    float wheel_br_rpm; 
    float wheel_br_km;          
    
    // --- 0x38 - Speed ---
    uint8_t data_0x38[8];
    bool data_0x38_valid;
    float speed_kmh;            
    
    unsigned long last_update;
} VehicleData_t;

// ===== Global Vehicle Data =====
extern VehicleData_t vehicleData;

// ===== Minimal dirty flag so UI can update immediately when parser writes new data =====
extern volatile bool vehicleData_dirty;

// ===== Function Declarations =====
void can_data_init();
void can_data_parse(twai_message_t &message);
bool can_data_is_fresh(unsigned long timeout_ms);
float get_arc_value_from_ecu(uint8_t ecu_byte);

#endif // __CAN_DATA_PARSER_H