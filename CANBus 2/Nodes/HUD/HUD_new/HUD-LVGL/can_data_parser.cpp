#include "can_data_parser.h"
#include <string.h> // for memset

// ===== Initialize Global Vehicle Data =====
VehicleData_t vehicleData = {0};

// ===== Initialize the Data Structure =====
void can_data_init() 
{
    vehicleData.ecu_byte0 = 0;
    vehicleData.ecu_byte1 = 0;
    vehicleData.ecu_valid = false;

    vehicleData.data_0x24_valid = false;
    vehicleData.data_0x34_valid = false;
    vehicleData.data_0x35_valid = false;
    vehicleData.data_0x36_valid = false;
    vehicleData.data_0x37_valid = false;
    vehicleData.data_0x38_valid = false;

    vehicleData.battery_voltage = 0;
    vehicleData.battery_current = 0;
    vehicleData.highest_cell_temp = 0;
    vehicleData.SOC = 0;
    vehicleData.wheel_fl_rpm = 0;
    vehicleData.wheel_bl_rpm = 0;
    vehicleData.wheel_fr_rpm = 0;
    vehicleData.wheel_br_rpm = 0;
    vehicleData.speed_kmh = 0;

    float wheel_fl_km;
    float wheel_fr_km;
    float wheel_bl_km;
    float wheel_br_km;
    
    vehicleData.last_update = 0;
}

// ===== Parse CAN Messages and Extract Data =====
void can_data_parse(twai_message_t &message) {
    switch(message.identifier) {
        case CAN_ID_ECU_STATUS:  // 0x08
            if (message.data_length_code >= 2) {
                vehicleData.ecu_byte0 = message.data[0];
                vehicleData.ecu_byte1 = message.data[1];

                vehicleData.ecu_valid = true;
                vehicleData.last_update = millis();
                
                Serial.printf("[0x08] ECU Status - Byte0: 0x%02X (%d), Byte1: 0x%02X (%d)\n", 
                             vehicleData.ecu_byte0, vehicleData.ecu_byte0,
                             vehicleData.ecu_byte1, vehicleData.ecu_byte1);
            }
            break;
            
        case CAN_ID_BATTERY_STATS:  // 0x24
            for (int i = 0; i < 8; i++) {
                if (i < message.data_length_code) {
                    vehicleData.data_0x24[i] = message.data[i];
                } else {
                    vehicleData.data_0x24[i] = 0;  // Fill remaining with zeros
                }
            }

            vehicleData.data_0x24_valid = true;
            vehicleData.last_update = millis();
            
            // --- Calculate battery voltage: (B1*256 + B0)*0.1 ---
            vehicleData.battery_voltage = (vehicleData.data_0x24[1] * 256 + vehicleData.data_0x24[0]) * 0.1;
            // --- Calculate battery current: -320 + (B3*256 + B2)*0.1 ---
            vehicleData.battery_current = -320.0 + (vehicleData.data_0x24[3] * 256 + vehicleData.data_0x24[2]) * 0.1;
            // --- Calculate highest cell temp: B7 - 40 (assuming B8 means index 7) ---
            vehicleData.highest_cell_temp = (float)((int8_t)vehicleData.data_0x24[7]) - 40.0f;
            // --- Battery SOC ---
            vehicleData.SOC = (vehicleData.data_0x24[6]);

            Serial.printf("[0x24] BATTERY - Voltage: %.1fV, Current: %.1fA, Temp: %d°C, SOC: %.1f%\n",
                         vehicleData.battery_voltage, vehicleData.battery_current, vehicleData.highest_cell_temp, vehicleData.SOC);
            
            Serial.printf("[0x24] BATTERY Status - Byte0: 0x%02X (%d), Byte1: 0x%02X (%d), Byte2: 0x%02X (%d), Byte3: 0x%02X (%d), Byte4: 0x%02X (%d), Byte5: 0x%02X (%d), Byte6: 0x%02X (%d), Byte7: 0x%02X (%d)\n",
                         vehicleData.data_0x24[0], vehicleData.data_0x24[0],
                         vehicleData.data_0x24[1], vehicleData.data_0x24[1],
                         vehicleData.data_0x24[2], vehicleData.data_0x24[2],
                         vehicleData.data_0x24[3], vehicleData.data_0x24[3],
                         vehicleData.data_0x24[4], vehicleData.data_0x24[4],
                         vehicleData.data_0x24[5], vehicleData.data_0x24[5],
                         vehicleData.data_0x24[6], vehicleData.data_0x24[6],
                         vehicleData.data_0x24[7], vehicleData.data_0x24[7]);
            break;
            
        case CAN_ID_FL_Wheel_Speed:  // 0x34
            for (int i = 0; i < message.data_length_code && i < 8; i++) {
                vehicleData.data_0x34[i] = message.data[i];
            }

            vehicleData.data_0x34_valid = true;
            vehicleData.last_update = millis();

            // --- Calculate wheel speed: (B1*256 + B0)/30 RPM  (B1*256 + B0)/256 KM/H ---
            vehicleData.wheel_fl_rpm = (vehicleData.data_0x34[1] * 256 + vehicleData.data_0x34[0]) / 30.0;
            vehicleData.wheel_fl_km = (vehicleData.data_0x34[1] * 256 + vehicleData.data_0x34[0]) / 256;
            
            Serial.printf("[0x34] Front Left Wheel: %.1f RPM\n", vehicleData.wheel_fl_rpm);
            Serial.printf("[0x34] Front Left Wheel: %.1f km/h\n", vehicleData.wheel_fl_km);
            
            Serial.printf("[0x34] FRONT LEFT WHEEL Speed - Byte0: 0x%02X (%d), Byte1: 0x%02X (%d)\n",
                        vehicleData.data_0x34[0], vehicleData.data_0x34[0],
                        vehicleData.data_0x34[1], vehicleData.data_0x34[1]);
            break;
            
        case CAN_ID_FR_Wheel_Speed:  // 0x35
            for (int i = 0; i < message.data_length_code && i < 8; i++) {
                vehicleData.data_0x35[i] = message.data[i];
            }
            vehicleData.data_0x35_valid = true;
            vehicleData.last_update = millis();

            // --- Calculate Wheel Speed: (B1*256 + B0)/30 RPM  (B1*256 + B0)/256 KM/H ---  
            vehicleData.wheel_fr_rpm = (vehicleData.data_0x35[1] * 256 + vehicleData.data_0x35[0]) / 30.0;
            vehicleData.wheel_fr_km = (vehicleData.data_0x35[1] * 256 + vehicleData.data_0x35[0]) / 256;
            
            Serial.printf("[0x35] Front Right Wheel: %.1f RPM\n", vehicleData.wheel_fr_rpm);
            Serial.printf("[0x35] Front Right Wheel: %.1f km/h\n", vehicleData.wheel_fr_km);
            
            Serial.printf("[0x35] FRONT RIGHT WHEEL Speed - Byte0: 0x%02X (%d), Byte1: 0x%02X (%d)\n",
                        vehicleData.data_0x35[0], vehicleData.data_0x35[0],
                        vehicleData.data_0x35[1], vehicleData.data_0x35[1]);
            break;
            
        case CAN_ID_RL_Wheel_Speed:  // 0x36
            for (int i = 0; i < message.data_length_code && i < 8; i++) {
                vehicleData.data_0x36[i] = message.data[i];
            }
            vehicleData.data_0x36_valid = true;
            vehicleData.last_update = millis();
            
            // --- Calculate Wheel Speed: (B1*256 + B0)/30 RPM  (B1*256 + B0)/256 KM/H ---
            vehicleData.wheel_bl_rpm = (vehicleData.data_0x36[1] * 256 + vehicleData.data_0x36[0]) / 30.0;
            vehicleData.wheel_bl_km = (vehicleData.data_0x36[1] * 256 + vehicleData.data_0x36[0]) / 256;
            
            Serial.printf("[0x36] Rear Left Wheel: %.1f RPM\n", vehicleData.wheel_bl_rpm);
            Serial.printf("[0x36] Rear Left Wheel: %.1f km/h\n", vehicleData.wheel_bl_km);
            
            Serial.printf("[0x36] REAR LEFT WHEEL Speed - Byte0: 0x%02X (%d), Byte1: 0x%02X (%d)\n",
                        vehicleData.data_0x36[0], vehicleData.data_0x36[0],
                        vehicleData.data_0x36[1], vehicleData.data_0x36[1]);
            break;
            
        case CAN_ID_RR_Wheel_Speed:  // 0x37
            for (int i = 0; i < message.data_length_code && i < 8; i++) {
                vehicleData.data_0x37[i] = message.data[i];
            }
            vehicleData.data_0x37_valid = true;
            vehicleData.last_update = millis();
            
            // --- Calculate wheel speed: (B1*256 + B0)/30 RPM  (B1*256 + B0)/256 KM/H ---
            vehicleData.wheel_br_rpm = (vehicleData.data_0x37[1] * 256 + vehicleData.data_0x37[0]) / 30.0;
            vehicleData.wheel_br_km = (vehicleData.data_0x37[1] * 256 + vehicleData.data_0x37[0]) / 256;
            
            Serial.printf("[0x37] Rear Right Wheel: %.1f RPM\n", vehicleData.wheel_br_rpm);
            Serial.printf("[0x37] Rear Right Wheel: %.1f km/h\n", vehicleData.wheel_br_km);

            Serial.printf("[0x37] REAR RIGHT WHEEL Speed - Byte0: 0x%02X (%d), Byte1: 0x%02X (%d)\n",
                        vehicleData.data_0x37[0], vehicleData.data_0x37[0],
                        vehicleData.data_0x37[1], vehicleData.data_0x37[1]);
            break;
            
        case CAN_ID_VEHICLE_Speed:  // 0x38
            for (int i = 0; i < message.data_length_code && i < 8; i++) {
                vehicleData.data_0x38[i] = message.data[i];
            }
            vehicleData.data_0x38_valid = true;
            vehicleData.last_update = millis();
            
            // --- Calculate Overall Speed: (B1*256 + B0)/256 ---
            vehicleData.speed_kmh = (vehicleData.data_0x38[1] * 256 + vehicleData.data_0x38[0]) / 256.0;

            Serial.printf("[0x38] Overall Speed: %.1f km/h\n", vehicleData.speed_kmh);

            Serial.printf("[0x38] VEHICLE Overall Speed - Byte0: 0x%02X (%d), Byte1: 0x%02X (%d)\n",
                        vehicleData.data_0x38[0], vehicleData.data_0x38[0],
                        vehicleData.data_0x38[1], vehicleData.data_0x38[1]);

            break;
            
        default:
            // --- Other message IDs - ignore ---
            break;
    }
}

bool can_data_is_fresh(unsigned long timeout_ms) {
    // --- Check if we have ANY valid data ---
    bool any_valid = vehicleData.ecu_valid || 
                     vehicleData.data_0x24_valid || 
                     vehicleData.data_0x34_valid || 
                     vehicleData.data_0x35_valid || 
                     vehicleData.data_0x36_valid || 
                     vehicleData.data_0x37_valid || 
                     vehicleData.data_0x38_valid;
    
    if (!any_valid) return false;
    return (millis() - vehicleData.last_update) < timeout_ms;
}

float get_arc_value_from_ecu(uint8_t ecu_byte) {
    float arc_value;
    
    if (ecu_byte == 255) {
        arc_value = 100.0;  // Offline
        Serial.printf("ECU Status: Offline \n");
    } 
    else if (ecu_byte == 0) {
        arc_value = 100.0; // Error
        Serial.printf("ECU Status: Error \n");
    }   
    else if (ecu_byte == 1) {
        arc_value = 100.0; // OK
        Serial.printf("ECU Status: OK \n");
    } 
    else {
        arc_value = 100.0; // Default
    }
    
    return arc_value;
}
