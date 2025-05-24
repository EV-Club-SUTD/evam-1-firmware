#include <EVAM.h>

unsigned long prevCoreMillis = 0; //timer for the important data service
unsigned long prevSlowMillis = 0; //timer for the less important data service
unsigned long prevMotorLockMillis = 0;  //timer for motor lock CAN Bus message

volatile bool lightSwitchOn = false;
volatile bool lightingISRFlag = false;
volatile unsigned long lastLightSwitchEvent = 0;

volatile bool boostMode = 0;
volatile bool revMode = 0;
volatile bool reverseISRFlag = false;
volatile bool boostISRFlag = false;
volatile unsigned long lastDriveModeSwitchEvent = 0;

//Sets the ESP32's GPIO Pins correctly for the connected switches
// to be run once in the Arduino Setup()
void setSwitchesGPIO(){
    pinMode(MOTOR_LOCK_PIN, INPUT);

    pinMode(LIGHTING_SWITCH_PIN, INPUT);
    pinMode(LIGHTING_LED_PIN, OUTPUT);

    //pinMode(BUZZER_OUT_PIN, OUTPUT);

    pinMode(REVERSE_SWITCH_PIN, INPUT);
    //pinMode(BOOST_SWITCH_PIN, INPUT);
    //pinMode(NORMAL_DRIVE_SWITCH_PIN, INPUT);
    // The above code should NOT BE USED!! NORMAL_DRIVE_SWITCH_PIN is GPIO1 which is UART Tx.
    // Setting it to high impedance will kill the serial monitor!
    // BOOST_SWITCH_PIN is GPIO3 which is UART Rx

    // Thankfully both of these pins aren't used anywhere else in the program, so we can choose not to use them
    pinMode(MOTOR_LOCK_PIN, INPUT);
}

//attaches interrupts for all the interrupt enabled switches
void attachInterrupts(){
    attachInterrupt(LIGHTING_SWITCH_PIN, lightingISR, RISING); //light switch is a pushbutton
    attachInterrupt(REVERSE_SWITCH_PIN, reverseISR, CHANGE);
    //attachInterrupt(BOOST_SWITCH_PIN, boostISR, CHANGE);
}

//Checks if the motor lock pin is used
//And sends the message on the CAN Bus
void checkSendMotorLockout(){
    motorLockMsg.data.u8[0] = digitalRead(MOTOR_LOCK_PIN);
    #ifdef SERIAL_DEBUG
    Serial.print("Motors Locked: ");
    Serial.println(motorLockMsg.data.u8[0]);
    #endif
    ESP32Can.CANWriteFrame(&motorLockMsg);
    #ifdef SERIAL_DEBUG
    Serial.print("Motor lock message sent.");
    #endif
}

//carries out any functions that need to be done before power is lost
void shutDown(){
    EEPROM.write(0, lightSwitchOn);
    EEPROM.commit();
}

void IRAM_ATTR lightingISR(){
    if((millis() - lastLightSwitchEvent) > LIGHT_SWITCH_DEBOUNCE_INTERVAL){
        lightSwitchOn = !lightSwitchOn;
        lightingISRFlag = true;  
    }
    lastLightSwitchEvent = millis();
}
void IRAM_ATTR reverseISR(){
    // if((millis() - lastDriveModeSwitchEvent) > DEBOUNCE_INTERVAL){
    //     revMode = digitalRead(REVERSE_SWITCH_PIN);
    //     reverseISRFlag = true;  
    // }
    reverseISRFlag = true; 
    lastDriveModeSwitchEvent = millis();
}
void IRAM_ATTR boostISR(){
    boostISRFlag = true; 
    lastDriveModeSwitchEvent = millis();
}

//re-enables interrupts after a DEBOUNCE_INTERVAL has elapsed
//acts as a software debounce for the interrupt-enabled buttons
//seems like disabling interrupts doesn't really work for some reason
// void checkReEnableInterrupts(unsigned long *_currentMillis){
//     if(*_currentMillis - lastLightSwitchEvent > LIGHT_SWITCH_DEBOUNCE_INTERVAL){
//         attachInterrupt(LIGHTING_SWITCH_PIN, lightingISR, RISING);
//     }
//     if(*_currentMillis - lastDriveModeSwitchEvent > DEBOUNCE_INTERVAL){
//         attachInterrupt(REVERSE_SWITCH_PIN, reverseISR, CHANGE);
//     }
// }
