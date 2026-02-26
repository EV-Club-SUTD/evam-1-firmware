#include <Wire.h>

const int SWITCH_PIN = 9;  // CHANGED from 9 to avoid I2C conflict
volatile bool switchState = 0;

unsigned long previousMillis = 0;  // Moved inside properly
unsigned long currentMillis = 0;   // Will be updated in loop

void setup() {
  pinMode(SWITCH_PIN, INPUT_PULLUP);
  Wire.begin(0x08);
  Wire.onReceive(receiveEvent);
  Wire.onRequest(requestEvent);
  Serial.begin(115200);
  Serial.println("Arduino Slave with 3-pin Switch Started");
}

void loop() {
  currentMillis = millis(); 
  if (currentMillis - previousMillis >= 500) {
    previousMillis = currentMillis;
    
    switchState = digitalRead(SWITCH_PIN);
    Serial.print("State of Switch: ");
    Serial.println(switchState);
  }
}  

void receiveEvent(int bytes) {
  String received = "";
  while (Wire.available()) {
    char c = Wire.read();
    received += c;
  }                       
  Serial.print("Received: ");
  Serial.println(received);
}

void requestEvent() {
  Wire.write(switchState);
  Serial.print("Sent switch state: ");
  Serial.println(switchState);
}