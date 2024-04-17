#include <Wire.h>

#define ARD_ADDR 11
#define PI_ADDR 1

// volatile int8_t offset = 0;
volatile int8_t instruction[1] = {0};
volatile uint8_t msgLength = 0;
volatile uint8_t reply[1] = {0};

void receive(void) {
  // Set the offset, this will always be the first byte.
  // offset = Wire.read();
  // If there is information after the offset, it is telling us more about the command.
  msgLength = 0;
  while (Wire.available()) {
    instruction[msgLength] = Wire.read();
    reply[msgLength] = instruction[msgLength];
    msgLength++;
  }
}

// function to request info from I2C
void request(void) {
  // According to the Wire source code, we must call write() within the requesting ISR
  // and nowhere else. Otherwise, the timing does not work out. See line238:
  // https://github.com/arduino/ArduinoCore-avr/blob/master/libraries/Wire/src/Wire.cpp
  msgLength = 0;
  
  Wire.write(reply[msgLength]); 
}

void setup() {
  // put your setup code here, to run once:
  Wire.begin(ARD_ADDR);
  Wire.onRequest(request);
  Wire.onReceive(receive);
  Serial.begin(9600);
}

void loop() {
  if (msgLength > 0) {
    switch(instruction[0]) {
      case 100:
        Serial.println("got 100, sending 1");
        reply[0] = 1;
        break;
      case 50:
        Serial.println("got 50, sending 0");
        reply[0] = 0;
        break;
    }
  }
  delay(100);
}
