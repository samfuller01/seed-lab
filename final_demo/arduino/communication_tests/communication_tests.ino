#include <Wire.h>

#define ARD_ADDR 11
#define PI_ADDR 1

// volatile int8_t offset = 0;
volatile int8_t instruction[2] = {0, 0};
volatile uint8_t msgLength = 0;
volatile uint8_t reply[2] = {0, 0};
int previous_message = 0;
int desired_angle, desired_distance;

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

const int commands[4] = {100, 85, 75, 50};
bool check_data(const int data) {
  for (int i = 0; i < 4; i++) {
    if (commands[i] == data) return false;
    else continue;
  }
  return true;
}

void loop() {
  if (msgLength > 0) {
    if (previous_message == 75) {
      if (check_data(instruction[0])) { // if instruction is command don't run
        desired_angle = instruction[0];
        previous_message = 0;
        reply[0] = 2;
        Serial.print("Angle: ");
        Serial.println(desired_angle);
      } else {
        previous_message = 0;
      }
    } else if (previous_message == 85) {
      if (check_data(instruction[0])) { // if instruction is command don't run
        desired_distance = instruction[0];
        previous_message = 0;
        reply[0] = 2;
        Serial.print("Distance: ");
        Serial.println(desired_distance);
      } else {
        previous_message = 0;
      }
    } else {
      previous_message = instruction[0];
      switch(instruction[0]) {
        case 100: // start turning command
          Serial.println("got 100, sending 0");
          reply[0] = 0;
          break;
        case 85: // distance to travel command
          Serial.println("got 85, sending 3");
          reply[0] = 3;
          break;
        case 75: // angle to turn command
          Serial.println("got 75, sending 1");
          reply[0] = 1;
          break;
        case 50: // stop turning command
          Serial.println("got 50, sending 0");
          reply[0] = 0;
          break;
        default:
          Serial.println("Command made no sense, default case");
          Serial.print("Got: ");
          Serial.println(instruction[0]);
          reply[0] = 0;
          break;
      }
    }
    msgLength = 0;
  }
  delay(100);
}
