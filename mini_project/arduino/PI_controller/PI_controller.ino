/*
* Name: Sam Fuller & Eugene Lee
* Assignment: mini project
* Description: When a message is received from the PI via I2C rotate the correct motor
*              180 degrees depending on which command is received
* Note: Integer value and corresponding quadrant
* 0: NE, 1: NW, 2: SW, 3: SE
*/
#include <Wire.h> // I2C Library

/******** Pin & Address Defines ********/

#define M1A 2 // motor 1 encoder
#define M2A 3 // motor 2 encoder
#define ENABLE_PIN 4 // motor driver enable
#define M1B 5 // motor 1 encoder
#define M2B 6 // motor 2 encoder
#define M1DIR 7 // motor 1 direction input (voltage sign)
#define M2DIR 8 // motor 2 direction input (voltage sign)
#define M1PWR 9 // motor 1 speed input (voltage)
#define M2PWR 10 // motor 2 speed input (voltage)
#define I2C_ADDR 11 // I2C address

/******** I2C Global Variables ********/

// I2C variables
volatile uint8_t offset = 0;
volatile uint8_t instruction[1] = {0};
volatile uint8_t msgLength = 0;
volatile uint8_t reply[10] = {0};

/******** Motor & Timer Global Variables ********/

// motor control variables
const unsigned long desired_time_us = 10000; // desired sample time in us (10 ms)
const float battery_voltage = 7.8; // motor power supply
const float Kp = 2; // velocity gain
float Kp_pos[2] = {5, 3.5}; // P controller gain, unique for each motor
const float Ki_pos = 0.7; // I controller gain
unsigned long last_time_us, start_time_us; // timer stuff
float current_time_ms = 0; // more timer stuff
//bool finished = false; // for printing finished out to MatLab - not used currently
unsigned long last_read_time_motors_us[2] = {0, 0}; // stores the last time the motors were read
long motor_counts_p[2] = {0, 0}; // private motor counts
long motor_counts_pos[2] = {0, 0}; // public motor counts to be calc'd with
float last_pos_rad[2] = {0, 0}; // previous position in radians
float voltage[2] = {0, 0}; // voltage to motors
float error[2] = {0, 0}; // error of feedback loop
float pos_error[2] = {0, 0}; // error of position section
float integral_error[2] = {0, 0}; // error of integral section
float desired_pos_rad[2] = {0, 0}; // desired position of wheel
float actual_pos_rad[2] = {0, 0}; // current position in radians
float desired_velocity_rad_s[2] = {0, 0}; // desired velocity in rad/s
float actual_velocity_rad_s[2] = {0, 0}; // current velocity in rad/s
unsigned int pwm[2] = {0, 0}; // PWM applied to motors

/******** Motor Encoder Helper Functions ********/

// encoder ISR from assignment 1
void motor_one_encoder_isr(void) {
  if (micros() - last_read_time_motors_us[0] > 100) {
    if (digitalRead(M1A) == digitalRead(M1B)) {
      motor_counts_p[0] -= 2;
    } else {
      motor_counts_p[0] += 2;
    }
    last_read_time_motors_us[0] = micros();
  }
}

void motor_two_encoder_isr(void) {
  if (micros() - last_read_time_motors_us[1] > 100) {
    if (digitalRead(M2A) == digitalRead(M2B)) {
      motor_counts_p[1] -= 2;
    } else {
      motor_counts_p[1] += 2;
    }
    last_read_time_motors_us[1] = micros();
  }
}

// encoder adjuster from assignment 1
long encoder(const int motor) {
  if (motor == 1) {
    if (digitalRead(M1A) != digitalRead(M1B)) return (motor_counts_p[0] + 1);
    else return motor_counts_p[0];
  } else {
    if (digitalRead(M2A) != digitalRead(M2B)) return (motor_counts_p[1] + 1);
    else return motor_counts_p[1];
  }
}

/******** I2C Helper Functions ********/

// function called when an I2C interrupt event happens
void receive(void) {
  // Set the offset, this will always be the first byte.
  offset = Wire.read();
  // If there is information after the offset, it is telling us more about the command.
  msgLength = 0;
  while (Wire.available()) {
    instruction[msgLength] = Wire.read();
    reply[msgLength] = instruction[msgLength];
    msgLength++;
  }
}

// function to request info from I2C - not used currently
void request(void) {
  // According to the Wire source code, we must call write() within therequesting ISR
  // and nowhere else. Otherwise, the timing does not work out. See line238:
  // https://github.com/arduino/ArduinoCore-avr/blob/master/libraries/Wire/src/Wire.cpp
  msgLength = 0;
  
  Wire.write(reply[msgLength]); 
}

// printReceived helps us see what data we are getting from the leader
void printReceived(void) {
// Print on serial console
  Serial.print("Instruction received: ");
  for (int i=0;i<msgLength;i++) {
    Serial.print(String(instruction[i])+"\t");
  }
}

void setup() {
  last_time_us = micros(); // timer setup
  start_time_us = last_time_us;
  // pin config
  pinMode(M1PWR, OUTPUT);
  pinMode(M1DIR, OUTPUT);
  pinMode(M2PWR, OUTPUT);
  pinMode(M2DIR, OUTPUT);
  pinMode(ENABLE_PIN, OUTPUT);
  digitalWrite(ENABLE_PIN, HIGH); // enable motor driver
  // ISR setup
  attachInterrupt(digitalPinToInterrupt(M1A), motor_one_encoder_isr, CHANGE);
  attachInterrupt(digitalPinToInterrupt(M2A), motor_two_encoder_isr, CHANGE);
  Serial.begin(115200); // high baud rate to zoom
  // I2C config
  Wire.begin(I2C_ADDR);
  Wire.onReceive(receive);
  Wire.onRequest(request);
  Serial.println("Ready!"); // MatLab ready up
  delay(3000);
}

void loop() {
  // read desired position from I2C - 180 degrees is 3.14 radians AKA pi.
  if (msgLength > 0) {
    switch(instruction[0]) {
      case 0:
        desired_pos_rad[0] = 0;
        desired_pos_rad[1] = 0;
        break;
      case 1:
        desired_pos_rad[0] = 0;
        desired_pos_rad[1] = PI;
        break;
      case 2:
        desired_pos_rad[0] = PI;
        desired_pos_rad[1] = PI;
        break;
      case 3:
        desired_pos_rad[0] = PI;
        desired_pos_rad[1] = 0;
        break;  
    }
    msgLength = 0;
  }
  
  desired_pos_rad[0] = PI;
  desired_pos_rad[1] = PI;

  motor_counts_pos[0] = encoder(1); // get motor counts
  motor_counts_pos[1] = encoder(2);

  actual_pos_rad[0] = 2 * PI * (float)motor_counts_pos[0] / 3200; // amount traveled in radians
  actual_pos_rad[1] = 2 * PI * (float)motor_counts_pos[1] / 3200;

  actual_velocity_rad_s[0] = (actual_pos_rad[0] - last_pos_rad[0]) / ((float)desired_time_us / 1000000); // angular velocity in rad/s
  actual_velocity_rad_s[1] = (actual_pos_rad[1] - last_pos_rad[1]) / ((float)desired_time_us / 1000000);

  current_time_ms = (float)(last_time_us - start_time_us) / 1000; // convert time to ms

  // motor controllers
  for (int i = 0; i < 2; i++) {
    pos_error[i] = desired_pos_rad[i] - actual_pos_rad[i]; // position feedback loop, calculating error
    integral_error[i] = integral_error[i] + pos_error[i] * ((float)desired_time_us / 1000000); // calculating integral error
    desired_velocity_rad_s[i] = Kp_pos[i] * pos_error[i] + Ki_pos * integral_error[i]; // getting desired velocity using PI controller
    error[i] = desired_velocity_rad_s[i] - actual_velocity_rad_s[i]; // getting error for voltage
    voltage[i] = Kp * error[i]; // getting voltage using gain and error
    pwm[i] = 255 * abs(voltage[i]) / battery_voltage; // set motor PWM
  }

  // set motor directions depending on motor voltage sign
  if (voltage[0] > 0) {
    digitalWrite(M1DIR, HIGH);
  } else {
    digitalWrite(M1DIR, LOW);
  }
  if (voltage[1] > 0) {
    digitalWrite(M2DIR, HIGH);
  } else {
    digitalWrite(M2DIR, LOW);
  }

  analogWrite(M1PWR, min(pwm[0], 255)); // write PWM to both motors
  analogWrite(M2PWR, min(pwm[1], 255));

  // print out status variables forever
  Serial.print(current_time_ms);
  Serial.print("\t");
  Serial.print(desired_pos_rad[0]);
  Serial.print("\t");
  Serial.print(desired_pos_rad[1]);
  Serial.print("\t");
  Serial.print(desired_velocity_rad_s[0]);
  Serial.print("\t");
  Serial.print(desired_velocity_rad_s[1]);
  Serial.print("\t");
  Serial.print(voltage[0]);
  Serial.print("\t");
  Serial.print(voltage[1]);
  Serial.print("\t");
  Serial.print(actual_pos_rad[0]);
  Serial.print("\t");
  Serial.println(actual_pos_rad[1]);

  while (micros() < last_time_us + desired_time_us); // sample rate things
  last_time_us = micros(); // set comparison variables
  // setting previous variables for calculating delta in feedback loop
  last_pos_rad[0] = actual_pos_rad[0];
  last_pos_rad[1] = actual_pos_rad[1];
}
