/*
Demo Two Part One Code
Desc: Code makes bot wait for I2C communication and get within 1 foot of marker.
*/
#include <Wire.h>

#define M1A 2 // motor 1 encoder
#define M2A 3 // motor 2 encoder
#define ENABLE_PIN 4 // motor driver enable
#define M1B 5 // motor 1 encoder
#define M2B 6 // motor 2 encoder
#define M1DIR 7 // motor 1 direction input (voltage sign)
#define M2DIR 8 // motor 2 direction input (voltage sign)
#define M1PWR 9 // motor 1 speed input (voltage)
#define M2PWR 10 // motor 2 speed input (voltage)
#define DESIRED_TIME_US 10000 // 10 ms
#define BATTERY_VOLTAGE 8 // voltage of motor power supply
#define RADIUS 0.237861 // radius of wheel
#define WHEEL_DISTANCE 0.979 // distance between each wheel
#define I2C_ADDR 11 // I2C address

// I2C variables
volatile int8_t offset = 0;
volatile int8_t instruction[1] = {0};
volatile uint8_t msgLength = 0;
volatile uint8_t reply[1] = {0};

// motor voltage controller variables
float motor_one_max_voltage[2] = {5, -5};
float motor_two_max_voltage[2] = {5, -5};

// gains of system
float KP_RHO_POS = 3; // forward - 3
float KP_RHO = 50; // forward - 50
float KI_RHO = 0.05; // forward - .09 for 3/5 & .05 for 7 
float KP_PHI_POS = 30; // forward - 330
float KP_PHI = 3; // forward - 50
float KI_PHI = 0.05; // forward - .05

// global variables
unsigned long last_time_us, start_time_us; // timing
unsigned long last_read_time_motors_us[2] = {0, 0}; // last motor read time
float current_time_ms = 0; // current time
float rho_desired, rho_actual, rho_dot_desired, rho_dot_actual; // distance variables
float phi_desired, phi_actual, phi_dot_desired, phi_dot_actual, phi_desired_degrees; // angle variables
float V_bar, V_delta; // voltage variables
float count_diff, combined_count_diff, last_count_diff; // comparing counts to keep motor speeds together
long motor_counts_p[2] = {0, 0}; // private motor counts
long motor_counts[2] = {0, 0};  // public motor counts to be calc'd with
float theta[2] = {0, 0}; // rotational position
float theta_dot[2] = {0, 0}; // rotational velocity
float last_theta[2] = {0, 0}; // previous position in radians
float voltage[2] = {0, 0}; // voltage to motors
float error[2] = {0, 0}; // error of feedback loop
float pos_error[2] = {0, 0}; // error of position section
float integral_error[2] = {0, 0}; // error of integral section
float desired_pos_rad[2] = {0, 0}; // desired position of wheel
float actual_pos_rad[2] = {0, 0}; // current position in radians
float desired_velocity_rad_s[2] = {0, 0}; // desired velocity in rad/s
float actual_velocity_rad_s[2] = {0, 0}; // current velocity in rad/s
unsigned int pwm[2] = {0, 0}; // PWM applied to motors
bool marker_found = false; // global flag for if aruco marker is found by camera team
bool start_turning = false; // global flag for if robot should start searching for marker
bool reset_done = false; // global flag for if reset is done
float current_time_copy = 0; // mirror of current time for comparison reasons

// encoder ISR from assignment 1
void motor_one_encoder_isr(void) {
  if (micros() - last_read_time_motors_us[0] > 100) {
    if (digitalRead(M1A) == digitalRead(M1B)) {
      motor_counts_p[0] += 2;
    } else {
      motor_counts_p[0] -= 2;
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

// reset variables to track distances for different parts correctly
void reset(void) {
  motor_counts_p[0] = 0;
  motor_counts_p[1] = 0;
  phi_desired_degrees = 0;
  phi_actual = 0;
}

/******** I2C Helper Functions ********/
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

// function to request info from I2C
void request(void) {
  // According to the Wire source code, we must call write() within therequesting ISR
  // and nowhere else. Otherwise, the timing does not work out. See line238:
  // https://github.com/arduino/ArduinoCore-avr/blob/master/libraries/Wire/src/Wire.cpp
  msgLength = 0;
  
  Wire.write(reply[msgLength]); 
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
  // I2C config
  Wire.begin(I2C_ADDR);
  Wire.onReceive(receive);
  Wire.onRequest(request);
  Serial.begin(115200); // high baud rate to zoom
  rho_desired = 0;
  phi_desired = 0;
  phi_desired_degrees = 0;
}

void loop() {
  current_time_ms = (float)(last_time_us - start_time_us) / 1000;
  // 1 inch = 0.08333 feet
  // I2C communication
  if (msgLength > 0) {
    if (instruction[0] == 100) {
      start_turning = true;
    } else if (instruction[0] == 50) {
      start_turning = false;
      reply[0] = 1; // when arduino requests info this is what is sent back
    } else {
      reset();
      marker_found = true;
      if (instruction[0] > 200) {
        phi_desired_degrees = (instruction[0] - 256);
        phi_desired = phi_desired_degrees * PI / 180;
      } else {
        phi_desired_degrees = instruction[0];
        phi_desired = phi_desired_degrees * PI / 180;
      }
    }
    msgLength = 0;
  }

  // once marker found go to marker
  if (marker_found && (phi_desired - phi_actual < 0.02)) {
    if (!reset_done) {
      reset();
    }
    rho_desired = 6.25;
    reset_done = true;
  }

  // finding camera code, every 3 seconds turn 20 degrees more if marker hasn't been found
  if (current_time_ms - current_time_copy >= 3000 && !marker_found && start_turning) {
    phi_desired_degrees += 40;
    current_time_copy = current_time_ms;
  }
  phi_desired = phi_desired_degrees * PI / 180; // convert from degrees to radians

  // get motor counts
  motor_counts[0] = encoder(1);
  motor_counts[1] = encoder(2);

  // motor voltage controller
  count_diff = abs(motor_counts[1]) - abs(motor_counts[0]);
  combined_count_diff = count_diff - last_count_diff;
  last_count_diff = count_diff;
  if (combined_count_diff > 0) { // if motor 2 going faster
    motor_one_max_voltage[0] = 5;
    motor_one_max_voltage[1] = -5;
    motor_two_max_voltage[0] = 1.75;
    motor_two_max_voltage[1] = -1.75;
  } else { // if motor 1 is going faster
    motor_one_max_voltage[0] = 2;
    motor_one_max_voltage[1] = -2;
    motor_two_max_voltage[0] = 4.5;
    motor_two_max_voltage[1] = -4.5;
  }
  
  // get position traveled
  theta[0] = 2 * PI * (float)motor_counts[0] / 3200;
  theta[1] = 2 * PI * (float)motor_counts[1] / 3200;
  // calculate velocity of each motor
  theta_dot[0] = (theta[0] - last_theta[0]) / ((float)DESIRED_TIME_US / 1000000);
  theta_dot[1] = (theta[1] - last_theta[1]) / ((float)DESIRED_TIME_US / 1000000);
  // actual positions and angles calculated
  rho_actual = RADIUS * (theta[0] + theta[1]) / 2;
  phi_actual = RADIUS * (theta[0] - theta[1]) / WHEEL_DISTANCE;
  // actual velocities calculated
  rho_dot_actual = RADIUS * (theta_dot[0] + theta_dot[1]) / 2;
  phi_dot_actual = RADIUS * (theta_dot[0] - theta_dot[1]) / WHEEL_DISTANCE;

  // angle position & velocity controller - PI -> P -> V_delta
  pos_error[0] = phi_desired - phi_actual;
  integral_error[0] = 0.5 * (integral_error[0] + pos_error[0]) * ((float)DESIRED_TIME_US / 1000000);
  phi_dot_desired = KP_PHI * pos_error[0] + KI_PHI * integral_error[0];
  error[0] = phi_dot_desired - phi_dot_actual;
  V_delta = error[0] * KP_PHI_POS;

  // distance position & velocity controller - PI -> P -> V_bar
  pos_error[1] = rho_desired - rho_actual;
  integral_error[1] = 0.5 * (integral_error[1] + pos_error[1]) * ((float)DESIRED_TIME_US / 1000000);
  rho_dot_desired = KP_RHO * pos_error[1] + KI_RHO * integral_error[1];
  error[1] = rho_dot_desired - rho_dot_actual;
  V_bar = error[1] * KP_RHO_POS;
  // calculate voltage
  voltage[0] = (V_bar + V_delta) / 2;
  voltage[1] = (V_bar - V_delta) / 2;

  //setting max voltages
  if (voltage[0] < 0) {
    voltage[0] = max(voltage[0], motor_one_max_voltage[1]);
  } else {
    voltage[0] = min(voltage[0], motor_one_max_voltage[0]);
  }
  if (voltage[1] < 0) {
    voltage[1] = max(voltage[1], motor_two_max_voltage[1]);
  } else {
    voltage[1] = min(voltage[1], motor_two_max_voltage[0]);
  }
  // calculate PWM
  pwm[0] = 255 * abs(voltage[0]) / BATTERY_VOLTAGE;
  pwm[1] = 255 * abs(voltage[1]) / BATTERY_VOLTAGE;
  // set motor direction
  if (voltage[0] > 0) {
    digitalWrite(M1DIR, LOW);
  } else {
    digitalWrite(M1DIR, HIGH);
  }
  if (voltage[1] > 0) {
    digitalWrite(M2DIR, HIGH);
  } else {
    digitalWrite(M2DIR, LOW);
  }
  // write power to motors
  analogWrite(M1PWR, min(pwm[0], 255));
  analogWrite(M2PWR, min(pwm[1], 255));
  // sample time and previous runs variable assignment
  while (micros() < last_time_us + DESIRED_TIME_US);
  last_time_us = micros();
  last_theta[0] = theta[0];
  last_theta[1] = theta[1];
}
