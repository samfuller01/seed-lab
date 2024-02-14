/*
* Name: Sam Fuller & Eugene Lee
* Assignment: mini project
* Description:
*/
#define M1A 2 // motor 1 encoder
#define M2A 3 // motor 2 encoder
#define ENABLE_PIN 4 // motor driver enable
#define M1B 5 // motor 1 encoder
#define M2B 6 // motor 2 encoder
#define M1DIR 7 // motor 1 direction input (voltage sign)
#define M2DIR 8 // motor 2 direction input (voltage sign)
#define M1PWR 9 // motor 1 speed input (voltage)
#define M2PWR 10 // motor 2 speed input (voltage)

// Integer value and corresponding quadrant
// 0: NE, 1: NW, 2: SW, 3: SE

const unsigned long desired_time_us = 10000; // desired sample time in us (10 ms)
const float battery_voltage = 7.8; // motor power supply
const float Kp = 2; // system gain
const float Kp_pos = 1;
const float Ki_pos = 1;
unsigned long last_time_us, start_time_us; // timer stuff
float current_time_ms = 0; // more timer stuff
bool finished = false; // for printing finished out to MatLab
unsigned long last_read_time_motors_us[2];
long motor_counts_p[2] = {0, 0}; // private motor counts
long motor_counts_pos[2] = {0, 0}; // public motor counts to be adjusted
float last_pos_rad[2] = {0, 0}; // previous position in radians
float voltage[2] = {0, 0}; // voltage to motors
float error[2] = {0, 0}; // error of feedback loop
float pos_error[2] = {0, 0}; // error of position loop
float integral_error[2] = {0, 0}; // error of integral section
float desired_pos_rad[2] = {0, 0}; // desired position of wheel
float actual_pos_rad[2] = {0, 0}; // current position in radians
float desired_velocity_rad_s[2] = {0, 0}; // desired velocity in rad/s
float actual_velocity_rad_s[2] = {0, 0}; // current velocity in rad/s
unsigned int pwm[2] = {0, 0}; // PWM applied to motors

// encoder ISR from assignment 1
void motor_one_encoder_isr() {
  if (micros() - last_read_time_motors_us[0] > 100) {
    if (digitalRead(M1A) == digitalRead(M1B)) {
      motor_counts_p[0] -= 2;
    } else {
      motor_counts_p[0] += 2;
    }
    last_read_time_motors_us[0] = micros();
  }
}

void motor_two_encoder_isr() {
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
  Serial.println("Ready!"); // MatLab ready up
}

void loop() {
  desired_pos_rad[0] = 3; // desired position for each wheel
  desired_pos_rad[1] = 3;

  motor_counts_pos[0] = encoder(1); // get motor counts
  motor_counts_pos[1] = encoder(2);

  actual_pos_rad[0] = 2 * PI * (float)motor_counts_pos[0] / 3200; // amount traveled in radians
  actual_pos_rad[1] = 2 * PI * (float)motor_counts_pos[1] / 3200;

  actual_velocity_rad_s[0] = (actual_pos_rad[0] - last_pos_rad[0]) / ((float)desired_time_us / 1000000); // angular velocity in rad/s
  actual_velocity_rad_s[1] = (actual_pos_rad[1] - last_pos_rad[1]) / ((float)desired_time_us / 1000000);

  current_time_ms = (float)(last_time_us - start_time_us) / 1000; // convert time to ms

  // motor controllers
  for (int i = 0; i < 2; i++) {
    pos_error[i] = desired_pos_rad[i] - actual_pos_rad[i];
    integral_error[i] = integral_error[i] + pos_error[i] * ((float)desired_time_us / 1000000);
    desired_velocity_rad_s[i] = Kp_pos * pos_error[i] + Ki_pos * integral_error[i];
    error[i] = desired_velocity_rad_s[i] - actual_velocity_rad_s[i];
    voltage[i] = Kp * error[i];
  }

  // set motor directions
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

  pwm[0] = 255 * abs(voltage[0]) / battery_voltage; // set motor PWMs
  pwm[1] = 255 * abs(voltage[1]) / battery_voltage;

  analogWrite(M1PWR, min(pwm[0], 255)); // write PWM to motor
  analogWrite(M2PWR, min(pwm[1], 255));

  // print out status variables
  Serial.print(current_time_ms);
  Serial.print("\t");
  Serial.print(desired_pos_rad[0]);
  Serial.print("\t");
  Serial.print(desired_velocity_rad_s[0]);
  Serial.print("\t");
  Serial.print(voltage[0]);
  Serial.print("\t");
  Serial.println(actual_pos_rad[0]);
  
  /*if (!finished) {
  }
  if (current_time_ms > 3000) { // stop after 3 seconds
    desired_velocity_one_rad_s = 0;
    desired_velocity_one_rad_s = 0;
    if (!finished) {
      finished = true;
      Serial.println("Finished");
    }
    analogWrite(M1PWR, 0);
    analogWrite(M2PWR, 0);
  }*/

  while (micros() < last_time_us + desired_time_us); // sample rate things
  last_time_us = micros(); // set comparison variables
  last_pos_rad[0] = actual_pos_rad[0];
  last_pos_rad[1] = actual_pos_rad[1];
}
