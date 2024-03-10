#define M1A 2 // motor 1 encoder
#define M2A 3 // motor 2 encoder
#define ENABLE_PIN 4 // motor driver enable
#define M1B 5 // motor 1 encoder
#define M2B 6 // motor 2 encoder
#define M1DIR 7 // motor 1 direction input (voltage sign)
#define M2DIR 8 // motor 2 direction input (voltage sign)
#define M1PWR 9 // motor 1 speed input (voltage)
#define M2PWR 10 // motor 2 speed input (voltage)
#define BATTERY_VOLTAGE 7.8
#define RADIUS 0.237861
#define EULER 2.718281828459045
#define DISTANCE 1
#define MAX_VELOCITY_RAD_S 2*PI

const unsigned long desired_time_us = 10000; // desired sample time in us (10 ms)
const float Kp = 1.5; // velocity gain
float Kp_pos[2] = {1.5, 1.5}; // P controller gain, unique for each motor
const float Ki_pos[2] = {0.17, 0.15}; // I controller gain
unsigned long last_time_us, start_time_us; // timer stuff
float current_time_ms = 0; // more timer stuff
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
float actual_pos_velocity_rad_s[2] = {0, 0}; // current positional velocity in rad/s
float actual_rot_velocity_rad_s[2] = {0, 0}; // current rotational velocity in rad/s
unsigned int pwm[2] = {0, 0}; // PWM applied to motors
// turning and driving
float desired_distance_feet = 0;
float desired_distance_rad = 0;
float desired_angle_robot_deg = 0;
float desired_angle_wheels_deg = 0;
float desired_angle_rad = 0;
float forward_distance_rad = 0; // rho
float forward_velocity_rad_s = 0; // rho dot
float rotational_position_rad = 0; // phi
float rotational_velocity_rad_s = 0; // phi dot
float voltage_sum = 0; // speed control
float voltage_delta = 0; // rotational control
float desired_angle[2] = {0, 0};
float actual_angle[2] = {0, 0};
float desired_velocity[2] = {0, 0};
float actual_velocity[2] = {0, 0};
float actual_distance_traveled_rad[2] = {0, 0};
float final_angle[2] = {0, 0};
float amount_over_ninety = 1;
float turning_constant = 2.05;
bool turning = false;

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
      motor_counts_p[1] += 2;
    } else {
      motor_counts_p[1] -= 2;
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
  Serial.println("Ready!");
}

void loop() {
  desired_distance_feet = 0; // positive is forward.
  desired_angle_robot_deg = 90; // positive is left, negative is right.

  if (desired_angle_robot_deg != 0) turning = true;
  if (current_time_ms > 5000) turning = false;
  
  desired_distance_rad = (desired_distance_feet / RADIUS);
  
  desired_pos_rad[0] = desired_distance_rad;
  desired_pos_rad[1] = desired_distance_rad;
  
  motor_counts_pos[0] = encoder(1); // get motor counts
  motor_counts_pos[1] = encoder(2);
  
  // rotational position - theta
  actual_pos_rad[0] = 2 * PI * (float)motor_counts_pos[0] / 3200; // amount traveled in radians
  actual_pos_rad[1] = 2 * PI * (float)motor_counts_pos[1] / 3200;

  // rotational velocity in rad/s - theta dot
  actual_rot_velocity_rad_s[0] = (actual_pos_rad[0] - last_pos_rad[0]) / ((float)desired_time_us / 1000000);
  actual_rot_velocity_rad_s[1] = (actual_pos_rad[1] - last_pos_rad[1]) / ((float)desired_time_us / 1000000);

  // this says if we are going forwards or backwards
  // instant forward velocity - rho dot - positive is forward, negative is backwards
  forward_velocity_rad_s = RADIUS * (actual_rot_velocity_rad_s[0] + actual_rot_velocity_rad_s[1]) / 2;
  // this says if we are turning left or right
  // instant rotational velocity - phi dot - negative is turning left, positive is turning right
  rotational_velocity_rad_s = RADIUS * (actual_rot_velocity_rad_s[0] - actual_rot_velocity_rad_s[1]) / DISTANCE;

  current_time_ms = (float)(last_time_us - start_time_us) / 1000; // convert time to ms

  turning_constant = 5.9491 * pow(desired_angle_robot_deg, -0.199852);

  desired_angle_wheels_deg = turning_constant * desired_angle_robot_deg;
  desired_angle_rad = desired_angle_wheels_deg * PI / 180;

  actual_angle[0] = actual_pos_rad[0];
  actual_angle[1] = actual_pos_rad[1];

  // account for any turning that happened before we start traveling
  actual_distance_traveled_rad[0] = actual_pos_rad[0] - final_angle[0];
  actual_distance_traveled_rad[1] = actual_pos_rad[1] - final_angle[1];

  if (desired_angle_wheels_deg > 0) { // turning right
    desired_angle[0] = -1 * desired_angle_rad;
    actual_angle[0] = actual_pos_rad[0];
    desired_angle[1] = desired_angle_rad;
    actual_angle[1] = actual_pos_rad[1];
  } else { // turning left
    desired_angle[0] = desired_angle_rad;
    actual_angle[0] = actual_pos_rad[0];
    desired_angle[1] = -1 * desired_angle_rad; 
    actual_angle[1] = actual_pos_rad[1];
  }

  //average_turn_error = (abs(desired_angle[0] - actual_angle[0]) + abs(desired_angle[1] - actual_angle[1]))/2;
  // turning code - get within 0.5 degrees to stop turning
//  if (average_turn_error > 0.00872665 && desired_angle_robot_deg != 0) {
  if (turning) {
    // turning feedback loop - P controller
    for (int i = 0; i < 2; i++) {
      error[i] = desired_angle[i] - actual_angle[i];
      voltage[i] = error[i] * 1.5;
      pwm[i] = 255 * abs(voltage[i]) / BATTERY_VOLTAGE;
    }
    if (voltage[0] > 0) {
      digitalWrite(M1DIR, HIGH);
    } else {
      digitalWrite(M1DIR, LOW);
    }
    if (voltage[1] > 0) {
      digitalWrite(M2DIR, LOW);
    } else {
      digitalWrite(M2DIR, HIGH);
    }
  
    analogWrite(M1PWR, min(pwm[0], 128));
    analogWrite(M2PWR, min(pwm[1], 128 * 0.95)); // adjust faster motor down by 8%

    final_angle[0] = actual_angle[0];
    final_angle[1] = actual_angle[1];
  
//    Serial.print(voltage[0]);
//    Serial.print("\t");
//    Serial.print(voltage[1]);
//    Serial.print("\t");
//    Serial.print(actual_rot_velocity_rad_s[0]);
//    Serial.print("\t");
//    Serial.print(actual_rot_velocity_rad_s[1]);
//    Serial.print("\t");
//    Serial.print(desired_angle[0]);
//    Serial.print("\t");
//    Serial.print(actual_angle[0]);
//    Serial.print("\t");
//    Serial.print(desired_angle[1]);
//    Serial.print("\t");
//    Serial.print(actual_angle[1]);
//    Serial.print("\t");
//    Serial.print(pwm[0]);
//    Serial.print("\t");
//    Serial.println(pwm[1]);
  } else { // driving code
    // motor controllers
    for (int i = 0; i < 2; i++) {
      pos_error[i] = desired_pos_rad[i] - actual_distance_traveled_rad[i]; // position feedback loop, calculating error
      integral_error[i] = integral_error[i] + pos_error[i] * ((float)desired_time_us / 1000000); // calculating integral error
      // getting desired velocity using PI controller
      desired_velocity_rad_s[i] = Kp_pos[i] * pos_error[i] + Ki_pos[i] * integral_error[i];
      error[i] = desired_velocity_rad_s[i] - actual_rot_velocity_rad_s[i]; // getting error for voltage
      voltage[i] = Kp * error[i]; // getting voltage using gain and error
      pwm[i] = 255 * abs(voltage[i]) / BATTERY_VOLTAGE; // set motor PWM
    }
  
    // set motor directions depending on motor voltage sign
    if (voltage[0] > 0) {
      digitalWrite(M1DIR, HIGH);
    } else {
      digitalWrite(M1DIR, LOW);
    }
    if (voltage[1] > 0) {
      digitalWrite(M2DIR, LOW);
    } else {
      digitalWrite(M2DIR, HIGH);
    }
  
    analogWrite(M1PWR, min(pwm[0], 128)); // write PWM to both motors
    analogWrite(M2PWR, min(pwm[1], 128 * 0.915)); // adjust faster motor down by 12%

//    Serial.print(actual_distance_traveled_rad[0]);
//    Serial.print("\t");
//    Serial.print(actual_distance_traveled_rad[1]);
//    Serial.print("\t");
//    Serial.print(min(pwm[0], 128));
//    Serial.print("\t");
//    Serial.println(min(pwm[1], 128 * 0.915));
  }

  while (micros() < last_time_us + desired_time_us); // sample rate things
  last_time_us = micros(); // set comparison variables
  // setting previous variables for calculating delta in feedback loop
  last_pos_rad[0] = actual_pos_rad[0];
  last_pos_rad[1] = actual_pos_rad[1];
}
