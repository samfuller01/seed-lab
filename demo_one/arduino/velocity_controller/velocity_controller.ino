#define M1A 3 // motor 1 encoder
#define M2A 2 // motor 2 encoder
#define ENABLE_PIN 4 // motor driver enable
#define M1B 6 // motor 1 encoder
#define M2B 5 // motor 2 encoder
#define M1DIR 8 // motor 1 direction input (voltage sign)
#define M2DIR 7 // motor 2 direction input (voltage sign)
#define M1PWR 10 // motor 1 speed input (voltage)
#define M2PWR 9 // motor 2 speed input (voltage)
#define RADIUS 0.237861
#define MAX_VELOCITY_RAD_S 10

const unsigned long desired_time_us = 10000; // desired sample time in us (10 ms)
const float battery_voltage = 7.8; // motor power supply
const float Kp = 2; // velocity gain
float Kp_pos[2] = {3.5, 5}; // P controller gain, unique for each motor
const float Ki_pos = 0.7; // I controller gain
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
float actual_velocity_rad_s[2] = {0, 0}; // current velocity in rad/s
float desired_angle_deg = 0;
float desired_distance_feet = 0;
float desired_distance_radians = 0;
unsigned int pwm[2] = {0, 0}; // PWM applied to motors

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
  // 1 rotation of wheel is 3200 counts or 2 pi radians 0.2378 feet is radius
  // distance in feet divided by radius in feet gives us distance in radians
  desired_distance_feet = 5;
  desired_distance_radians = -1 * desired_distance_feet / RADIUS; // positive is backwards...
  desired_pos_rad[0] = desired_distance_radians;
  desired_pos_rad[1] = desired_distance_radians;
  
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
    error[i] = min(desired_velocity_rad_s[i] - actual_velocity_rad_s[i], MAX_VELOCITY_RAD_S); // getting error for voltage
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

  Serial.print(current_time_ms);
  Serial.print("\t");
  Serial.print(desired_pos_rad[0]);
  Serial.print("\t");
  Serial.print(desired_pos_rad[1]);
  Serial.print("\t");
  Serial.print(actual_velocity_rad_s[0]);
  Serial.print("\t");
  Serial.print(actual_velocity_rad_s[1]);
  Serial.print("\t");
  Serial.print(voltage[0]);
  Serial.print("\t");
  Serial.print(voltage[1]);
  Serial.print("\t");
  Serial.print(actual_pos_rad[0]);
  Serial.print("\t");
  Serial.print(actual_pos_rad[1]);
  Serial.print("\t");
  Serial.print(motor_counts_pos[0]);
  Serial.print("\t");
  Serial.println(motor_counts_pos[1]);

  analogWrite(M1PWR, min(pwm[0], 255)); // write PWM to both motors
  analogWrite(M2PWR, min(pwm[1], 255));

  while (micros() < last_time_us + desired_time_us); // sample rate things
  last_time_us = micros(); // set comparison variables
  // setting previous variables for calculating delta in feedback loop
  last_pos_rad[0] = actual_pos_rad[0];
  last_pos_rad[1] = actual_pos_rad[1];
}
