/*
* Name: Sam Fuller
* Assignment: 2A - motor driving with feedback loop
* Description: Sets the motors desired velocities and prints them out to the serial monitor
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

const unsigned long desired_time_us = 10000; // desired sample time in us (10 ms)
unsigned long last_time_us, start_time_us, last_read_time_motor_one, last_read_time_motor_two; // timer stuff
long motor_one_counts_p, motor_one_pos_counts, motor_two_counts_p, motor_two_pos_counts; // motor encoder counts
float current_time_ms = 0; // more timer stuff
float last_pos_one_rad, last_pos_two_rad, motor_one_pos_rad, motor_one_velocity_rad_s, motor_two_pos_rad, motor_two_velocity_rad_s; // motor positions and velocities
float voltage_one = 0; // motor voltage
float voltage_two = 0;
float error_one = 0; // differential between speed SP and sensor
float error_two = 0;
float battery_voltage = 7.8; // motor power supply
float Kp = 2; // system gain
float desired_velocity_one_rad_s = 0; // velocity setpoint
float desired_velocity_two_rad_s = 0;
float actual_velocity_one_rad_s = 0; // velocity sensor
float actual_velocity_two_rad_s = 0;
unsigned int pwm_one = 0; // PWM to write to motors
unsigned int pwm_two = 0;
bool finished = false; // for printing finished out to MatLab

// encoder ISR from assignment 1
void motor_one_encoder_isr() {
  if (micros() - last_read_time_motor_one > 100) {
    if (digitalRead(M1A) == digitalRead(M1B)) {
      motor_one_counts_p -= 2;
    } else {
      motor_one_counts_p += 2;
    }
    last_read_time_motor_one = micros();
  }
}

void motor_two_encoder_isr() {
  if (micros() - last_read_time_motor_two > 100) {
    if (digitalRead(M2A) == digitalRead(M2B)) {
      motor_two_counts_p -= 2;
    } else {
      motor_two_counts_p += 2;
    }
    last_read_time_motor_two = micros();
  }
}

// encoder adjuster from assignment 1
long encoder(int motor) {
  if (motor == 1) {
    if (digitalRead(M1A) != digitalRead(M1B)) return (motor_one_counts_p + 1);
    else return motor_one_counts_p;
  } else {
    if (digitalRead(M2A) != digitalRead(M2B)) return (motor_two_counts_p + 1);
    else return motor_two_counts_p;
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
  motor_one_pos_counts = encoder(1); // get motor counts
  motor_two_pos_counts = encoder(2);

  motor_one_pos_rad = 2 * PI * (float)motor_one_pos_counts / 3200; // amount traveled in radians
  motor_two_pos_rad = 2 * PI * (float)motor_two_pos_counts / 3200;

  motor_one_velocity_rad_s = (motor_one_pos_rad - last_pos_one_rad) / ((float)desired_time_us / 1000000); // angular velocity in rad/s
  motor_two_velocity_rad_s = (motor_two_pos_rad - last_pos_two_rad) / ((float)desired_time_us / 1000000);

  actual_velocity_one_rad_s = motor_one_velocity_rad_s; // set actual velocity for error calcs
  actual_velocity_two_rad_s = motor_two_velocity_rad_s;

  current_time_ms = (float)(last_time_us - start_time_us) / 1000; // convert time to ms

  if (!finished) {
    // print out status variables
    Serial.print(current_time_ms);
    Serial.print("\t");
    Serial.print(voltage_one);
    Serial.print("\t");
    Serial.print(voltage_two);
    Serial.print("\t");
    Serial.print(motor_one_velocity_rad_s);
    Serial.print("\t");
    Serial.print(motor_two_velocity_rad_s);
    Serial.print("\t");
    Serial.println(error_one);
  }

  if (current_time_ms >= 1000 && current_time_ms <= 3000) {
    desired_velocity_one_rad_s = 3.14; // 1-3 seconds set velocity to 1
    desired_velocity_two_rad_s = 3.14;
  }

  // motor controllers
  error_one = desired_velocity_one_rad_s - actual_velocity_one_rad_s; // motor self correction
  error_two = desired_velocity_two_rad_s - actual_velocity_two_rad_s;

  voltage_one = Kp * error_one; // scale error by gain to get proper voltage
  voltage_two = Kp * error_two;

  // set motor directions
  if (voltage_one > 0) {
    digitalWrite(M1DIR, HIGH);
  } else {
    digitalWrite(M1DIR, LOW);
  }
  if (voltage_two > 0) {
    digitalWrite(M2DIR, HIGH);
  } else {
    digitalWrite(M2DIR, LOW);
  }

  pwm_one = 255 * abs(voltage_one) / battery_voltage; // set motor PWMs
  pwm_two = 255 * abs(voltage_two) / battery_voltage;

  analogWrite(M1PWR, min(pwm_one, 255)); // write PWM to motor
  analogWrite(M2PWR, min(pwm_two, 255));

  if (current_time_ms > 3000) { // stop after 3 seconds
    desired_velocity_one_rad_s = 0;
    desired_velocity_one_rad_s = 0;
    if (!finished) {
      finished = true;
      Serial.println("Finished");
    }
    analogWrite(M1PWR, 0);
    analogWrite(M2PWR, 0);
  }

  while (micros() < last_time_us + desired_time_us); // sample rate things
  last_time_us = micros(); // set comparison variables
  last_pos_one_rad = motor_one_pos_rad;
  last_pos_two_rad = motor_two_pos_rad;
}
