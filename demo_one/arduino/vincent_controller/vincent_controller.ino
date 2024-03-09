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
#define BATTERY_VOLTAGE 7.8
#define RADIUS 0.237861
#define WHEEL_DISTANCE 1
#define KP 1.9958
#define KP_RHO 4.8206
#define KI_RHO 2.0106
#define KP_PHI 4.8206
#define KI_PHI 2.0106

// theta - rotational position
// theta dot - rotational velocity
// rho - distance traveled
// rho dot - forward velocity

unsigned long last_time_us, start_time_us;
float current_time_ms = 0;
float desired_distance_feet = 0;
float desired_distance_rad = 0;
float desired_angle_deg = 0;
float desired_angle_rad = 0;
float rho_desired, rho_actual, rho_dot_desired, rho_dot_actual, phi_desired, phi_actual, phi_dot_desired, phi_dot_actual;
float V_bar, V_delta;
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
}

void loop() {
  desired_distance_feet = 4;
  desired_angle_deg = 90;

  phi_desired = desired_angle_deg * PI / 180;
  rho_desired = desired_distance_feet / RADIUS;

  motor_counts[0] = encoder(1);
  motor_counts[1] = encoder(2);

  theta[0] = 2 * PI * (float)motor_counts[0] / 3200;
  theta[1] = 2 * PI * (float)motor_counts[1] / 3200;

  theta_dot[0] = (theta[0] - last_theta[0]) / ((float)DESIRED_TIME_US / 1000000);
  theta_dot[1] = (theta[1] - last_theta[1]) / ((float)DESIRED_TIME_US / 1000000);

  rho_actual = RADIUS * (theta[0] + theta[1]) / 2;
  phi_actual = RADIUS * (theta[0] - theta[1]) / WHEEL_DISTANCE;

  rho_dot_actual = RADIUS * (theta_dot[0] + theta_dot[1]) / 2;
  phi_dot_actual = RADIUS * (theta_dot[0] - theta_dot[1]) / WHEEL_DISTANCE;

  // angle position & velocity controller - PI -> P -> V_delta
  pos_error[0] = phi_desired - phi_actual;
  integral_error[0] = integral_error[0] + pos_error[0] * ((float)DESIRED_TIME_US / 1000000);
  phi_dot_desired = KP_PHI * pos_error[0] + KI_PHI * integral_error[0];
  error[0] = phi_dot_desired - phi_dot_actual;
  V_delta = error[0] * KP;

  // distance position & velocity controller - PI -> P -> V_bar
  pos_error[1] = rho_desired - rho_actual;
  integral_error[1] = integral_error[1] + pos_error[1] * ((float)DESIRED_TIME_US / 1000000);
  rho_dot_desired = KP_RHO * pos_error[1] + KI_RHO * integral_error[1];
  error[1] = rho_dot_desired - rho_dot_actual;
  V_bar = error[1] * KP;

  voltage[0] = (V_bar - V_delta) / 2;
  voltage[1] = (V_bar + V_delta) / 2;

  pwm[0] = 255 * abs(voltage[0]) / BATTERY_VOLTAGE;
  pwm[1] = 255 * abs(voltage[1]) / BATTERY_VOLTAGE;


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
  analogWrite(M2PWR, min(pmw[1], 128));

  while (micros() < last_time_us + DESIRED_TIME_US);
  last_time_us = micros();
  last_theta[0] = actual_pos_rad[0];
  last_theta[1] = actual_pos_rad[1];
}
