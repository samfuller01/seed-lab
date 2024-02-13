/* 'ReadFromArduino.mlx' reads three columns of data from Arduino's serial monitor,
 * then it saves the data to a 3-column matrix in 'stepData.mat' under variable
 * name 'data'.
 * This program prints time, the right wheel's angular velocity, and the left wheel's
 * angular velocity to columns 1, 2, and 3 respectively.
 * The values saved to 'stepData.mat' will be used by 'localization_animation.mlx' to
 * calculate position and orientation. Then the calculated values will be saved to a
 * new matrix which will be used to animate the ghost robot's movements. */

// name motor pins
#define ENABLE_PIN 4
#define M1DIR 7 // motor 1 direction input (voltage sign)
#define M2DIR 8 // motor 2 direction input (voltage sign)
#define M1PWR 9 // motor 1 speed input (voltage)
#define M2PWR 10
// name encoder pins
#define M1A 2 // motor 1 encoder channel A
#define M2A 3 // motor 2 encoder channel A
#define M1B 5 // motor 1 encoder channel B
#define M2B 6

// variables for sampling times; used to debounce encoder routine
unsigned long desired_time_us = 100000; // desired sample time in us (100 ms)
unsigned long last_time_us;
unsigned long start_time_us;
unsigned long lastReadTime1, lastReadTime2;
float current_time = 0.0; // must be declared float for velocity conversion

// variables for encoder counts, pi for radian conversion
long motor_one_counts, motor_two_counts;
long pos1_counts, pos2_counts;
const float pi = 3.1415;

// wheel rotation of motors in radians
float motor1_rad;
float motor2_rad;
// angular velocities of wheels
float angVelocity1;
float angVelocity2;
// reference for radial positions of wheels
float last_pos1 = 0;
float last_pos2 = 0;

// ISR to read motor 1's encoder counts
void motor_one_encoder_isr() {
  // if more than 0.1 ms has passed since last execution
  if (micros() - lastReadTime1 > 100) {
    if (digitalRead(M1A) == digitalRead(M1B)) {
      motor_one_counts -= 2;
    } else {
      motor_one_counts += 2;
    }
    lastReadTime1 = micros(); // update time of last execution
  }
}

void motor_two_encoder_isr() {
  if (micros() - lastReadTime2 > 100) {
    if (digitalRead(M2A) == digitalRead(M2B)) {
      motor_two_counts -= 2;
    } else {
      motor_two_counts += 2;
    }
    lastReadTime2 = micros();
  }
}

// REMEMBER: THE WHEELS ARE SWITCHED, encoder(1) RETURNS ENCODER 2 COUNTS
// function to return encoder counts from motor 1 or 2; actual counting is done by 'motor_NUM_encoder_isr'
int encoder(int motor) {
  // if we want counts from motor 1's encoder
  if (motor == 1) {
    // correct encoder counts based on states of motors' encoder channels
    if (digitalRead(M1A) != digitalRead(M1B)) return (motor_one_counts + 1);
    else return motor_one_counts;
  } else { // else we want motor 2's encoder counts
    if (digitalRead(M2A) != digitalRead(M2B)) return (motor_two_counts + 1);
    else return motor_two_counts;
  }
}


void setup() {
  // set a high baud rate to keep up with received data
  Serial.begin(115200);

  // record start time at setup as reference for 'loop()' time
  last_time_us = micros();
  start_time_us = last_time_us;

  // pins 9, 10 send PWM signals to motors
  pinMode(M1PWR, OUTPUT);
  pinMode(M2PWR, OUTPUT);
  // pins 7, 8 change sign of voltage using H-bridge
  pinMode(M1DIR, OUTPUT);
  pinMode(M2DIR, OUTPUT);
  // pin 4 disables both motors' channels when LOW
  pinMode(ENABLE_PIN, OUTPUT);
  // start with motors enabled
  digitalWrite(ENABLE_PIN, HIGH);

  // setup encoder count ISRs for each motor
  attachInterrupt(digitalPinToInterrupt(M1A), motor_one_encoder_isr, CHANGE);
  attachInterrupt(digitalPinToInterrupt(M2A), motor_two_encoder_isr, CHANGE);

  // flag to start matlab recording
  Serial.println("Ready!");
} // end setup

void loop() {
  // set voltage sign (direction)
  digitalWrite(M1DIR, HIGH);
  digitalWrite(M2DIR, HIGH);
  
  // update last time
  last_time_us = micros();
  // start recording time in 'loop()'
  current_time = (float)(last_time_us - start_time_us) / 1000;

  // set up encoder counts and radial positions
  pos1_counts = encoder(1); // return wheel 2 counts
  motor1_rad = 2 * pi * (float)pos1_counts / 3200; // convert counts to radial position
  pos2_counts = encoder(2); // return wheel 1 counts
  motor2_rad = 2 * pi * (float)pos2_counts / 3200;
  last_pos1 = motor1_rad; // record radial position of motor 1's wheel
  last_pos2 = motor2_rad; // record radial position of motor 2's wheel

  // for 3 seconds
  while (current_time < 20000){
    // update current time
    current_time = (float)(last_time_us - start_time_us) / 1000;

    // record encoder 1 counts
    pos1_counts = encoder(2);
    // convert counts to radial position
    motor1_rad = 2 * pi * (float)pos1_counts / 3200;
    // calculate angular velocity of wheel 1
    angVelocity1 = (motor1_rad - last_pos1) / ((float)desired_time_us/1000000);

    // record encoder 2 counts
    pos2_counts = encoder(1);
    // convert counts to radial position
    motor2_rad = 2 * pi * (float)pos2_counts / 3200;
    // calculate angular velocity of wheel 2
    angVelocity2 = (motor2_rad - last_pos2) / ((float)desired_time_us/1000000);

    // print data for 'ReadFromArduino.mlx'
    Serial.print(current_time); // time in column 1
    Serial.print("\t");
    Serial.print(-angVelocity1); // right wheel angular velocity in column 2
    Serial.print("\t");
    Serial.print(angVelocity2); // left wheel angular velocity in column 3
    Serial.println("");

    // update last position of wheel 1
    last_pos1 = motor1_rad;
    // update last position of wheel 2
    last_pos2 = motor2_rad;

    // wait to satisfy samping rate
    while(micros() < last_time_us + desired_time_us);
    // update last time
    last_time_us = micros();
  } // end while

  // after 3 seconds have passed
  while (current_time >= 20000){
    // update current time
    current_time = (float)(last_time_us - start_time_us) / 1000;

    // if 3 seconds have passed
    if (current_time > 20000 && current_time <= 20150) { // range of time for reliability
      // flag to end matlab recording
      Serial.println("Finished");
    } // end if

    // wait to satisfy sampling rate
    while(micros() < last_time_us + desired_time_us);
    // update last time
    last_time_us = micros();
  } // end while

} // end loop
