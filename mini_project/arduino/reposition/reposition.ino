// Wheels turn according to quadrant position of marker on Raspberry Pi.
// Wheels resist changes to position.

// name motor pins
#define ENABLE_PIN 4
#define M1DIR 7 // motor 1 direction input (voltage sign)
#define M2DIR 8 // motor 2 direction input (voltage sign)
#define M1PWR 9 // motor 1 speed input (voltage)
#define M2PWR 10

// times in ms for given wheel to turn 180 degrees
float turnTime1;
float turnTime2;
// positions of wheels; both start with 0 upwards
int position1 = 0;
int position2 = 0;
// quadrant position of marker
int marker; // 1:NE, 2:NW, 3:SW, 4:SE

void setup() {
  // set a high baud rate to keep up with received data
  Serial.begin(115200);
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
} // end setup

void loop() {
 /*  
 *   Marker | Left Wheel | Right Wheel
 *     NE          0            0
 *     NW          0            1
 *     SW          1            1
 *     SE          1            0
 */
  if(marker == 1){ // if marker is in NE
    if(position1 != 0){ // if left wheel is not 0 
      analogWrite(9, 127); // power left motor
      delay(turnTime1); // wait to turn 180 degrees
    }
    if(position2 != 0){ // if right wheel is not 0
      analogWrite(10, 127);
      delay(turnTime2);
    }
  } // end if (marker 1)
  
  if(marker == 2){ // if marker is in NW
    if(position1 != 0){ // if left wheel is not 0 
      analogWrite(9, 127); // power left motor
      delay(turnTime1); // wait to turn 180 degrees
    }
    if(position2 != 1){ // if right wheel is not 1
      analogWrite(10, 127);
      delay(turnTime2);
    }
  } // end if (marker 2)
  
  if(marker == 3){ // if marker is in SW
    if(position1 != 1){ // if left wheel is not 1
      analogWrite(9, 127); // power left motor
      delay(turnTime1); // wait to turn 180 degrees
    }
    if(position2 != 1){ // if right wheel is not 1
      analogWrite(10, 127);
      delay(turnTime2);
    }
  } // end if (marker 3)
  
  if(marker == 4){ // if marker is in SE
    if(position1 != 0){ // if left wheel is not 1
      analogWrite(9, 127);
      delay(turnTime1);
    }
    if(position2 != 0){ // if right wheel is not 0
      analogWrite(10, 127);
      delay(turnTime2);
    }
  } // end if (marker 4
  
} // end loop
