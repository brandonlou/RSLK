#include <ECE3.h>
#include <PID_v1.h>
#include "/Users/brandon/Documents/Energia/RSLK/Main/Main.h"


void setup() {

  pinMode(LEFT_NSLP,  OUTPUT);
  pinMode(LEFT_DIR,   OUTPUT);
  pinMode(LEFT_PWM,   OUTPUT);
  pinMode(RIGHT_NSLP, OUTPUT);
  pinMode(RIGHT_DIR,  OUTPUT);
  pinMode(RIGHT_PWM,  OUTPUT);
  pinMode(YELLOW_LED, OUTPUT);

  digitalWrite(LEFT_NSLP,  HIGH);
  digitalWrite(LEFT_DIR,   LOW); // Forwards direction.
  digitalWrite(RIGHT_NSLP, HIGH);
  digitalWrite(RIGHT_DIR,  LOW); // Forwards direction.

  ECE3_Init();
  PIDController.SetMode(AUTOMATIC);
  PIDController.SetOutputLimits(-BASE_SPEED, BASE_SPEED);

//  Serial.begin(9600);

  delay(5000); // Wait 5 seconds before starting.

}


void loop() {

  ECE3_read_IR(IR_Values); // 0 (white) to 2500 (black)

  fusedInput = summedInput = 0;
  for(i = 0; i < NUM_IRS; ++i) {
    
    // Calibrate each sensor to a value between [0, 1000].
    calibratedInput = min(max(IR_Values[i] - IR_OFFSET[i], 0) * IR_SCALE[i], 1000);

    // Multiply each calibrated input with its corresponding weight.
    fusedInput += calibratedInput * IR_WEIGHT[i];

    // Add all sensor inputs to identify crossbars.
    summedInput += calibratedInput;
    
  }

  // On top of a crossbar.
  if(summedInput >= BAR_THRESHOLD) {
    digitalWrite(YELLOW_LED, HIGH);
    
    // Do not count another crossbar if BAR_TIME has not been elapsed.
    if(millis() - detectBarTime >= BAR_TIME) {
      numBars++;
      detectBarTime = millis();
    }

    // Do a 180 depending on the track and number of crossbars seen.
    if((currentMode == STRAIGHT && numBars == STRAIGHT_BARS) || (currentMode == RIBBON && numBars == RIBBON_BARS)) {
      donut();
    }

    // If on ribbon track and back at start/finish line, stop moving.
    if(currentMode == RIBBON && numBars >= RIBBON_BARS * 2) {
      while(true) {
        analogWrite(LEFT_PWM,  0);
        analogWrite(RIGHT_PWM, 0);
      }
    }

  // Not under a crossbar.
  } else {
    digitalWrite(YELLOW_LED, LOW);
    goStraight();
  }
  
}


void goStraight() {
  
  // Comput output using kP, kI, kD, fusedInput, and targetValue.
  PIDController.Compute();

  // Modify base speed by the PID correction.
  leftSpeed  = BASE_SPEED + output;
  rightSpeed = BASE_SPEED - output;

  // Account for deadband.
  leftSpeed  = map(leftSpeed,  0, 255, DEADBAND, 255);
  rightSpeed = map(rightSpeed, 0, 255, DEADBAND, 255);

  // Move the motors.
  analogWrite(LEFT_PWM,  leftSpeed);
  analogWrite(RIGHT_PWM, rightSpeed);
  
}


void donut() {

  // Reverse right wheel.
  digitalWrite(RIGHT_DIR, HIGH);

  // Spin!!! (ideally, would use encoders)
  for(i = 0; i < SPIN_TIME; ++i) {
    analogWrite(LEFT_PWM,  SPIN_SPEED);
    analogWrite(RIGHT_PWM, SPIN_SPEED);
  }

  // Re-reverse right wheel.
  digitalWrite(RIGHT_DIR, LOW);
    
}
