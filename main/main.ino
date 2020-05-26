#include <ECE3.h>
#include <PID_v1.h>

// Helper enum to remember which side of the track the RSLK is on.
enum side {LEFT, RIGHT};

// PIN constants.
const uint8_t LEFT_NSLP  = 31,
              LEFT_DIR   = 29,
              LEFT_PWM   = 40,
              RIGHT_NSLP = 11,
              RIGHT_DIR  = 30,
              RIGHT_PWM  = 39;

// PID constants.
const double kP = 0.0008,
             kI = 0.0,
             kD = 0.0005;

// Number of IR sensors on the RSLK.
const uint8_t NUM_IRS = 8;

// Offset values are used to set the calibrated white value of every sensor to 0.
const uint16_t IR_OFFSET[NUM_IRS] = {589, 528, 566, 529, 541, 551, 621, 597};

// Scaling values scale each sensor's raw input to a value between 0 and 1000.
const double IR_SCALE[NUM_IRS] = {0.523286, 0.507099, 0.517063, 0.523560, 0.539957, 0.513084, 0.532198, 0.525486};
// Give different weights depending on how far the sensor is from the center.
const short IR_WEIGHT[NUM_IRS] = {-8, -4, -2, -1, 1, 2, 4, 8};

// Analog base speed of the RSLK before PID corrections.
const uint8_t BASE_SPEED = 50;

// Below the deadband range, wheels will not spin due to static friction.
const uint8_t DEADBAND = 11;

// Stores the raw IR readings from each sensor. 0 --> 7; right --> left.
uint16_t IR_Values[NUM_IRS];

double summedInput;

double calibratedInput;

// Stores the computed input value after calibration and fusion.
double fusedInput;

// Set by PIDController. If negative, RSLK is to the left of the track. If positive, to the right.
double output;

// Setpoint of our PID controller.
double targetValue = 0;

double leftSpeed, rightSpeed;

PID PIDController(&fusedInput, &output, &targetValue, kP, kI, kD, DIRECT);

// Stores which side of the track the RSLK is on.
enum side currentSide;

size_t i;


void setup() {

  pinMode(LEFT_NSLP,  OUTPUT);
  pinMode(LEFT_DIR,   OUTPUT);
  pinMode(LEFT_PWM,   OUTPUT);
  pinMode(RIGHT_NSLP, OUTPUT);
  pinMode(RIGHT_DIR,  OUTPUT);
  pinMode(RIGHT_PWM,  OUTPUT);

  digitalWrite(LEFT_NSLP,  HIGH);
  digitalWrite(LEFT_DIR,   LOW); // Set HIGH to reverse.
  digitalWrite(RIGHT_NSLP, HIGH);
  digitalWrite(RIGHT_DIR,  LOW); // Set HIGH to reverse.

  ECE3_Init();
  PIDController.SetMode(AUTOMATIC);
  PIDController.SetOutputLimits(-BASE_SPEED, BASE_SPEED);

//  Serial.begin(9600);

  delay(5000); // Wait 5 seconds before starting.

}


void loop() {


  ECE3_read_IR(IR_Values); // 0 (white) to 2500 (black)

  fusedInput = summedInput = 0;
  for(i = 0; i < NUM_IRS; i++) {
    
    // Calibrate each sensor to a value between [0, 1000].
    calibratedInput = min(max(IR_Values[i] - IR_OFFSET[i], 0) * IR_SCALE[i], 1000);

    // Multiply each calibrated input with its corresponding weight.
    fusedInput += calibratedInput * IR_WEIGHT[i];

    summedInput += calibratedInput;
    
  }

  // Keep track of which side of the track the car is on.
  currentSide = (fusedInput < 0) ? LEFT : RIGHT;

  if(summedInput >= 7000) {
    donut();
    
  } else {
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
  for(i = 0; i < 140000; i++) {
    analogWrite(LEFT_PWM, 40);
    analogWrite(RIGHT_PWM, 40);
  }

  analogWrite(LEFT_PWM, 0);
  analogWrite(RIGHT_PWM, 0);

  // Re-reverse right wheel.
  digitalWrite(RIGHT_DIR, LOW);
  
}
