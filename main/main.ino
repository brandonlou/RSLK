#include <ECE3.h>
#include <PID_v1.h>

enum side {LEFT, RIGHT};

const uint8_t LEFT_NSLP  = 31,
              LEFT_DIR   = 29,
              LEFT_PWM   = 40,
              RIGHT_NSLP = 11,
              RIGHT_DIR  = 30,
              RIGHT_PWM  = 39;

const double kP = 0.0008,
             kI = 0.0,
             kD = 0.0005;

const uint8_t NUM_IRS = 8;
const int IR_OFFSET[NUM_IRS] = {655, 574, 632, 585, 596, 619, 701, 687};
const double IR_SCALE[NUM_IRS] = {0.542005, 0.519211, 0.535332, 0.535619, 0.535332, 0.531632, 0.555864, 0.551572};
const int IR_WEIGHT[NUM_IRS] = {-8, -4, -2, -1, 1, 2, 4, 8};
const double BASE_SPEED = 50;

uint16_t IR_Values[NUM_IRS]; // 0 --> 7; rightmost --> leftmost sensor
double fusedInput,
       output; // If negative, car is to the left of the line. If positive, to the right.
double targetValue = 0;
double leftSpeed, rightSpeed;
size_t i;
PID PIDController(&fusedInput, &output, &targetValue, kP, kI, kD, DIRECT);
enum side currentSide;


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
  PIDController.SetOutputLimits(-50, 50);

//  Serial.begin(9600);
  delay(5000); // Wait 3 seconds before starting.

}


void loop() {

  ECE3_read_IR(IR_Values); // 0 (white) to 2500 (black)

  fusedInput = 0;
  for(i = 0; i < NUM_IRS; i++) {
    
    // Calibrate each sensor.
    double test = min(max(IR_Values[i] - IR_OFFSET[i], 0) * IR_SCALE[i], 1000);

    // Multiply each calibrated input with its corresponding weight.
    fusedInput += test * IR_WEIGHT[i];
    
  }

  // Keep track of which side of the track the car is on.
  currentSide = (fusedInput < 0) ? LEFT : RIGHT;

  // Modify output passed by referenced using kP, kI, kD, fusedInput, and TARGET_VALUE values.
  PIDController.Compute();

//  Serial.print(output);
//  Serial.println();
//  delay(100);

  leftSpeed  = BASE_SPEED + output;
  rightSpeed = BASE_SPEED - output;
  
  tunedAnalogWrite(LEFT_PWM,  leftSpeed);
  tunedAnalogWrite(RIGHT_PWM, rightSpeed);
  
}

// Takes in a value from 0 to 255
// And maps it to a value from DEADBAND to 255.
void tunedAnalogWrite(int PIN, int value) {

  value = map(value, 0, 255, 11, 255);
  analogWrite(PIN, value);
  
}
