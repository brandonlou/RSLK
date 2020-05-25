#include <ECE3.h>
#include <PID_v1.h>

const uint8_t LEFT_NSLP = 31,
              LEFT_DIR = 29,
              LEFT_PWM = 40,
              RIGHT_NSLP = 11,
              RIGHT_DIR = 30,
              RIGHT_PWM = 39;

const double kP = 0,
             kI = 0,
             kD = 0;

const uint8_t NUM_IRS = 8;
const int8_t IR_NORMALIZE[NUM_IRS] = {1, 1, 1, 1, 1, 1, 1, 1}; // Calibration.
const int8_t IR_WEIGHT[NUM_IRS] = {-8, -4, -2, -1, 1, 2, 4, 8};
double TARGET_VALUE = 100; // TODO
double baseSpeed = 100; // TODO

uint16_t IR_Values[NUM_IRS]; // right -> left, 0 -> 7
double fusedInput, output;
size_t i;
PID PIDController(&fusedInput, &output, &TARGET_VALUE, kP, kI, kD, DIRECT);

void setup() {

  pinMode(LEFT_NSLP,  OUTPUT);
  pinMode(LEFT_DIR,   OUTPUT);
  pinMode(LEFT_PWM,   OUTPUT);
  pinMode(RIGHT_NSLP, OUTPUT);
  pinMode(RIGHT_DIR,  OUTPUT);
  pinMode(RIGHT_PWM,  OUTPUT);

  digitalWrite(LEFT_NSLP,  HIGH);
  digitalWrite(LEFT_DIR,   HIGH);
  digitalWrite(RIGHT_NSLP, HIGH);
  digitalWrite(RIGHT_DIR,  HIGH);

  ECE3_Init();
  PIDController.SetMode(AUTOMATIC);

  delay(2000); // Wait 2 seconds before starting.

}

void loop() {

  ECE3_read_IR(IR_Values); // 0 (white) to 2500 (black)

  // TODO: Adjust IR_Values based on calibration.

  fusedInput = 0;
  for(i = 0; i < NUM_IRS; i++) {
    fusedInput += IR_WEIGHT[i] * IR_Values[i];
  }

  PIDController.Compute(); // Modifies output passed by reference.
  
  double leftSpeed = baseSpeed + output; // TODO
  double rightSpeed = baseSpeed - output;
  analogWrite(LEFT_PWM, leftSpeed);
  analogWrite(RIGHT_PWM, rightSpeed);
  
}
