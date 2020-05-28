#ifndef MAIN_H
#define MAIN_H

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
const double kP = 0.006,
             kI = 0.00000000001,
             kD = 0.00055;

// Number of IR sensors on the RSLK.
const uint8_t NUM_IRS = 8;

// Offset values are used to set the calibrated white value of every sensor to 0.
const uint16_t IR_OFFSET[NUM_IRS] = {394, 326, 371, 349, 361, 349, 417, 385};

// Scaling values scale each sensor's raw input to a value between 0 and 1000.
const double IR_SCALE[NUM_IRS] = {0.474834, 0.459982, 0.469704, 0.552181, 0.514139, 0.464900, 0.480077, 0.472813};

// Give different weights depending on how far the sensor is from the center.
const short IR_WEIGHT[NUM_IRS] = {-8, -4, -2, -1, 1, 2, 4, 8};

// Analog base speed of the RSLK before PID corrections.
const uint8_t BASE_SPEED = 40;

// Below the deadband range, wheels will not spin due to static friction.
const uint8_t DEADBAND = 11;

// Threshold of when a crossbar is detected based on summedInput.
const uint16_t BAR_THRESHOLD = 6000;

// Time after a crossbar is detected before another crossbar can be counted.
const uint16_t BAR_TIME = 750;

// Amount of time to spin for (in cycles; may not be accurate).
const uint32_t SPIN_TIME = 180000;

// Analog PWM speed of both left and right wheels to spin.
const uint8_t SPIN_SPEED = 40;

// Stores the raw IR readings from each sensor. 0 --> 7; right --> left.
uint16_t IR_Values[NUM_IRS];

// Stores calibrated input based on offset and scale to a range between 0 and 1000.
double calibratedInput;

// Stores the sum of all calibrated inputs (no fusion).
double summedInput;

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

// Stores the time of last crossbar detection.
unsigned long detectBarTime = 0;

uint8_t numBars = 0;

size_t i;

#endif
