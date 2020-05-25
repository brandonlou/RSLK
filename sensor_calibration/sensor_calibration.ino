#include <ECE3.h>

const uint8_t  NUM_IRS = 8;
const uint16_t BAUD_RATE = 9600;

uint16_t IR_Values[NUM_IRS];
uint16_t maxValues[NUM_IRS] = {0, 0, 0, 0, 0, 0, 0, 0};
uint16_t minValues[NUM_IRS] = {3000, 3000, 3000, 3000, 3000, 3000, 3000, 3000};
size_t i;


void setup() {
  
  ECE3_Init();
  Serial.begin(BAUD_RATE);
  delay(2000);
  
}


void loop() {

  ECE3_read_IR(IR_Values);

  for(i = 0; i < NUM_IRS; i++) {

    // Only save the minimum and maximum values of each sensor.
    maxValues[i] = max(maxValues[i], IR_Values[i]);
    minValues[i] = min(minValues[i], IR_Values[i]);

    // View the Serial Monitor and move the RSLK around your track, making sure each sensor gets a reading of the most white
    // and most black sections. Then stop autoscroll and save your values to be used for calibration.
    Serial.print(i);
    Serial.print(" | Min: ");
    Serial.print(minValues[i]);
    Serial.print(" | Max: ");
    Serial.print(maxValues[i]);
    Serial.println();
      
  }
    
  delay(50);
    
}
