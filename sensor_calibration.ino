#include <ECE3.h>

const uint8_t  NUM_IRS = 8;
const uint16_t BAUD_RATE = 9600;

uint16_t IR_Values[NUM_IRS];
size_t i;

void setup() {

  ECE3_Init();
  Serial.begin(BAUD_RATE);
  delay(2000);
  
}

void loop() {

  ECE3_read_IR(IR_Values);

  for(i = 0; i < NUM_IRS; i++) {
    Serial.print(IR_Values[i]);
    Serial.print('\t');
  }
  
  Serial.println();
  delay(50);
    
}
