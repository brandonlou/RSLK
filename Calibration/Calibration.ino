#include <ECE3.h>

// TODO: Test later.
//const uint8_t BUTTON1 = 73;
//const uint8_t BUTTON2 = 74;

const uint8_t  NUM_IRS = 8;
const uint16_t BAUD_RATE = 9600;
const uint16_t MAX_NORMALIZED_VALUE = 1000;
const uint8_t  PRINT_PRECISION = 6;

uint16_t IR_Values[NUM_IRS];
uint16_t maxValues[NUM_IRS] = {0, 0, 0, 0, 0, 0, 0, 0};
uint16_t minValues[NUM_IRS] = {2500, 2500, 2500, 2500, 2500, 2500, 2500, 2500};
double scaleFactor;
String offset; // For some reason, doubles cannot easily converted into strings so there's no string for all the scaleFactors.
size_t i;


void setup() {
  
  ECE3_Init();
  Serial.begin(BAUD_RATE);
  delay(2000);
  
}


void loop() {

  ECE3_read_IR(IR_Values);

  offset = "offset = {";
  Serial.print("scale = {");

  for(i = 0; i < NUM_IRS; i++) {

    // Save the minimum and maximum values of each sensor.
    maxValues[i] = max(maxValues[i], IR_Values[i]);
    minValues[i] = min(minValues[i], IR_Values[i]);
    scaleFactor = (double)MAX_NORMALIZED_VALUE / (maxValues[i] - minValues[i]);

    offset += String(minValues[i]);
    Serial.print(scaleFactor, PRINT_PRECISION);

    if(i < NUM_IRS - 1) {
      offset += ", ";
      Serial.print(", ");
    }
    
  }

  // View the Serial Monitor and move the RSLK around your track, making sure each sensor gets a reading of the most white
  // and most black regions. Then stop autoscroll and save the latest minimum and scaling values of each sensor for calibration.
  Serial.println("};");
  Serial.println(offset + "};");
    
  delay(50);
    
}
