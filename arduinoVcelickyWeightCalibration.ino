#include "HX711.h"

//Arduino pins: 
//2 -> HX711 CLK
//3 -> DOUT
//5V -> VCC
//GND -> GND

#define DOUT  50
#define CLK  51

HX711 scale(DOUT, CLK);
float calibrationFactor = -21740;

void setup() {
  Serial.begin(115200);
  Serial.println("HX711 Calibration");
  Serial.println("Remove all weight from scale");
  Serial.println("Press a,s,d,f to increase calibration factor by 10,100,1000,10000 respectively");
  Serial.println("Press z,x,c,v to decrease calibration factor by 10,100,1000,10000 respectively");
  Serial.println("Press t to tare");
  
  scale.set_scale();
  scale.tare();  
}

void loop() {

  Serial.println("Place known weight on scale");
  scale.set_scale(calibrationFactor); 
  
  Serial.print("Reading: ");
  Serial.print(scale.get_units(), 3);
  Serial.print(" kg,");
  Serial.print(" calibration_factor: ");
  Serial.print(calibrationFactor);
  Serial.println();

  if(Serial.available()){
    
    char temp = Serial.read();
    
    if(temp == 'a')calibrationFactor += 10;
    else if(temp == 'z')calibrationFactor -= 10;
    else if(temp == 's')calibrationFactor += 100;  
    else if(temp == 'x')calibrationFactor -= 100;  
    else if(temp == 'd')calibrationFactor += 1000;  
    else if(temp == 'c')calibrationFactor -= 1000;
    else if(temp == 'f')calibrationFactor += 10000;  
    else if(temp == 'v')calibrationFactor -= 10000;  
    else if(temp == 't')scale.tare();  
  }
}

