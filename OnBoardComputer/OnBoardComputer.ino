#include "Rakieta.h"


Rakieta rakieta;

void setup()
{
  debugInit(115200);
  delay(50);
  debugln(F("Hello. Rocket setup"));
  
  // I2C for GPS
  Wire.begin();
  
  rakieta.init();
}

void loop()
{
  rakieta.loop();
  delay(2);  // for lower power consumption
}