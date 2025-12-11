#include "Rakieta.h"


Rakieta rakieta;

void setup()
{
  debugInit(9600);
  delay(50);
  debugln(F("Hello. Rocket setup"));
  
  // I2C for GPS and SPI
  Wire.begin();
  SPI.begin();
  
  rakieta.init();
}

void loop()
{
  rakieta.loop();
  delay(2);
}

