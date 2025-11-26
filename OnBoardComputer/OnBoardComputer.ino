#include "Rakieta.h"

Rakieta rakieta;

void setup() {
  // put your setup code here, to run once:
  debugInit(9600);
  delay(50);
  debugln(F("Hello. Rocket setup"));
  
  // I2C for GPS and SPI
  Wire.begin();
  SPI.begin();
  
  rakieta.init();
}

void loop() {
  // put your main code here, to run repeatedly:
  rakieta.handleGps();
  rakieta.handleLsm6();
  rakieta.handleAdxl();
  rakieta.handleBmp();
  rakieta.handleMax();
  rakieta.updateStatus();
  rakieta.writeRocketData();  
  rakieta.sendMsg();
  rakieta.watchdog();
  delay(1000);
}
