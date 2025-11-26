#ifndef RAKIETA_H
#define RAKIETA_H

#include <stdlib.h>
#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <SoftwareSerial.h>  /// zamienić na HardwareSerial
// #include <LoRaWan_APP.h>  /// odkomentować jak przejdę na stm32
#include <TinyGPSPlus.h>
#include <Adafruit_LSM6DS.h>    // LSM6DS3 via SPI
#include <Adafruit_BMP3XX.h>    // BMP388 via SPI
#include <Adafruit_ADXL343.h>   // ADXL375 via SPI
#include <Adafruit_MAX31855.h>  // MAX31855 via SPI
#include <Adafruit_SPIFlash.h>


#include "config.h"
#include "BitStorage.h"


class Rakieta
{
  private:
    enum class State {
      debug,  // debug purpose only
      idle,  // waiting in the pad
      ready,  // waiting for start
      burn,  // engine run
      rising,  // engine cut-off
      apogee,  // waiting for the parashute
      falling,  // parashute open
      touchdown  // grounded
    } status;

    struct {
      struct {
        float lat = 0;
        float lng = 0;
        float altiM = 0;
        float altiF = 0;
        float speed = 0;
      } gps;
      struct {
        float ax = 0;
        float ay = 0;
        float az = 0;
        float gx = 0;
        float gy = 0;
        float gz = 0;
      } lsm;
      struct {
        float ax = 0;
        float ay = 0;
        float az = 0;
      } adxl;
      struct {
        float altitude = 0;
      } bmp;
      struct {
      } max;
    } offsets;

    struct {
      struct {
        float lat = 0;
        float lng = 0;
        float altiM = 0;
        float altiF = 0;
        float speed = 0;
        uint8_t h = 0;
        uint8_t m = 0;
        uint8_t s = 0;
        uint8_t centi = 0;
        float course = 0;
        uint8_t satNum = 0;
        uint8_t hdop = 0;
        float lastAltitude = 0;
        float lastVerticalSpeed = 0;
        float maxAltitude = 0;
        uint32_t lastTime = 0;
      } gps;
      struct {
        float ax = 0;
        float ay = 0;
        float az = 0;
        float gx = 0;
        float gy = 0;
        float gz = 0;
        float temp = 0;
        float lastTotalSpeed = 0;
        float lastTotalAccel = 0;
        float lastTotalRotation = 0;
        uint32_t lastTime = 0;
      } lsm;
      struct {
        float ax = 0;
        float ay = 0;
        float az = 0;
        float lastTotalSpeed = 0;
        float lastTotalAccel = 0;
        uint32_t lastTime = 0;
      } adxl;
      struct {
        float pressure = 0;
        float altitude = 0;
        float lastAltitude = 0;
        float lastVerticalSpeed = 0;
        float maxAltitude = 0;
        uint32_t lastTime = 0;
        float temp = 0;
      } bmp;
      struct {
        float temp = 0;
      } max;
    } data;

    uint16_t error = 0;
    uint32_t packet = 0;

    uint32_t lastWatchdogTime = 0;

    uint32_t startTime = 0;
    uint32_t afterburnStartTime = 0;
    uint32_t afterRisingStartTime = 0;
    uint32_t apogeeStartTime = 0;
    uint32_t touchdownStartTime = 0;

    BitStorage message;
    
    bool sdReady = false, flashReady = false;
    uint32_t fileNumber = 0;
    String currentFileName;
    File32 SDDataFile;
    File32 flashDataFile;

    SdFat sd;
    /// SPIClass flashSPI(HSPI);
    /// Adafruit_FlashTransport_SPI flashTransport(FLASH_CS, &flashSPI);
    Adafruit_FlashTransport_SPI flashTransport;
    Adafruit_SPIFlash flash;
    FatVolume fatfs;

    TinyGPSPlus gps;
    SoftwareSerial gpsSerial;
    Adafruit_LSM6DS lsm;
    Adafruit_BMP3XX bmp;
    Adafruit_ADXL343 adxl;
    Adafruit_MAX31855 max3;

    void prepareMsg();

    bool flashInit();
    bool SDInit();
    bool flashFindNextFileNumber();
    bool flashOpenNewFile();
    bool flashWriteData(const String& data);
    bool SDOpenNewFile();
    bool SDWriteData(const String& data);
    
    void parashuteOpen();
    void setupServo();

  public:
    Rakieta();

    void init();

    void radioConfig() {};
    void onTxDone(void) {};
    void onTxTimeout(void) {};
    void onRxDone(uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr) {};
    
    bool writeRocketData();
    
    void sendMsg();
    void watchdog();
    void setOffsets();
    void handleGps();
    void handleLsm6();
    void handleAdxl();
    void handleBmp();
    void handleMax();
    void updateStatus();
    void servoDeg(uint8_t);
};

#endif  // RAKIETA_H
