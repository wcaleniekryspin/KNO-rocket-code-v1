#ifndef RAKIETA_H
#define RAKIETA_H

#include <stdlib.h>
#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <HardwareSerial.h>
#include <RadioLib.h>           // SX1262
#include <TinyGPSPlus.h>        // MAX M10S
#include <Adafruit_LSM6DS.h>    // LSM6DS3 via SPI
#include <Adafruit_BMP3XX.h>    // BMP388 via SPI
#include <Adafruit_ADXL343.h>   // ADXL375 via SPI
#include <Adafruit_MAX31855.h>  // MAX31855 via SPI
#include <Adafruit_SPIFlash.h>  // W25Q128 via SPI
#include <Servo.h>

#include "config.h"
#include "BitStorage.h"

// It must be here for the RadioLib module to function properly
inline volatile bool operationDone = false;
inline void setOperationFlag(void) { operationDone = true; }


class Rakieta
{
  private:
    enum class State {
      debug,     // debug purpose only
      idle,      // waiting in the pad
      ready,     // waiting for start
      burn,      // engine run
      rising,    // engine cut-off
      apogee,    // waiting for the parashute
      falling,   // parashute open
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
      struct { /// raczej do usunięcia bo tu nie ma offsetu
      } max;
    } offsets;

    struct {
      struct {
        float lat = 0;
        float lng = 0;
        float altiM = 0;
        float altiF = 0;
        uint8_t h = 0;
        uint8_t m = 0;
        uint8_t s = 0;
        uint8_t centi = 0;
        float speed = 0;
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
        float temp = 0;
        float pressure = 0;
        float altitude = 0;
        float lastAltitude = 0;
        float lastVerticalSpeed = 0;
        float maxAltitude = 0;
        uint32_t lastTime = 0;
      } bmp;
      struct {
        float temp = 0;
      } max;
      struct {
        float voltage = 0;
      } battery;
    } data;

    SPIClass* spi1;      // High speed sensors (LSM6, ADXL)
    SPIClass* spi2;      // Memory (SD, Flash)
    SPIClass* spi3;      // Thermal/pressure sensors (BMP, MAX)

    bool ledState = false;
    bool buzzerEnabled = true;
    bool buzzerState = false;
    bool handleSensors = true;
    bool inFlight = false;

    uint16_t error = 0;
    uint32_t packet = 0;

    const uint32_t watchdogInterval = WATCHDOG_INTERVAL;
    uint32_t handleSensorsInterval = SEND_INTERVAL_DEBUG / 2;  /// do zastanowienia się czy dzielić czy nie
    uint32_t dataSaveInterval = SEND_INTERVAL_DEBUG / 2;
    uint32_t msgSendInterval = SEND_INTERVAL_DEBUG;
    uint32_t buzzerInterval = BUZZER_INTERVAL_DEBUG;

    uint32_t lastWatchdogTime = 0;
    uint32_t lastHandleSensorsTime = 0;
    uint32_t lastDataSaveTime = 0;
    uint32_t lastMsgSendTime = 0;
    uint32_t lastBuzzerTime = 0;

    uint32_t startTime = 0;
    uint32_t afterburnStartTime = 0;
    uint32_t afterRisingStartTime = 0;
    uint32_t apogeeStartTime = 0;
    uint32_t touchdownStartTime = 0;

    bool solenoidActive = false;
    uint32_t solenoidStartTime = 0;
    uint8_t solenoidPulses = 0;
    uint32_t solenoidPulseDuration = SOLENOID_PULSE;
    uint32_t solenoidPulseInterval = SOLENOID_PULSE;
    uint8_t servoAngle = 5;

    BitStorage message;
    SX1262 radio;

    bool transmitting = false;
    bool messagePending = false;
    String pendingMessage = "";
    String receivedMessage = "";
    uint32_t messageStartTime = 0;

    bool sdReady = false, flashReady = false;
    uint32_t fileNumber = 0;
    String currentFileName;
    File32 SDDataFile;
    File32 flashDataFile;
    SdFat sd;
    Adafruit_FlashTransport_SPI flashTransport;
    Adafruit_SPIFlash flash;
    FatVolume fatfs;

    Servo myServo;
    TinyGPSPlus gps;
    HardwareSerial gpsSerial;
    Adafruit_LSM6DS lsm;
    Adafruit_BMP3XX bmp;
    Adafruit_ADXL343 adxl;
    Adafruit_MAX31855 max3;

    bool initializeRadio();                    // Initializes the LoRa radio module with the required parameter configuration
    void printRadioStatus();                   // Prints the radio's current configuration and status to the console/log
    void checkRadio();                         // Handles incoming LoRa messages (callback invoked when a packet is received)
    void transmit(String);                     // Sends a message over LoRa with buffering/queueing support
    void prepareMsg();                         // Prepares data for transmission over LoRa by packing it into a binary structure
    void startListening();                     // Starts listening on the LoRa channel for incoming messages
    void handleCommand(String);                // Executes an action in response to a received command string
    void sendMsg();                            // Sends the prepared message over LoRa and increments the packet counter
    void sendGpsOffset();                      // Sends GPS offset values to the ground station

    String prepareOffsetsMsg();                // Builds and returns a string message containing the sensor offsets
    String prepareDataLineMsg();               // Builds and returns a string message containing a single data line

    bool flashInit();                          // Initializes the SPI flash memory and mounts/creates a FAT filesystem
    bool SDInit();                             // Initializes the SD card and sets up the filesystem for use
    bool flashFindNextFileNumber();            // Finds the next available file number/index in flash storage
    bool flashOpenNewFile();                   // Opens a new CSV file in flash memory for writing telemetry/data
    bool flashWriteData(const String&);        // Writes the provided string data to the currently open flash file
    bool SDOpenNewFile();                      // Opens a new CSV file on the SD card for writing telemetry/data
    bool SDWriteData(const String&);           // Writes the provided string data to the currently open SD file
    bool writeRocketData();                    // Writes all current sensor and telemetry data to CSV files (flash and/or SD)

    void watchdog();                           // Monitors system components and attempts to recover or repair faulty modules
    void setOffsets();                         // Calibrates sensors by averaging multiple readings to determine offsets
    void handleGps();                          // Reads and processes data from the GPS receiver
    void handleLsm6();                         // Reads and processes data from the LSM6 gyroscope/accelerometer
    void handleAdxl();                         // Reads and processes data from the ADXL accelerometer
    void handleBmp();                          // Reads and processes data from the BMP barometer/altimeter
    void handleMax();                          // Reads and processes temperature from the MAX31855 thermocouple amplifier
    void handleBattery();                      // Reads the battery voltage and updates battery state information

    void parashuteOpen();                      // Initiates the parachute deployment procedure (activates solenoid)
    void activateSolenoid(uint8_t, uint32_t);  // Activates a solenoid valve with the specified number of pulses and pulse timing
    void updateSolenoid();                     // Updates solenoid state machine and pulse timing (call frequently to manage pulses)
    void updateBuzzer();                       // Updates buzzer state (tones, durations, alerts)
    void updateStatus();                       // Updates the rocket's flight status/state based on current sensor data

    void setFlightMode(bool);                  // Sets the inFlight flag (true = flight mode enabled, false = ground mode)
    void emergencyStop();                      // Performs an emergency stop procedure to halt the mission safely

  public:
    Rakieta();                                 // Constructor: initializes internal members, configures I/O pins, and clears error state

    void init();                               // Initializes all rocket systems, calibrates sensors, and positions servos
    void loop();                               // Main rocket loop: runs periodic tasks, sensor handling, communications, and state updates
};

#endif  // RAKIETA_H