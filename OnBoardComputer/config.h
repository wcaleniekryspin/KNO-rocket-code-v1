#ifndef CONFIG_H
#define CONFIG_H

#define SERIAL_DEBUG    1
#if SERIAL_DEBUG == 1
  #define debugInit(x)  Serial.begin(x)
  #define debug(x)      Serial.print(x)
  #define debugln(x)    Serial.println(x)
  #define debugBin(x)   Serial.print(x, BIN)
  #define debugHex(x)   Serial.print(x, HEX)
  #define debugf(...)   Serial.printf(__VA_ARGS__)
#else
  #define debugInit(x)
  #define debug(x)
  #define debugln(x)
  #define debugBin(x)
  #define debugHex(x)
  #define debugf(...)
#endif

#define BV16(x)         (uint16_t(1u) << (x))
#define max(a, b)       (((a) > (b)) ? (a) : (b))

// LoRa CONFIG
#define FREQUENCY                           868.0    // MHz
#define BANDWIDTH                           125.0    // kHz
#define SF                                  9        // 7-12
#define CODING_RATE                         5        // 5-8
#define POWER                               20       // dBm (do 17-22 dBm)
#define PREAMBLE_LENGTH                     15       // 6-30 symbols, the longer the symbols, the better the synchronization and range?, but the slower the transmission.

#define RX_TIMEOUT                          3000
#define ARRAY_SIZE                          42
#define HEADER                              (0xFF66)

// SX1262 pins
#define NSS                                 PE11   // Chip Select
#define DIO1                                PE10   // Digital IO 1
#define NRST                                PE9    // Reset
#define BUSY                                PE8    // Busy

// SPI1 - high-frequency sensors
#define SPI1_SCK                            PA5
#define SPI1_MISO                           PA6
#define SPI1_MOSI                           PA7

// SPI2 - memory
#define SPI2_SCK                            PB13
#define SPI2_MISO                           PB14
#define SPI2_MOSI                           PB15

// SPI3 - thermal/pressure sensors
#define SPI3_SCK                            PB3
#define SPI3_MISO                           PB4
#define SPI3_MOSI                           PB5

#define LSM_CS                              PA4     // LSM6DS3
#define ADXL_CS                             PB12    // ADXL375
#define BMP_CS                              PA15    // BMP388
#define MAX_CS                              PB0     // MAX31855
#define SD_CS                               PB1     // Karta SD
#define FLASH_CS                            PB10    // W25Q128

#define BATTERY                             PC0     // ADC_IN10 - voltage measurement
#define BUZZER                              PE13
#define SERVO                               PB8
#define LED_1                               PE14
#define LED_2                               PE15
#define SOLENOID                            PE12

#define GPS_RX                              PA10
#define GPS_TX                              PA9

// TIMING CONFIG
#define SEND_INTERVAL_DEBUG                 1000
#define SEND_INTERVAL_IDLE                  5000
#define SEND_INTERVAL_READY                 1000
#define SEND_INTERVAL_BURN                  100
#define SEND_INTERVAL_RISING                100
#define SEND_INTERVAL_FALLING               500
#define SEND_INTERVAL_TOUCHDOWN             10000

// WATCHDOG / BUZZER
#define WATCHDOG_INTERVAL                   2500
#define BUZZER_INTERVAL_DEBUG               5000
#define BUZZER_INTERVAL_IDLE                10000
#define BUZZER_INTERVAL_READY               1000
#define BUZZER_INTERVAL_BURN                65535
#define BUZZER_INTERVAL_RISING              65535
#define BUZZER_INTERVAL_FALLING             65535
#define BUZZER_INTERVAL_TOUCHDOWN           10000

// SENSORS / MEMORY
#define GPS_BAUDRATE                        4800
#define SEA_LEVEL_PRESSURE_HPA              1013.25f
#define ADXL375_MG2G_MULTIPLIER             (0.049f / ADXL343_MG2G_MULTIPLIER)

// SERVO / SOLENOID
#define SERVO_OPEN_TIME                     150
#define SOLENOID_PULSE                      150
#define SOLENOID_PULSE_COUNT                3  // number of pulses

// STATUS UPGRADE CONST      /// tu trzeba wpisać dane z symulacji
#define BURN_ACCEL_THRESHOLD                5.0f
#define MAX_FREEFALLING_SPEED               (-20.0f)
#define BURN_ACCEL_CHECK_TIME               300
#define RISING_ACCEL_CHECK_TIME             3000
#define APOGEE_CHECK_TIME                   1000
#define MAX_RISING_TIME                     45000
#define MIN_PARACHUTE_TIME                  (MAX_RISING_TIME + 5000)
#define TOUCHDOWN_CHECK_TIME                2000

// ERROR CODES
#define LORA_ERROR                          BV16(0)
#define LSM_ERROR                           BV16(1)
#define BMP_ERROR                           BV16(2)
#define ADXL_ERROR                          BV16(3)
#define MAX_ERROR                           BV16(4)
#define SD_ERROR                            BV16(5)
#define SD_FILE_ERROR                       BV16(6)
#define FLASH_ERROR                         BV16(7)
#define FLASH_FILE_ERROR                    BV16(8)

// DATA LEN AND POSITIONS
// HEADER for 16 bits
#define timePos                             16
#define timeLen                             22
#define packetPos                           (timePos + timeLen)
#define packetLen                           16
#define errorPos                            (packetPos + packetLen)
#define errorLen                            16
#define statusPos                           (errorPos + errorLen)
#define statusLen                           4
#define handlePos                           (statusPos + statusLen)
#define handleLen                           1

#define gpsLatPos                           (handlePos + handleLen)
#define gpsLatLen                           (17+1)
#define gpsLngPos                           (gpsLatPos + gpsLatLen)
#define gpsLngLen                           (17+1)
#define gpsAltiPos                          (gpsLngPos + gpsLngLen)
#define gpsAltiLen                          14
#define gpsHourPos                          (gpsAltiPos + gpsAltiLen)
#define gpsHourLen                          5
#define gpsMinPos                           (gpsHourPos + gpsHourLen)
#define gpsMinLen                           6
#define gpsSecPos                           (gpsMinPos + gpsMinLen)
#define gpsSecLen                           6
#define gpsCentisecPos                      (gpsSecPos + gpsSecLen)
#define gpsCentisecLen                      7
#define gpsSpeedPos                         (gpsCentisecPos + gpsCentisecLen)
#define gpsSpeedLen                         (14+1)
#define gpsCoursePos                        (gpsSpeedPos + gpsSpeedLen)
#define gpsCourseLen                        9
#define gpsSatNumPos                        (gpsCoursePos + gpsCourseLen)
#define gpsSatNumLen                        3
#define gpsHdopPos                          (gpsSatNumPos + gpsSatNumLen)
#define gpsHdopLen                          5

#define lsmAccelXPos                        (gpsHdopPos + gpsHdopLen)
#define lsmAccelXLen                        (14+1)
#define lsmAccelYPos                        (lsmAccelXPos + lsmAccelXLen)
#define lsmAccelYLen                        (14+1)
#define lsmAccelZPos                        (lsmAccelYPos + lsmAccelYLen)
#define lsmAccelZLen                        (14+1)
#define lsmGyroXPos                         (lsmAccelZPos + lsmAccelZLen)
#define lsmGyroXLen                         (14+1)
#define lsmGyroYPos                         (lsmGyroXPos + lsmGyroXLen)
#define lsmGyroYLen                         (14+1)
#define lsmGyroZPos                         (lsmGyroYPos + lsmGyroYLen)
#define lsmGyroZLen                         (14+1)
#define lsmTempPos                          (lsmGyroZPos + lsmGyroZLen)
#define lsmTempLen                          (9+1)
#define lsmSpeedPos                         (lsmTempPos + lsmTempLen)
#define lsmSpeedLen                         (10+1)

#define adxlAccelXPos                       (lsmSpeedPos + lsmSpeedLen)
#define adxlAccelXLen                       (15+1)
#define adxlAccelYPos                       (adxlAccelXPos + adxlAccelXLen)
#define adxlAccelYLen                       (15+1)
#define adxlAccelZPos                       (adxlAccelYPos + adxlAccelYLen)
#define adxlAccelZLen                       (15+1)
#define adxlSpeedPos                        (adxlAccelZPos + adxlAccelZLen)
#define adxlSpeedLen                        (14+1)

#define bmpTempPos                          (adxlSpeedPos + adxlSpeedLen)
#define bmpTempLen                          (9+1)
#define bmpPressPos                         (bmpTempPos + bmpTempLen)
#define bmpPressLen                         14
#define bmpAltiPos                          (bmpPressPos + bmpPressLen)
#define bmpAltiLen                          14
#define bmpSpeedPos                         (bmpAltiPos + bmpAltiLen)
#define bmpSpeedLen                         (14+1)

#define maxTempPos                          (bmpSpeedPos + bmpSpeedLen)
#define maxTempLen                          (9+1)
#define batteryPos                          (maxTempPos + maxTempLen)
#define batteryLen                          7

// CHECKSUM for 8 bits

#endif  // CONFIG_H