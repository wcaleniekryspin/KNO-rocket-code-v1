#ifndef CONFIG_H
#define CONFIG_H

#define SERIAL_DEBUG 1
#if SERIAL_DEBUG == 1
  #define debugInit(x)  Serial.begin(x)
  #define debug(x)      Serial.print(x)
  #define debugln(x)    Serial.println(x)
  #define debugBin(x)   Serial.print(x, BIN)
  #define debugHex(x)   Serial.print(x, HEX)
  /// #define debugf(...)   Serial.printf(__VA_ARGS__)
#else
  #define debugInit(x)
  #define debug(x)
  #define debugln(x)
  #define debugBin(x)
  #define debugHex(x)
  /// #define debugf(...)
#endif

#define max(a, b) (((a) > (b)) ? (a) : (b))

// LORA / TIMING
#define RF_FREQUENCY                        868000000 // Hz
#define TX_OUTPUT_POWER                     5         // dBm
#define LORA_BANDWIDTH                      0         // [0: 125 kHz,
                                                  //  1: 250 kHz,
                                                  //  2: 500 kHz,
                                                  //  3: Reserved]
#define LORA_SPREADING_FACTOR               7         // [SF7..SF12]
#define LORA_CODINGRATE                     1         // [1: 4/5,
                                                  //  2: 4/6,
                                                  //  3: 4/7,
                                                  //  4: 4/8]
#define LORA_PREAMBLE_LENGTH                8
#define LORA_SYMBOL_TIMEOUT                 0
#define LORA_FIX_LENGTH_PAYLOAD_ON          false
#define LORA_IQ_INVERSION_ON                false

#define RX_TIMEOUT                          3000
#define RX_BUFFER_SIZE                      20  // w razie potrzeby zwiększyć
#define ARRAY_SIZE                          42
#define HEADER                              (0xFF66)

#define SEND_INTERVAL_DEBUG                 1000
#define SEND_INTERVAL_IDLE                  5000
#define SEND_INTERVAL_READY                 1000
#define SEND_INTERVAL_RISING                100
#define SEND_INTERVAL_FALLING               1000
#define SEND_INTERVAL_TOUCHDOWN             10000

// SENSORS / MEMORY
#define GPS_BANDWIDTH                       4800
#define SEA_LEVEL_PRESSURE_HPA              1013.25f
#define ADXL375_MG2G_MULTIPLIER             0.049f
#define FLASH_TYPE                          SPIFLASHTYPE_W25Q128
#define FLASH_SPI                           SPI

// SERVO
#define SERVO_OPEN_TIME                     150
#define SERVO_PWM_CHANNEL                   LEDC_CHANNEL_0
#define SERVO_PWM_TIMER                     LEDC_TIMER_0
#define SERVO_PWM_MODE                      LEDC_LOW_SPEED_MODE
#define SERVO_PWM_RESOLUTION                LEDC_TIMER_13_BIT
#define SERVO_PWM_FREQ                      50
#define SERVO_MIN_PULSE_WIDTH               544
#define SERVO_MAX_PULSE_WIDTH               2500

// BATTERY
#define BATTERY_MIN_VOLTAGE                 1.5f  // do zmiany na prawdziwe dane
#define BATTERY_MAX_VOLTAGE                 4.3f  // do zmiany na prawdziwe dane
#define BATTERY_CHECK_INTERVAL              50000

// PINS  /// WSZYSTKIE DO ZAMIANY NA RZECZYWISTE PINY, NIE UŻYWAMY JUŻ MCP !!
#define BATTERY                             1  // dedykowany do baterii
#define BUZZER                              3
#define SERVO                               4
#define LED_1                               5
#define LED_2                               6
#define SOLENOID                            7

#define GPS_RX                              16
#define GPS_TX                              17
#define SCK                                 18
#define MISO                                19
#define MOSI                                23

// SD 
// #define SD_USE_SDIO  /// odkomentować dla SDIO, zakomentować dla SPI
#define SD_SCK   13  
#define SD_MISO  14
#define SD_MOSI  15
#define SD_DETECT_PIN 13

#define LSM_CS                              0
#define ADXL_CS                             1
#define MAX_CS                              2
#define BMP_CS                              3
#define SD_CS                               4
#define FLASH_CS                            5

// ERROR CODES
#define LORA_ERROR                          _BV(0)
#define LSM_ERROR                           _BV(2)
#define BMP_ERROR                           _BV(3)
#define ADXL_ERROR                          _BV(4)
#define MAX_ERROR                           _BV(5)
#define SD_ERROR                            _BV(6)
#define SD_FILE_ERROR                       _BV(7)
#define FLASH_ERROR                         _BV(8)
#define FLASH_FILE_ERROR                    _BV(9)

// WATCHDOG / BUZZER
#define WATCHDOG_INTERVAL                   1000

#define BUZZER_INTERVAL_DEBUG               5000
#define BUZZER_INTERVAL_IDLE                10000
#define BUZZER_INTERVAL_READY               1000
#define BUZZER_INTERVAL_RISING_FALLING      65535
#define BUZZER_INTERVAL_TOUCHDOWN           10000

#define FREQUENCY_DEBUG                     1000  // trzeba zobaczyć co jest bardziej przyjemne co mniej
#define FREQUENCY_IDLE                      2000
#define FREQUENCY_READY                     5000  // tu nie może być przyjemne
#define FREQUENCY_RISING_FALLING            2000
#define FREQUENCY_TOUCHDOWN                 5000

// STATUS UPGRADE CONST
#define BURN_ACCEL_THRESHOLD                5.0f
#define MAX_FREEFALLING_SPEED               -15
#define BURN_ACCEL_CHECK_TIME               300
#define RISING_ACCEL_CHECK_TIME             3000
#define APOGEE_CHECK_TIME                   1000
#define MAX_RISING_TIME                     40000
#define MIN_PARACHUTE_TIME                  50000
#define TOUCHDOWN_CHECK_TIME                2000

// DATA LEN AND POSITIONS
#define timePos                             32
#define timeLen                             22
#define packetPos                           (timePos + timeLen)
#define packetLen                           22
#define errorPos                            (packetPos + packetLen)
#define errorLen                            16
#define statusPos                           (errorPos + errorLen)
#define statusLen                           3
#define gpsLatPos                           (statusPos + statusLen)
#define gpsLatLen                           17
#define gpsLngPos                           (gpsLatPos + gpsLatLen)
#define gpsLngLen                           17
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
#define gpsSpeedLen                         10
#define gpsCoursePos                        (gpsSpeedPos + gpsSpeedLen)
#define gpsCourseLen                        9
#define gpsSatNumPos                        (gpsCoursePos + gpsCourseLen)
#define gpsSatNumLen                        3
#define gpsHdopPos                          (gpsSatNumPos + gpsSatNumLen)
#define gpsHdopLen                          5
#define lsmAccelXPos                        (gpsHdopPos + gpsHdopLen)
#define lsmAccelXLen                        15
#define lsmAccelYPos                        (lsmAccelXPos + lsmAccelXLen)
#define lsmAccelYLen                        15
#define lsmAccelZPos                        (lsmAccelYPos + lsmAccelYLen)
#define lsmAccelZLen                        15
#define lsmGyroXPos                         (lsmAccelZPos + lsmAccelZLen)
#define lsmGyroXLen                         10
#define lsmGyroYPos                         (lsmGyroXPos + lsmGyroXLen)
#define lsmGyroYLen                         10
#define lsmGyroZPos                         (lsmGyroYPos + lsmGyroYLen)
#define lsmGyroZLen                         10
#define lsmSpeedPos                         (lsmGyroZPos + lsmGyroZLen)
#define lsmSpeedLen                         10
#define lsmTempPos                          (lsmSpeedPos + lsmSpeedLen)
#define lsmTempLen                          8
#define adxlAccelXPos                       (lsmTempPos + lsmTempLen)
#define adxlAccelXLen                       15
#define adxlAccelYPos                       (adxlAccelXPos + adxlAccelXLen)
#define adxlAccelYLen                       15
#define adxlAccelZPos                       (adxlAccelYPos + adxlAccelYLen)
#define adxlAccelZLen                       15
#define bmpTempPos                          (adxlAccelZPos + adxlAccelZLen)
#define bmpTempLen                          8
#define bmpPressPos                         (bmpTempPos + bmpTempLen)
#define bmpPressLen                         14
#define bmpAltiPos                          (bmpPressPos + bmpPressLen)
#define bmpAltiLen                          14
#define bmpSpeedPos                         (bmpAltiPos + bmpAltiLen)
#define bmpSpeedLen                         10
#define maxTempPos                          (bmpSpeedPos + bmpSpeedLen)
#define maxTempLen                          12
#define batteryPos                          (maxTempPos + maxTempLen)
#define batteryLen                          7

#endif  // CONFIG_H