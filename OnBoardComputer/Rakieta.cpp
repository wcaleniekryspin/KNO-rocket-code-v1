#include "Rakieta.h"

/***************************************************
chyba DONE:
 - reakcja na komendy
 - działanie funkcji parashuteOpen()
 - dodać zmianę parametrów (buzzer, msgDelay i inne) w funkcji updateStatus()

 -- dodać opcję wyłączenia zbierania danych handleSensors = true/false
 -- dodać mnożniki do wartości w funkcji prepareMsg()
 - zmiana softwareserial gps na hardwareserial
    W Rakieta.h: HardwareSerial gpsSerial(PA10, PA9);
    W setup(): gpsSerial.begin(9600);
 -- dodać timeout w funkcji transmit(String)
    while (!operationDone && (millis() - messageStartTime >= TX_TIMEOUT))
 -- dodać funkcję zapisu do pliku czasów xStartTime po touchdown
 - do zmiany większość STATUS UPGRADE CONST w pliku config.h !!!
 -- dodać funkcję emergencyStop() (odcina tlen i wyrzuca spadochron)

 -- zastanowić się czy wychodzić z funkcji handleSensor() jeśli jest błąd na sensorze (NIE)
 -- zastanowić się czy robić reset jeśli LoRa nie działa (TAK bo tylko w inFlight == false)

TO DO:
 -- dodanie trybu oszczędzania energii
    #include <STM32LowPower.h>
    void enterSleepMode() {
      if (!inFlight) {
        lsm.end();
        bmp.end();
        // ...

        LowPower.enableWakeupFrom(GPIOA, PA0, RISING);  // coś innego
        LowPower.sleep();

        spi1.begin();
        // ...
      }
    }

 --- czy chcemy dodać czujnik ciśnienia do komory spalania??
     kod do odczytu ciśnienia
     kod PID do sterowania serwem

 ---- dodać hardware watchdog w trybach debug/idle/ready (może przez flagę bool inFlight)
 ---- teraz jest jedna termopara trzeba dodać więcej (na przyszłość)
 ---- Dodać listę wiadomości do buforowania (bardzo mało ważne raczej, jak znikną dane w transmisji to trudno)

***************************************************/

Rakieta::Rakieta():
  status(State::debug),
  spi1(SPI1_SCK, SPI1_MISO, SPI1_MOSI),
  spi2(SPI2_SCK, SPI2_MISO, SPI2_MOSI),
  spi3(SPI3_SCK, SPI3_MISO, SPI3_MOSI),
  spi4(SPI4_SCK, SPI4_MISO, SPI4_MOSI),
  gpsSerial(GPS_RX, GPS_TX),
  lsm(), bmp(), adxl((uint8_t)255, &spi3), max3((uint8_t)255, &spi4),
  flashTransport(FLASH_CS, spi1), flash(&flashTransport),
  radio(new Module(NSS, DIO1, NRST, BUSY))
{
  error |= LORA_ERROR | LSM_ERROR | BMP_ERROR | ADXL_ERROR | MAX_ERROR |
           SD_ERROR | SD_FILE_ERROR | FLASH_ERROR | FLASH_FILE_ERROR;

  pinMode(BATTERY, INPUT_ANALOG);
  pinMode(BUZZER, OUTPUT);
  pinMode(SERVO, OUTPUT);
  pinMode(LED_1, OUTPUT);
  pinMode(LED_2, OUTPUT);
  pinMode(SOLENOID, OUTPUT);

  pinMode(LSM_CS, OUTPUT);
  pinMode(ADXL_CS, OUTPUT);
  pinMode(BMP_CS, OUTPUT);
  pinMode(MAX_CS, OUTPUT);
  pinMode(SD_CS, OUTPUT);
  pinMode(FLASH_CS, OUTPUT);

  digitalWrite(LSM_CS, HIGH);
  digitalWrite(ADXL_CS, HIGH);
  digitalWrite(BMP_CS, HIGH);
  digitalWrite(MAX_CS, HIGH);
  digitalWrite(SD_CS, HIGH);
  digitalWrite(FLASH_CS, HIGH);
}

Rakieta::~Rakieta()
{
  radio.clearDio1Action();
  myServo.detach();

  if (SDDataFile) SDDataFile.close();
  if (flashDataFile) flashDataFile.close();
  
  digitalWrite(BUZZER, LOW);
  digitalWrite(SOLENOID, LOW);
  digitalWrite(LED_1, LOW);
  digitalWrite(LED_2, LOW);

  // delete spi1; delete spi2; delete spi3; delete spi4;
}

void Rakieta::init()
{
  debugln("Initializing rocket systems...");

  gpsSerial.begin(GPS_BAUDRATE);
  debugln("GPS UART initialized");

  // Init all sensors
  watchdog();
  startListening();

  if (error == 0)
  {
    debugln("All systems OK");
    digitalWrite(LED_2, LOW);
  }
  else
  {
    debug("Errors present: ");
    debugBin(error);
    digitalWrite(LED_2, HIGH);
  }
  
  delay(100);
  setOffsets();

  // Move servo to position 0
  myServo.attach(SERVO);
  myServo.write(0);

  setFlightMode(false);
}

void Rakieta::loop()
{
  // 1. HIGHEST PRIORITY: RECOVERY SYSTEM
  updateSolenoid();  // Sterowanie elektrozaworem
  updateStatus();  // Decyzje o otwarciu spadochronu

  // 2. CRITICAL: EMERGENCY SAFETY CHECKS
  if (inFlight) { emergencyStop(); }

  if (handleSensors)
  {
    // 3. MEDIUM PRIORITY: DATA SAVING
    uint32_t now = millis();
    if (now - lastDataSaveTime >= dataSaveInterval)
    {
      lastDataSaveTime = now;
      writeRocketData();
    }

    // 4. MEDIUM PRIORITY: SENSOR READING
    now = millis();
    if (now - lastHandleSensorsTime >= handleSensorsInterval)
    {
      lastHandleSensorsTime = now;
      handleGps();
      handleLsm6();
      handleAdxl();
      handleBmp();
      handleMax();
      handleBattery();
    }
  }

  // 5. MEDIUM/LOW PRIORITY: RADIO CHECK
  checkRadio();

  // 6. LOW PRIORITY: WATCHDOG (only when NOT in flight)
  if (!inFlight)
  {
    uint32_t now = millis();
    if (now - lastWatchdogTime >= watchdogInterval)
    {
      lastWatchdogTime = now;
      watchdog();
    }
  }

  // 7. LOW PRIORITY: MESSAGE SENDING
  uint32_t now = millis();
  if (now - lastMsgSendTime >= msgSendInterval)
  {
    lastMsgSendTime = now;
    sendMsg();
  }

  // 8. VERY LOW PRIORITY: BUZZER
  updateBuzzer();
}

bool Rakieta::initializeRadio()
{
  debugln("Initialization SX1262");
  int state = radio.begin(FREQUENCY);
  if (state != RADIOLIB_ERR_NONE)
  {
    debug("Error: ");
    debugln(state);
    error |= LORA_ERROR;
    return false;
  }
  
  // LoRa parameter configuration
  state = radio.setBandwidth(BANDWIDTH);
  state |= radio.setSpreadingFactor(SF);
  state |= radio.setCodingRate(CODING_RATE);
  state |= radio.setOutputPower(POWER);
  state |= radio.setPreambleLength(PREAMBLE_LENGTH);
  
  if (state != RADIOLIB_ERR_NONE)
  {
    debug("Configuration error: ");
    debugln(state);
    error |= LORA_ERROR;
    return false;
  }

  radio.setDio1Action(setOperationFlag);
  printRadioStatus();
  debugln(F("Radio ready - listening..."));
  error &= ~LORA_ERROR;
  return true;
}

void Rakieta::startListening()
{
  int state = radio.startReceive();
  if (state != RADIOLIB_ERR_NONE)
  {
    debug(F("Error starting listening: "));
    debugln(state);
  }
  transmitting = false;
}

void Rakieta::handleCommand(String command)
{
  if (command == "TO_DEBUG")
  {
    setFlightMode(false);
    status = State::debug;
    handleSensorsInterval = SEND_INTERVAL_DEBUG / 4;
    dataSaveInterval = SEND_INTERVAL_DEBUG / 4;
    msgSendInterval = SEND_INTERVAL_DEBUG;
    buzzerInterval = BUZZER_INTERVAL_DEBUG;
  }
  else if (command == "TO_IDLE")
  {
    setFlightMode(false);
    status = State::idle;
    handleSensorsInterval = SEND_INTERVAL_IDLE / 4;
    dataSaveInterval = SEND_INTERVAL_IDLE / 4;
    msgSendInterval = SEND_INTERVAL_IDLE;
    buzzerInterval = BUZZER_INTERVAL_IDLE;
  }
  else if (command == "TO_READY")
  {
    setFlightMode(false);
    status = State::ready;
    handleSensorsInterval = SEND_INTERVAL_READY / 4;
    dataSaveInterval = SEND_INTERVAL_READY / 4;
    msgSendInterval = SEND_INTERVAL_READY;
    buzzerInterval = BUZZER_INTERVAL_READY;
  }
  else if (command == "TO_BURN")
  {
    setFlightMode(true);
    status = State::burn;
    handleSensorsInterval = SEND_INTERVAL_BURN / 4;
    dataSaveInterval = SEND_INTERVAL_BURN / 4;
    msgSendInterval = SEND_INTERVAL_BURN;
    buzzerInterval = BUZZER_INTERVAL_BURN;
    startTime = millis();
    myServo.write(90);
  }
  else if (command == "TO_FALLING")
  {
    setFlightMode(true);
    status = State::falling;
    handleSensorsInterval = SEND_INTERVAL_FALLING / 4;
    dataSaveInterval = SEND_INTERVAL_FALLING / 4;
    msgSendInterval = SEND_INTERVAL_FALLING;
    buzzerInterval = BUZZER_INTERVAL_FALLING;
    myServo.write(0);
    parashuteOpen();
  }
  else if (command == "SERVO_TEST") { myServo.write(servoAngle); delay(100); myServo.write(0); }
  else if (command == "SERVO_PLUS") { if (servoAngle < 45) servoAngle += 2; }
  else if (command == "SERVO_MINUS") { if (servoAngle > 2) servoAngle -= 2; }
  else if (command == "GET_SERVO_ANGLE") { transmit(String(servoAngle)); }
  else if (command == "BEEP_ON") { buzzerEnabled = true; }
  else if (command == "BEEP_OFF") { buzzerEnabled = false; }
  else if (command == "RESET") { NVIC_SystemReset(); }  /// trzeba zobaczyć czy działa jak nie to jakoś inaczej (NIBY SIĘ ROBI KOMPILACJA WIĘC TRZREBA PRZETESTOWAĆ)
  else if (command == "CALIBRATE") { setOffsets(); }
  else if (command == "GET_GPS_OFFSET") { sendGpsOffset(); }
  else if (command == "HANDLE_SENSORS_ON") { handleSensors = true; }
  else if (command == "HANDLE_SENSORS_OFF") { handleSensors = false; }
  else if (command == "EMERGENCY_STOP") { emergencyStop(); }
}

void Rakieta::checkRadio()
{
  String msg;
  int state = radio.readData(msg);

  if (state == RADIOLIB_ERR_NONE)
  {
    debugln(F("== MESSAGE RECEIVED =="));
    debug(F("Message: "));
    debugln(msg);
    debug(F("RSSI: "));
    debug(radio.getRSSI());
    debug(F(" dBm, "));
    debug(F("SNR: "));
    debug(radio.getSNR());
    debugln(F(" dB"));

    handleCommand(msg);
  }
  else
  {
    debug(F("Data reading error: "));
    debugln(state);
  }
}

void Rakieta::printRadioStatus()
{
  debugln(F("\n=== RADIO STATUS ==="));
  debug(F("Frequency: "));
  debug(FREQUENCY);
  debugln(F(" MHz"));
  debug(F("Moc: "));
  debug(POWER);
  debugln(F(" dBm"));
  debug(F("SF: "));
  debugln(SF);
  debug(F("BW: "));
  debug(BANDWIDTH);
  debugln(F(" kHz"));
  debug(F("CR: 4/"));
  debugln(CODING_RATE);
  debugln(F("===================\n"));
}

void Rakieta::transmit(String msg)
{
  if (msg.length() > 255)
  {
    debugln(F("Error: Message too long"));
    return;
  }

  if (!operationDone)
  {
    if (millis() - messageStartTime >= TX_TIMEOUT)
    {
      debugln("ERROR: LoRa timeout, forcing radio reset");
      radio.standby();
      delay(2);
      startListening();
    }
    else
    {
      debugln("LoRa busy");
      return;
    }
  }
  
  debug(F("Sending: "));
  debugln(msg);

  operationDone = false;
  messageStartTime = millis();
  int state = radio.startTransmit(msg);

  if (state != RADIOLIB_ERR_NONE)
  {
    debug(F("Transmit error: "));
    debugln(state);
    operationDone = true;
    startListening();
  }
}

void Rakieta::transmit(uint8_t *msg, size_t len)
{
  if (len > 255)
  {
    debugln(F("Error: Message too long"));
    return;
  }

  if (!operationDone)
  {
    if (millis() - messageStartTime >= TX_TIMEOUT)
    {
      debugln("ERROR: LoRa timeout, forcing radio reset");
      radio.standby();
      delay(2);
      startListening();
    }
    else
    {
      debugln("LoRa busy");
      return;
    }
  }
  
  debug(F("Sending: "));
  debugln(*msg);

  operationDone = false;
  messageStartTime = millis();
  int state = radio.startTransmit(msg, len);

  if (state != RADIOLIB_ERR_NONE)
  {
    debug(F("Transmit error: "));
    debugln(state);
    operationDone = true;
    startListening();
  }
}

bool Rakieta::flashInit()
{
  if (!flash.begin())
  {
    debugln("Flash initialization failed!");
    flashReady = false;
    return false;
  }

  debug("Flash chip JEDEC ID: 0x"); debugHex(flash.getJEDECID());
  debug("Flash size: "); debugln(flash.size());

  if (!fatfs.begin(&flash))
  {
    debugln("Failed to mount filesystem!");
    flashReady = false;
    return false;
  }

  debugln("SPI Flash filesystem mounted successfully!");
  if (!flashFindNextFileNumber())
  {
    debugln("Failed to find next file number");
    sdReady = false;
    return false;
  }

  if (!flashOpenNewFile())
  {
    debugln("Failed to open new file");
    flashReady = false;
    return false;
  }
  
  flashReady = true;
  debugln("SPI Flash initialized successfully!");
  return true;
}

bool Rakieta::flashFindNextFileNumber()
{
  fileNumber = 0;
  while (fileNumber < 1000) // Max 1000 files
  {
    currentFileName = "data_";
    if (fileNumber < 10) currentFileName += "00";
    else if (fileNumber < 100) currentFileName += "0";
    currentFileName += String(fileNumber);
    currentFileName += ".csv";
    
    if (!fatfs.exists(currentFileName.c_str()))
    {
      break;
    }
      
    fileNumber++;
  }
  
  if (fileNumber >= 1000)
  {
    debugln("Too many data files!");
    return false;
  }
  
  debug("Creating new file: "); debugln(currentFileName.c_str());
  return true;
}

bool Rakieta::flashOpenNewFile()
{
  if (flashDataFile)
  {
    flashDataFile.close();
  }
  
  flashDataFile = fatfs.open(currentFileName.c_str(), FILE_WRITE);
  if (!flashDataFile)
  {
    debugln("Failed to open file for writing");
    return false;
  }
  
  flashWriteData("timestamp,packet,status,error,"
                 "gps_lat,gps_lng,gps_alt,gps_h,gps_m,gps_s,gps_c,"
                 "gps_speed,gps_course,gps_satNum,gps_hdop,"
                 "lsm_ax,lsm_ay,lsm_az,lsm_gx,lsm_gy,lsm_gz,lsm_temp,lsm_speed,"
                 "adxl_ax,adxl_ay,adxl_az,adxl_speed,"
                 "bmp_temp,bmp_press,bmp_alt,bmp_speed,"
                 "max_temp,battery");
  
  debug("File opened: "); debugln(currentFileName.c_str());
  return true;
}

bool Rakieta::flashWriteData(const String& data)
{
  if (!flashReady || !flashDataFile) return false;
  
  flashDataFile.println(data);
  
  // Flush data every 10 writes
  static uint16_t flashWriteCount = 0;
  flashWriteCount++;
  if (flashWriteCount >= 10)
  {
    flashDataFile.flush();
    flashWriteCount = 0;
  }
  
  return true;
}

bool Rakieta::SDInit()
{
  debugln("Initializing SD card...");

  if (!sd.begin(SdSpiConfig(SD_CS, 2, SPI2_SPEED, &spi2)))  /// do kontroli
  {
    debugln("Card Mount Failed");
    error |= SD_ERROR;
    sdReady = false;
    return false;
  }
  
  if (!SDOpenNewFile())
  {
    debugln("Failed to open new file");
    sdReady = false;
    return false;
  }
  
  sdReady = true;
  debugln("SD card initialized successfully!");
  return true;
}

bool Rakieta::SDOpenNewFile()
{
  if (SDDataFile)
  {
    SDDataFile.close();
  }
  
  SDDataFile = sd.open(currentFileName.c_str(), FILE_WRITE);
  if (!SDDataFile)
  {
    debugln("Failed to open file for writing");
    return false;
  }
  
  SDWriteData("timestamp,packet,status,error,"
              "gps_lat,gps_lng,gps_alt,gps_h,gps_m,gps_s,gps_c,"
              "gps_speed,gps_course,gps_satNum,gps_hdop,"
              "lsm_ax,lsm_ay,lsm_az,lsm_gx,lsm_gy,lsm_gz,lsm_temp,lsm_speed,"
              "adxl_ax,adxl_ay,adxl_az,adxl_speed,"
              "bmp_temp,bmp_press,bmp_alt,bmp_speed,"
              "max_temp,battery");
  
  debug("File opened: "); debugln(currentFileName.c_str());
  return true;
}

bool Rakieta::SDWriteData(const String& data)
{
  if (!sdReady || !SDDataFile) return false;
  
  SDDataFile.println(data);
  
  // Flush data every 10 writes
  static uint16_t SDWriteCount = 0;
  SDWriteCount++;
  if (SDWriteCount >= 10)
  {
    SDDataFile.flush();
    SDWriteCount = 0;
  }
  
  return true;
}

void Rakieta::writeRocketData()
{
  String msg = prepareDataLineMsg();
  debugln(msg);
  
  if (flashReady)
  {
    if (!flashWriteData(msg))
    {
      error |= FLASH_FILE_ERROR;
      debugln("Flash write failed!");
    }
  }
  if (sdReady)
  {
    if (!SDWriteData(msg))
    {
      error |= SD_FILE_ERROR;
      debugln("SD write failed!");
    }
  }
}

void Rakieta::watchdog()
{
  if (inFlight) { return; }

  if (error & LORA_ERROR)
  {
    uint8_t times = 0;
    while (!initializeRadio() && times < 10)
    {
      error |= LORA_ERROR;
      debugln(F("Radio initialization error!"));
      times++;
    }

    if (times >= 10) { NVIC_SystemReset(); }
    else
    {
      error &= ~LORA_ERROR;
      startListening();
      delay(10);
    }
  }

  if (error & LSM_ERROR)
  {
    if (!lsm.begin_SPI(255, &spi3))
    {
      debugln("Cannot init lsm (LSM6DS3)");
      error |= LSM_ERROR;
    }
    else
    {
      lsm.setAccelRange(LSM6DS_ACCEL_RANGE_16_G);
      lsm.setGyroRange(LSM6DS_GYRO_RANGE_2000_DPS);
      lsm.setAccelDataRate(LSM6DS_RATE_416_HZ);
      lsm.setGyroDataRate(LSM6DS_RATE_416_HZ);
      error &= ~LSM_ERROR;
    }
    delay(10);
  }
  
  if (error & BMP_ERROR)
  {
    if (!bmp.begin_SPI(255, &spi4))
    {
      debugln("Cannot init bmp");
      error |= BMP_ERROR;
    }
    else
    {
      // Set up oversampling and filter initialization
      bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_4X);  // 1X, 2X, 4X, 8X, 16X, 32X mean from N samples
      bmp.setPressureOversampling(BMP3_OVERSAMPLING_4X);  // 1X, 2X, 4X, 8X, 16X, 32X mean from N samples
      bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_1);  // 0, 1, 3, 7, 15, 31, 63, 127 noise reduction
      bmp.setOutputDataRate(BMP3_ODR_200_HZ);   // ...1, 10, 25, 50, 100, 200 Hz frequency
      error &= ~BMP_ERROR;
    }
    delay(10);
  }
  
  if (error & ADXL_ERROR)
  {
    if (!adxl.begin())
    {
      debugln("Cannot init ADXL375");
      error |= ADXL_ERROR;
    }
    else
    {
      error &= ~ADXL_ERROR;
    }
    delay(10);
  }

  if (error & MAX_ERROR)
  {
    if (!max3.begin())
    {
      debugln("Cannot init MAX31855");
      error |= MAX_ERROR;
    }
    else
    {
      error &= ~MAX_ERROR;
      uint8_t e = max3.readError();
      if (e & MAX31855_FAULT_OPEN)
      {
        debugln("FAULT: Thermocouple is open - no connections.");
        error |= MAX_ERROR;
      }
      if (e & MAX31855_FAULT_SHORT_GND)
      {
        debugln("FAULT: Thermocouple is short-circuited to GND.");
        error |= MAX_ERROR;
      }
      if (e & MAX31855_FAULT_SHORT_VCC)
      {
        debugln("FAULT: Thermocouple is short-circuited to VCC.");
        error |= MAX_ERROR;
      }
    }
    delay(10);
  }
  
  if ((error & SD_ERROR) || (error & SD_FILE_ERROR))
  {
    SDInit();
    delay(10);
  }

  if ((error & FLASH_ERROR) || (error & FLASH_FILE_ERROR))
  {
    flashInit();
    delay(10);
  }
}

void Rakieta::prepareMsg()
{
  uint32_t now = millis();
  message.add(uint32_t(now / 10), timePos, timeLen);
  message.add(uint16_t(packet), packetPos, packetLen);
  message.add(uint16_t(error), errorPos, errorLen);
  message.add(uint8_t(status), statusPos, statusLen);
  message.add(uint8_t(handleSensors), handlePos, handleLen);

  message.add(int16_t(data.gps.lat * 100000), gpsLatPos, gpsLatLen, true);
  message.add(int16_t(data.gps.lng * 100000), gpsLngPos, gpsLngLen, true);
  message.add(uint16_t(data.gps.altiM * 10), gpsAltiPos, gpsAltiLen);
  message.add(uint8_t(data.gps.h), gpsHourPos, gpsHourLen);
  message.add(uint8_t(data.gps.m), gpsMinPos, gpsMinLen);
  message.add(uint8_t(data.gps.s), gpsSecPos, gpsSecLen);
  message.add(uint8_t(data.gps.centi), gpsCentisecPos, gpsCentisecLen);
  message.add(int16_t(data.gps.speed * 10), gpsSpeedPos, gpsSpeedLen, true);
  message.add(uint16_t(data.gps.course), gpsCoursePos, gpsCourseLen);
  message.add(uint8_t(data.gps.satNum), gpsSatNumPos, gpsSatNumLen);
  message.add(uint8_t(data.gps.hdop), gpsHdopPos, gpsHdopLen);
  
  message.add(int16_t(data.lsm.ax * 100), lsmAccelXPos, lsmAccelXLen, true);
  message.add(int16_t(data.lsm.ay * 100), lsmAccelYPos, lsmAccelYLen, true);
  message.add(int16_t(data.lsm.az * 100), lsmAccelZPos, lsmAccelZLen, true);
  message.add(int16_t(data.lsm.gx * 10), lsmGyroXPos, lsmGyroXLen, true);
  message.add(int16_t(data.lsm.gy * 10), lsmGyroYPos, lsmGyroYLen, true);
  message.add(int16_t(data.lsm.gz * 10), lsmGyroZPos, lsmGyroZLen, true);
  message.add(int8_t(data.lsm.temp), lsmTempPos, lsmTempLen, true);
  message.add(int16_t(data.lsm.lastTotalSpeed * 10), lsmSpeedPos, lsmSpeedLen, true);
  
  message.add(int16_t(data.adxl.ax * 10), adxlAccelXPos, adxlAccelXLen, true);
  message.add(int16_t(data.adxl.ay * 10), adxlAccelYPos, adxlAccelYLen, true);
  message.add(int16_t(data.adxl.az * 10), adxlAccelZPos, adxlAccelZLen, true);
  message.add(int16_t(data.adxl.lastTotalSpeed * 10), adxlSpeedPos, adxlSpeedLen, true);

  message.add(int8_t(data.bmp.temp), bmpTempPos, bmpTempLen, true);
  message.add(uint16_t(data.bmp.pressure * 10), bmpPressPos, bmpPressLen);
  message.add(uint16_t(data.bmp.altitude * 10), bmpAltiPos, bmpAltiLen);
  message.add(int16_t(data.bmp.lastVerticalSpeed * 10), bmpSpeedPos, bmpSpeedLen, true);

  message.add(int16_t(data.max.temp), maxTempPos, maxTempLen, true);
  message.add(uint8_t(data.battery.voltage * 10), batteryPos, batteryLen);
}

void Rakieta::sendMsg()
{
  packet++;
  prepareMsg();
  uint8_t *txPacket = message.data();

  debugln("\n\nSending: ");

  for (int i=0; i<ARRAY_SIZE; i++)
    debugHex(txPacket[i]);
  debugln("");

  for (int i=0; i<ARRAY_SIZE; i++)
    for (int j=7; j>=0; --j)
      debug((txPacket[i] >> j) & 1);

  debugln("\nEnd of the message.\n\n");

  transmit(txPacket, ARRAY_SIZE);

  ledState = !ledState;
  digitalWrite(LED_1, ledState);

  debugln("Sending DONE");
}

String Rakieta::prepareOffsetsMsg()
{
  String msg = "Valid num:Lsm:";
  msg += String(validLsm);
  msg += ",Adxl:";
  msg += String(validAdxl);
  msg += ",Bmp:";
  msg += String(validBmp);
  msg += ",GPS:";
  msg += String(validGPS);
  msg += "Offsets:Lsm:{ax:";
  msg += String(offsets.lsm.ax);
  msg += ",ay:";
  msg += String(offsets.lsm.ay);
  msg += ",az:";
  msg += String(offsets.lsm.az);
  msg += ",gx:";
  msg += String(offsets.lsm.gx);
  msg += ",gy:";
  msg += String(offsets.lsm.gy);
  msg += ",gz:";
  msg += String(offsets.lsm.gz);
  msg += "}Adxl:{ax:";
  msg += String(offsets.adxl.ax);
  msg += ",ay:";
  msg += String(offsets.adxl.ay);
  msg += ",az:";
  msg += String(offsets.adxl.az);
  msg += "}GPS:{lat:";
  msg += String(offsets.gps.lat);
  msg += ",lng:";
  msg += String(offsets.gps.lng);
  msg += ",altiM:";
  msg += String(offsets.gps.altiM);
  msg += ",speed:";
  msg += String(offsets.gps.speed);
  msg += "}";
  return msg;
}

String Rakieta::prepareDataLineMsg()
{
  String msg = "RocketData:";

  // Timestamp
  msg += String(millis());
  msg += ",";
  msg += String(packet);
  msg += ",";
  msg += String(static_cast<int>(status));
  msg += ",";
  msg += String(error, HEX);
  msg += ",";
  
  // GPS
  msg += String(data.gps.lat, 6);
  msg += ",";
  msg += String(data.gps.lng, 6);
  msg += ",";
  msg += String(data.gps.altiM, 2);
  msg += ",";
  msg += String(data.gps.h);
  msg += ",";
  msg += String(data.gps.m);
  msg += ",";
  msg += String(data.gps.s);
  msg += ",";
  msg += String(data.gps.centi);
  msg += ",";
  msg += String(data.gps.speed, 2);
  msg += ",";
  msg += String(data.gps.course, 0);
  msg += ",";
  msg += String(data.gps.satNum);
  msg += ",";
  msg += String(data.gps.hdop);
  msg += ",";
  
  // LSM6
  msg += String(data.lsm.ax, 4);
  msg += ",";
  msg += String(data.lsm.ay, 4);
  msg += ",";
  msg += String(data.lsm.az, 4);
  msg += ",";
  msg += String(data.lsm.gx, 3);
  msg += ",";
  msg += String(data.lsm.gy, 3);
  msg += ",";
  msg += String(data.lsm.gz, 3);
  msg += ",";
  msg += String(data.lsm.temp, 2);
  msg += ",";
  msg += String(data.lsm.lastTotalSpeed, 4);
  msg += ",";

  // ADXL
  msg += String(data.adxl.ax, 4);
  msg += ",";
  msg += String(data.adxl.ay, 4);
  msg += ",";
  msg += String(data.adxl.az, 4);
  msg += ",";
  msg += String(data.adxl.lastTotalSpeed, 4);
  msg += ",";
  
  // BMP
  msg += String(data.bmp.temp, 2);
  msg += ",";
  msg += String(data.bmp.pressure, 2);  /// sprawdzić czy jest w Pa czy w hPa
  msg += ",";
  msg += String(data.bmp.altitude, 2);
  msg += ",";
  msg += String(data.bmp.lastVerticalSpeed, 4);
  msg += ",";

  // MAX
  msg += String(data.max.temp, 2);
  msg += ",";

  // BATTERY
  msg += String(data.battery.voltage, 1);
  return msg;
}

void Rakieta::sendGpsOffset()
{
  String msg = "GSP offset:Now:";

  msg += String(millis());
  msg += ",lat:";
  msg += offsets.gps.lat;
  msg += ",lng:";
  msg += offsets.gps.lng;

  transmit(msg);  
}

void Rakieta::setOffsets()
{
  debugln(F("Sensors calibration. Wait..."));

  validLsm = 0, validAdxl = 0, validBmp = 0, validMax = 0, validGPS = 0;

  uint8_t num = 50;
  for (uint8_t i = 0; i < num; i++)
  {
    sensors_event_t accel, gyro, temp;
    if (lsm.getEvent(&accel, &gyro, &temp))
    {
      offsets.lsm.ax += accel.acceleration.x;
      offsets.lsm.ay += accel.acceleration.y;
      offsets.lsm.az += accel.acceleration.z;
      offsets.lsm.gx += gyro.gyro.x;
      offsets.lsm.gy += gyro.gyro.y;
      offsets.lsm.gz += gyro.gyro.z;
      validLsm++;
    }

    sensors_event_t event;
    if (adxl.getEvent(&event))
    {
      offsets.adxl.ax += event.acceleration.x * ADXL375_MG2G_MULTIPLIER;
      offsets.adxl.ay += event.acceleration.y * ADXL375_MG2G_MULTIPLIER;
      offsets.adxl.az += event.acceleration.z * ADXL375_MG2G_MULTIPLIER;
      validAdxl++;
    }

    if (bmp.performReading())
    {
      offsets.bmp.altitude += bmp.readAltitude(SEA_LEVEL_PRESSURE_HPA);
      validBmp++;
    }

    delay(25);
  }

  offsets.lsm.ax = (validLsm ? ((float)offsets.lsm.ax / (float)validLsm) : 0);
  offsets.lsm.ay = (validLsm ? ((float)offsets.lsm.ay / (float)validLsm) : 0);
  offsets.lsm.az = (validLsm ? ((float)offsets.lsm.az / (float)validLsm) : 0);
  offsets.lsm.gx = (validLsm ? ((float)offsets.lsm.gx / (float)validLsm) : 0);
  offsets.lsm.gy = (validLsm ? ((float)offsets.lsm.gy / (float)validLsm) : 0);
  offsets.lsm.gz = (validLsm ? ((float)offsets.lsm.gz / (float)validLsm) : 0);
  offsets.adxl.ax = (validAdxl ? ((float)offsets.adxl.ax / (float)validAdxl) : 0);
  offsets.adxl.ay = (validAdxl ? ((float)offsets.adxl.ay / (float)validAdxl) : 0);
  offsets.adxl.az = (validAdxl ? ((float)offsets.adxl.az / (float)validAdxl) : 0);
  offsets.bmp.altitude = (validBmp ? ((float)offsets.bmp.altitude / (float)validBmp) : 0);

  num = 10;
  uint32_t timeout = millis() + 30000;

  while (validGPS < num || millis() < timeout)
  {
    if (gps.location.isValid() && gps.altitude.isValid())
    {
      offsets.gps.lat += gps.location.lat();
      offsets.gps.lng += gps.location.lng();
      offsets.gps.altiM += gps.altitude.meters();
      offsets.gps.altiF += gps.altitude.feet();
      offsets.gps.speed += gps.speed.mps();
      validGPS++;
    }
    delay(200);
  }

  /// czy odejmować tylko całości czy całkowitą wartość (teraz są całości)
  offsets.gps.lat = int32_t(validGPS ? ((float)offsets.gps.lat / (float)validGPS) : 0);
  offsets.gps.lng = int32_t(validGPS ? ((float)offsets.gps.lng / (float)validGPS) : 0);
  offsets.gps.altiM = (validGPS ? ((float)offsets.gps.altiM / (float)validGPS) : 0);
  offsets.gps.altiF = (validGPS ? ((float)offsets.gps.altiF / (float)validGPS) : 0);
  offsets.gps.speed = (validGPS ? ((float)offsets.gps.speed / (float)validGPS) : 0);

  String msg = prepareOffsetsMsg();

  debug(msg);
  debugln("Calibration completed!");
}

void Rakieta::handleGps()
{
  while (gpsSerial.available() > 0)
    gps.encode(gpsSerial.read());

  if (gps.location.isValid() && gps.location.isUpdated())
  {
    data.gps.lat = gps.location.lat() - offsets.gps.lat;
    data.gps.lng = gps.location.lng() - offsets.gps.lng;
  }
  if (gps.altitude.isValid() && gps.altitude.isUpdated())
  {
    data.gps.altiM = gps.altitude.meters() - offsets.gps.altiM;
    data.gps.altiF = gps.altitude.feet() - offsets.gps.altiF;

    if (data.gps.maxAltitude < data.gps.altiM)
      data.gps.maxAltitude = data.gps.altiM;
    
    uint32_t now = millis();
    float dt = (now - data.gps.lastTime) / 1000.0f;
    if (dt <= 0.0) dt = 0.001;
    data.gps.lastVerticalSpeed = (data.gps.altiM - data.gps.lastAltitude) / dt;
    data.gps.lastAltitude = data.gps.altiM;
    data.gps.lastTime = now;
  }
  if (gps.time.isValid() && gps.time.isUpdated())
  {
    data.gps.h = gps.time.hour();
    data.gps.m = gps.time.minute();
    data.gps.s = gps.time.second();
    data.gps.centi = gps.time.centisecond();
  }
  if (gps.speed.isValid() && gps.speed.isUpdated())
  { data.gps.speed = gps.speed.mps() - offsets.gps.speed; }
  if (gps.course.isValid() && gps.course.isUpdated())
  { data.gps.course = gps.course.deg(); }
  if (gps.satellites.isValid() && gps.satellites.isUpdated())
  { data.gps.satNum = gps.satellites.value(); }
  if (gps.hdop.isValid() && gps.hdop.isUpdated())
  { data.gps.hdop = gps.hdop.hdop(); }
}

void Rakieta::handleLsm6()
{
  sensors_event_t accel, gyro, temp;
  if (lsm.getEvent(&accel, &gyro, &temp))
  {
    data.lsm.ax = accel.acceleration.x - offsets.lsm.ax;
    data.lsm.ay = accel.acceleration.y - offsets.lsm.ay;
    data.lsm.az = accel.acceleration.z - offsets.lsm.az;
    data.lsm.gx = gyro.gyro.x - offsets.lsm.gx;
    data.lsm.gy = gyro.gyro.y - offsets.lsm.gy;
    data.lsm.gz = gyro.gyro.z - offsets.lsm.gz;
    data.lsm.temp = temp.temperature;

    uint32_t now = millis();
    float dt = (now - data.lsm.lastTime) / 1000.0f;
    data.lsm.lastTotalAccel = sqrt(data.lsm.ax * data.lsm.ax + 
                                   data.lsm.ay * data.lsm.ay + 
                                   data.lsm.az * data.lsm.az);
    data.lsm.lastTotalSpeed += data.lsm.lastTotalAccel * dt;
    data.lsm.lastTotalRotation = data.lsm.gx + data.lsm.gy + data.lsm.gz;
    data.lsm.lastTime = now;

    error &= ~LSM_ERROR;
  }
  else
    error |= LSM_ERROR;
}

void Rakieta::handleAdxl()
{
  sensors_event_t event;
  if (adxl.getEvent(&event))
  {
    data.adxl.ax = (event.acceleration.x * ADXL375_MG2G_MULTIPLIER) - offsets.adxl.ax;
    data.adxl.ay = (event.acceleration.y * ADXL375_MG2G_MULTIPLIER) - offsets.adxl.ay;
    data.adxl.az = (event.acceleration.z * ADXL375_MG2G_MULTIPLIER) - offsets.adxl.az;

    uint32_t now = millis();
    float dt = (now - data.adxl.lastTime) / 1000.0f;
    data.adxl.lastTotalAccel = sqrt(data.adxl.ax * data.adxl.ax + 
                                    data.adxl.ay * data.adxl.ay + 
                                    data.adxl.az * data.adxl.az);
    data.adxl.lastTotalSpeed += data.adxl.lastTotalAccel * dt;
    data.adxl.lastTime = now;

    error &= ~ADXL_ERROR;
  }
  else
    error |= ADXL_ERROR;
}

void Rakieta::handleBmp()
{
  if (bmp.performReading())
  {
    data.bmp.temp = bmp.temperature;
    data.bmp.pressure = bmp.pressure;
    data.bmp.altitude = bmp.readAltitude(SEA_LEVEL_PRESSURE_HPA) - offsets.bmp.altitude;

    if (data.bmp.maxAltitude < data.bmp.altitude)
      data.bmp.maxAltitude = data.bmp.altitude;
    
    uint32_t now = millis();
    float dt = (now - data.bmp.lastTime) / 1000.0f;
    if (dt <= 0.0) dt = 0.001;
    data.bmp.lastVerticalSpeed = (data.bmp.altitude - data.bmp.lastAltitude) / dt;
    data.bmp.lastAltitude = data.bmp.altitude;
    data.bmp.lastTime = now;

    error &= ~BMP_ERROR;
  }
  else
    error |= BMP_ERROR;
}

void Rakieta::handleMax()
{
  float tempC = max3.readCelsius();
  uint8_t e = max3.readError();

  if (isnan(tempC) || e)
  {
    debugln("MAX31855 read error!");
    error |= MAX_ERROR;
    if (e & MAX31855_FAULT_OPEN)
    {
      debugln("FAULT: Thermocouple is open - no connections.");
    }
    if (e & MAX31855_FAULT_SHORT_GND)
    {
      debugln("FAULT: Thermocouple is short-circuited to GND.");
    }
    if (e & MAX31855_FAULT_SHORT_VCC)
    {
      debugln("FAULT: Thermocouple is short-circuited to VCC.");
    }
  }
  else
  {
    data.max.temp = tempC;
    error &= ~MAX_ERROR;
  }
}

void Rakieta::handleBattery()
{
  int rawValue = analogRead(BATTERY);
  data.battery.voltage = (rawValue * 3.3f / 4095.0f) * 2.0f;  // scale to 3.3V and multiply by the divisor ratio
}

void Rakieta::activateSolenoid(uint8_t pulses = SOLENOID_PULSE_COUNT, uint32_t duration = SOLENOID_PULSE)
{
  solenoidActive = true;
  solenoidPulses = pulses * 2; // On and off are 2 states (toggle actions)
  solenoidPulseDuration = duration;
  solenoidStartTime = millis();
  solenoidPulseInterval = duration;
  
  digitalWrite(SOLENOID, HIGH);
  debugln("Solenoid ON - pulse 1");
}

void Rakieta::updateSolenoid()
{
  if (!solenoidActive) return;
  
  uint32_t now = millis();
  uint32_t elapsed = now - solenoidStartTime;
  uint32_t pulseIndex = elapsed / solenoidPulseInterval;
  
  if (pulseIndex < solenoidPulses)
  {
    // Even indices = ON, odd indices = OFF
    bool shouldBeOn = !(pulseIndex & 1);
    bool isOn = digitalRead(SOLENOID);
    
    if (shouldBeOn && !isOn)
    {
      digitalWrite(SOLENOID, HIGH);
      debugf("Solenoid ON - pulse %d\n", (pulseIndex / 2) + 1);
    }
    else if (!shouldBeOn && isOn)
    {
      digitalWrite(SOLENOID, LOW);
      debug("Solenoid OFF\n");
    }
  }
  else
  {
    // End of the sequence
    digitalWrite(SOLENOID, LOW);
    solenoidActive = false;
    debugln("Solenoid sequence completed");
  }
}

void Rakieta::updateBuzzer()
{
  if (!buzzerEnabled)
  {
    digitalWrite(BUZZER, LOW);
    return;
  }

  uint32_t now = millis();
  if (now - lastBuzzerTime >= buzzerInterval)
  {
    buzzerState = !buzzerState;
    lastBuzzerTime = now;
    digitalWrite(BUZZER, buzzerState ? HIGH : LOW);

    #if SERIAL_DEBUG == 1
      if (buzzerState) { debugln("Buzzer ON"); }
      else { debugln("Buzzer OFF"); }
    #endif
  }
}

void Rakieta::parashuteOpen()  /// trzeba dokończyć (CHYBA JUŻ JEST OK)
{
  debugln("Opening parachute!");
  activateSolenoid(SOLENOID_PULSE_COUNT, SOLENOID_PULSE);
}

void Rakieta::updateStatus()
{
  uint32_t now = millis();

  switch (status)
  {
    case State::debug:
      break;
    case State::idle:
      break;
    case State::ready:
      break;
    case State::burn:
    {
      // przyśpieszenie mniejsze niż 5 m/s^2 przez minimum 0,3s ? -> przechodzi na rising
      // acceleration less than 5 m/s^2 for at least 0.3s ? -> goes to rising
      if (max(data.lsm.lastTotalAccel, data.adxl.lastTotalAccel) < BURN_ACCEL_THRESHOLD)
      {
        if (afterburnStartTime == 0) { afterburnStartTime = now; }
        else if (now - afterburnStartTime >= BURN_ACCEL_CHECK_TIME)
        {
          status = State::rising;
          handleSensorsInterval = SEND_INTERVAL_RISING / 4;
          dataSaveInterval = SEND_INTERVAL_RISING / 4;
          msgSendInterval = SEND_INTERVAL_RISING;
          buzzerInterval = BUZZER_INTERVAL_RISING;
        }
      }
      else
      {
        afterburnStartTime = 0;
      }
      break;
    }
    case State::rising:
    {
      // obecna alti < max alti, vertical speed < 0, po upływie czasu -> przechodzi na apogee
      // current alti < max alti, vertical speed < 0, after time -> goes to apogee
      if (data.gps.lastAltitude <= data.gps.maxAltitude || data.bmp.lastAltitude <= data.bmp.maxAltitude)
      {
        uint8_t neg = 0;
        if (data.gps.lastVerticalSpeed <= 0) neg++;
        if (data.bmp.lastVerticalSpeed <= 0) neg++;
        if (data.lsm.lastTotalSpeed <= 0) neg++;
        if (data.adxl.lastTotalSpeed <= 0) neg++;
        if (neg >= 2)
        {
          if (afterRisingStartTime == 0)
            afterRisingStartTime = now;
        }
      }
      else
      {
        afterRisingStartTime = 0;
      }

      if (afterRisingStartTime > 0 && (now - afterRisingStartTime >= RISING_ACCEL_CHECK_TIME))
      {
        status = State::apogee;
      }
      break;
    }
    case State::apogee:
    {
      // vertical speed < thresh, after max time -> goes to falling
      uint8_t neg = 0;
      if (data.gps.lastVerticalSpeed <= MAX_FREEFALLING_SPEED) neg++;
      if (data.bmp.lastVerticalSpeed <= MAX_FREEFALLING_SPEED) neg++;
      if (data.lsm.lastTotalSpeed <= MAX_FREEFALLING_SPEED) neg++;
      if (data.adxl.lastTotalSpeed <= MAX_FREEFALLING_SPEED) neg++;
      if (neg >= 2)
      {
        if (apogeeStartTime == 0)
          apogeeStartTime = now;
      }
      else
      {
        apogeeStartTime = 0;
      }

      if (apogeeStartTime > 0 && (now - apogeeStartTime >= APOGEE_CHECK_TIME))
      {
        status = State::falling;
        handleSensorsInterval = SEND_INTERVAL_FALLING / 4;
        dataSaveInterval = SEND_INTERVAL_FALLING / 4;
        msgSendInterval = SEND_INTERVAL_FALLING;
        buzzerInterval = BUZZER_INTERVAL_FALLING;
        parashuteOpen();
      }
      break;
    }
    case State::falling:
    {
      if (status == State::falling && now - startTime < MIN_PARACHUTE_TIME) return;
        
      // przyśpieszenie = 0, żyro = 0, alti się nie zmienia, vertical speed = 0 -> przechodzi na touchdown
      // acceleration = 0, gyro = 0, alt speed does not change, vertical speed = 0 -> goes to touchdown
      if (abs(data.gps.lastVerticalSpeed) < 0.1 && abs(data.lsm.lastTotalAccel) < 0.1 &&
          abs(data.lsm.lastTotalRotation) < 0.1 && abs(data.lsm.lastTotalSpeed) < 0.1 &&
          abs(data.adxl.lastTotalSpeed) < 0.1 && abs(data.adxl.lastTotalAccel) < 0.1 &&
          abs(data.bmp.lastVerticalSpeed) < 0.1)
      {
        if (touchdownStartTime == 0)
          touchdownStartTime = now;
      }
      else
      {
        touchdownStartTime = 0;
      }

      if (touchdownStartTime > 0 && (now - touchdownStartTime >= TOUCHDOWN_CHECK_TIME))
      {
        status = State::touchdown;
        handleSensorsInterval = SEND_INTERVAL_TOUCHDOWN;
        dataSaveInterval = SEND_INTERVAL_TOUCHDOWN;
        msgSendInterval = SEND_INTERVAL_TOUCHDOWN;
        buzzerInterval = BUZZER_INTERVAL_TOUCHDOWN;

        String msg = "Start times:Now:";

        msg += String(millis());
        msg += ",startTime:";
        msg += String(startTime);
        msg += ",afterburnStartTime:";
        msg += String(afterburnStartTime);
        msg += ",afterRisingStartTime:";
        msg += String(afterRisingStartTime);
        msg += ",apogeeStartTime:";
        msg += String(apogeeStartTime);
        msg += ",touchdownStartTime:";
        msg += String(touchdownStartTime);

        flashWriteData(msg);
        SDWriteData(msg);
        transmit(msg);
      }
      break;
    }
    case State::touchdown:
      break;
    default:
      break;
  }

  if (now - startTime > MAX_RISING_TIME && (status == State::burn || status == State::rising || status == State::apogee))
  {
    status = State::falling;
    handleSensorsInterval = SEND_INTERVAL_FALLING / 4;
    dataSaveInterval = SEND_INTERVAL_FALLING / 4;
    msgSendInterval = SEND_INTERVAL_FALLING;
    buzzerInterval = BUZZER_INTERVAL_FALLING;
    parashuteOpen();
  }
}

void Rakieta::emergencyStop()
{
  bool emergency = false;
  String log = "";

  if (data.max.temp > 500.0f)  // 500°C emergency!
  {
    log += String(millis()) + "EMERGENCY: Engine fire detected! ";
    emergency = true;
  }

  if (data.adxl.lastTotalAccel > 100.0f)  // >100g emergency!
  {
    log += String(millis()) + "EMERGENCY: Crash/explosion detected! ";
    emergency = true;
  }

  if (emergency)
  {
    log += "EXECUTING EMERGENCY STOP!";
    debugln(log);
    flashWriteData(log);
    SDWriteData(log);
    transmit(log);

    myServo.write(0);  // Close fuel valve
    parashuteOpen();  // Deploy parachute
    buzzerEnabled = true;
    buzzerInterval = BUZZER_INTERVAL_READY;
  }
}

void Rakieta::setFlightMode(bool flight)
{
  inFlight = flight;
  debug("Flight mode: ");
  debugln(inFlight ? "ACTIVE (no watchdog)" : "INACTIVE");
}


