#include "Rakieta.h"

/***************************************************
TO DO:
 - dodać LoRa (onTx, onRx, onTxTimeout, sendMsg)
 - dodać mnożniki do wartości w funkcji prepareMsg()
 - zrobić parashuteOpen()
 - dodać zmianę parametrów (buzzer, msgDelay i inne) w funkcji updateStatus()

 -- odkomentować servo dla esp32/stm32
 -- zastanowić się czy wychodzić z funkcji handleSensor() jeśli jest błąd na sensorze

 - do zmiany większość STATUS UPGRADE CONST !!!

***************************************************/

Rakieta::Rakieta(): status(State::debug), gpsSerial(3, 4), lsm(), bmp(), adxl((uint8_t)255, &SPI), max3((uint8_t)255, &SPI), flashTransport(FLASH_CS, &SPI), flash(&flashTransport)
{
  error |= LORA_ERROR | LSM_ERROR | BMP_ERROR | ADXL_ERROR | MAX_ERROR |
           SD_ERROR | SD_FILE_ERROR | FLASH_ERROR | FLASH_FILE_ERROR;

  pinMode(BATTERY, INPUT);
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

void Rakieta::init()
{
  debugln("Initializing rocket systems...");

  // Init all sensors
  watchdog();

  if (error == 0) debugln("All systems OK");
  else { debug("Errors present: "); debugBin(error); }

  delay(100);
  setOffsets();
  
  // Move servo to position 0
  setupServo();
  servoDeg(0);
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
  while (fileNumber < 1000) // Maksymalnie 1000 plików
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
  
  /// Nagłówek CSV do zmiany
  flashWriteData("timestamp,packet,status,error,"
              "gps_lat,gps_lng,gps_alt,"
              "lsm_ax,lsm_ay,lsm_az,"
              "bmp_alt,bmp_press,bmp_temp,"
              "max_temp,battery");
  
  debug("File opened: "); debugln(currentFileName.c_str());
  return true;
}

bool Rakieta::flashWriteData(const String& data)
{
  if (!flashReady || !flashDataFile) return false;
  
  flashDataFile.println(data);
  
  // Flush every 10 writes
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

  bool sdInit = false;
  
  #if defined(ARDUINO_ARCH_ESP32)  // ESP32 - użyj SPI z domyślnymi pinami
    if (!sd.begin()) {
  #elif defined(ARDUINO_ARCH_STM32)  // STM32 - często wymaga podania CS pinu
    if (!sd.begin(SD_CS)) {
  #else  // Dla innych platform
    if (!sd.begin(10)) {
  #endif
    
    debugln("Card Mount Failed");
    return false;
  }
  
  if (!sdInit)
  {
    debugln("SD card initialization failed!");
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
  
  /// Nagłówek CSV do zmiany
  SDWriteData("timestamp,packet,status,error,"
              "gps_lat,gps_lng,gps_alt,"
              "lsm_ax,lsm_ay,lsm_az,"
              "bmp_alt,bmp_press,bmp_temp,"
              "max_temp,battery");
  
  debug("File opened: "); debugln(currentFileName.c_str());
  return true;
}

bool Rakieta::SDWriteData(const String& data)
{
  if (!sdReady || !SDDataFile) return false;
  
  SDDataFile.println(data);
  
  // Flush every 10 writes
  static uint16_t SDWriteCount = 0;
  SDWriteCount++;
  if (SDWriteCount >= 10)
  {
    SDDataFile.flush();
    SDWriteCount = 0;
  }
  
  return true;
}

/// trzeba dodać mnożniki
bool Rakieta::writeRocketData()
{
  String dataLine = "";
  
  // Timestamp
  dataLine += String(millis());
  dataLine += ",";
  dataLine += String(packet);
  dataLine += ",";
  dataLine += String(static_cast<int>(status));
  dataLine += ",";
  dataLine += String(error, HEX);
  dataLine += ",";
  
  // GPS
  dataLine += String(data.gps.lat, 6);
  dataLine += ",";
  dataLine += String(data.gps.lng, 6);
  dataLine += ",";
  dataLine += String(data.gps.altiM, 2);
  dataLine += ",";
  dataLine += String(data.gps.h);
  dataLine += ",";
  dataLine += String(data.gps.m);
  dataLine += ",";
  dataLine += String(data.gps.s);
  dataLine += ",";
  dataLine += String(data.gps.centi);
  dataLine += ",";
  dataLine += String(data.gps.speed, 2);
  dataLine += ",";
  dataLine += String(data.gps.course, 0);
  dataLine += ",";
  dataLine += String(data.gps.satNum);
  dataLine += ",";
  dataLine += String(data.gps.hdop);
  dataLine += ",";
  
  // LSM6
  dataLine += String(data.lsm.ax, 4);
  dataLine += ",";
  dataLine += String(data.lsm.ay, 4);
  dataLine += ",";
  dataLine += String(data.lsm.az, 4);
  dataLine += ",";
  dataLine += String(data.lsm.gx, 4);
  dataLine += ",";
  dataLine += String(data.lsm.gy, 4);
  dataLine += ",";
  dataLine += String(data.lsm.gz, 4);
  dataLine += ",";
  dataLine += String(data.lsm.lastTotalSpeed, 4);
  dataLine += ",";
  dataLine += String(data.lsm.temp, 2);
  dataLine += ",";

  // ADXL
  dataLine += String(data.adxl.ax, 4);
  dataLine += ",";
  dataLine += String(data.adxl.ay, 4);
  dataLine += ",";
  dataLine += String(data.adxl.az, 4);
  dataLine += ",";
  
  // BMP
  dataLine += String(data.bmp.temp, 2);
  dataLine += ",";
  dataLine += String(data.bmp.pressure, 2);
  dataLine += ",";
  dataLine += String(data.bmp.altitude, 2);
  dataLine += ",";
  dataLine += String(data.bmp.lastVerticalSpeed, 2);
  dataLine += ",";
  
  // MAX
  dataLine += String(data.max.temp, 2);
  dataLine += ",";
  
  /// do dodania później
  // BATTERY
  // dataLine += ",";
  // dataLine += String(batteryVoltage, 2);
  debugln(dataLine);

  /// przenieść na samą górę ↓
  if ((!sdReady || !SDDataFile) && (!flashReady || !flashDataFile)) return false;
  
  flashWriteData(dataLine);
  SDWriteData(dataLine);
}

void Rakieta::watchdog()
{
  lastWatchdogTime = millis();
  /// odkomentować
  if (error & LORA_ERROR)
  {
    radioConfig();
    // Radio.Rx(0);
    delay(10);
  }

  if (error & LSM_ERROR)
  {
    if (!lsm.begin_SPI(255, &SPI))
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
      // debugf("asr: %d gsr: %d", lsm.accelerationSampleRate(), lsm.gyroscopeSampleRate());
      error &= ~LSM_ERROR;
    }
    delay(10);
  }
  
  if (error & BMP_ERROR)
  {
    if (!bmp.begin_SPI(255, &SPI))
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
  message.add(uint32_t(packet), packetPos, packetLen);
  message.add(uint16_t(error), errorPos, errorLen);
  message.add(uint8_t(status), statusPos, statusLen);

  message.add(uint16_t(data.gps.lat), gpsLatPos, gpsLatLen);
  message.add(uint16_t(data.gps.lng), gpsLngPos, gpsLngLen);
  message.add(uint16_t(data.gps.altiM), gpsAltiPos, gpsAltiLen);
  message.add(uint8_t(data.gps.h), gpsHourPos, gpsHourLen);
  message.add(uint8_t(data.gps.m), gpsMinPos, gpsMinLen);
  message.add(uint8_t(data.gps.s), gpsSecPos, gpsSecLen);
  message.add(uint8_t(data.gps.centi), gpsCentisecPos, gpsCentisecLen);
  message.add(int16_t(data.gps.speed), gpsSpeedPos, gpsSpeedLen, true);
  message.add(uint16_t(data.gps.course), gpsCoursePos, gpsCourseLen);
  message.add(uint8_t(data.gps.satNum), gpsSatNumPos, gpsSatNumLen);
  message.add(uint8_t(data.gps.hdop), gpsHdopPos, gpsHdopLen);
  
  message.add(int16_t(data.lsm.ax), lsmAccelXPos, lsmAccelXLen, true);
  message.add(int16_t(data.lsm.ay), lsmAccelYPos, lsmAccelYLen, true);
  message.add(int16_t(data.lsm.az), lsmAccelZPos, lsmAccelZLen, true);
  message.add(int16_t(data.lsm.gx), lsmGyroXPos, lsmGyroXLen, true);
  message.add(int16_t(data.lsm.gy), lsmGyroYPos, lsmGyroYLen, true);
  message.add(int16_t(data.lsm.gz), lsmGyroZPos, lsmGyroZLen, true);
  message.add(int16_t(data.lsm.lastTotalSpeed), lsmSpeedPos, lsmSpeedLen, true);
  message.add(uint8_t(data.lsm.temp), lsmTempPos, lsmTempLen);
  
  message.add(int16_t(data.adxl.ax), adxlAccelXPos, adxlAccelXLen, true);
  message.add(int16_t(data.adxl.ay), adxlAccelYPos, adxlAccelYLen, true);
  message.add(int16_t(data.adxl.az), adxlAccelZPos, adxlAccelZLen, true);

  message.add(uint8_t(data.bmp.temp), bmpTempPos, bmpTempLen);
  message.add(uint16_t(data.bmp.pressure), bmpPressPos, bmpPressLen);
  message.add(uint16_t(data.bmp.altitude), bmpAltiPos, bmpAltiLen);
  message.add(int16_t(data.bmp.lastVerticalSpeed), bmpSpeedPos, bmpSpeedLen, true);

  message.add(int16_t(data.max.temp), maxTempPos, maxTempLen, true);

  /// dodać baterie
  /// message.add(uint8_t(...), batteryPos, batteryLen);
}

void Rakieta::sendMsg()
{
  packet++;
  prepareMsg();
  /*
    uint8_t *txPacket = message.data();

    debugln("\n\nSending: ");

    for (int i=0; i<ARRAY_SIZE; i++)
      debugHex(txPacket[i]);
    debugln("");

    for (int i=0; i<ARRAY_SIZE; i++)
      for (int j=7; j>=0; --j)
        debug((txPacket[i] >> j) & 1);

    debugln("\nEnd of the message.\n\n");

    Radio.Send((uint8_t*)txPacket, (uint8_t)ARRAY_SIZE);
    if (status == State::debug || status == State::dynamometer)
    {
      ledState = !ledState;
      digitalWrite(LED_1, ledState);
    }
    debugln("Sending DONE");
  */
}

void Rakieta::setOffsets()
{
  debugln(F("Sensors calibration. Wait..."));

  float validLsm = 0, validAdxl = 0, validBmp = 0, validMax = 0;

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
      offsets.adxl.ax += event.acceleration.x;
      offsets.adxl.ay += event.acceleration.y;
      offsets.adxl.az += event.acceleration.z;
      validAdxl++;
    }

    if (bmp.performReading())
    {
      offsets.bmp.altitude += bmp.readAltitude(SEA_LEVEL_PRESSURE_HPA);
      validBmp++;
    }

    delay(25);
  }

  offsets.lsm.ax = (validLsm ? (offsets.lsm.ax / validLsm) : 0);
  offsets.lsm.ay = (validLsm ? (offsets.lsm.ay / validLsm) : 0);
  offsets.lsm.az = (validLsm ? (offsets.lsm.az / validLsm) : 0);
  offsets.lsm.gx = (validLsm ? (offsets.lsm.gx / validLsm) : 0);
  offsets.lsm.gy = (validLsm ? (offsets.lsm.gy / validLsm) : 0);
  offsets.lsm.gz = (validLsm ? (offsets.lsm.gz / validLsm) : 0);
  offsets.adxl.ax = (validAdxl ? (offsets.adxl.ax / validAdxl) : 0);
  offsets.adxl.ay = (validAdxl ? (offsets.adxl.ay / validAdxl) : 0);
  offsets.adxl.az = (validAdxl ? (offsets.adxl.az / validAdxl) : 0);
  offsets.bmp.altitude = (validBmp ? (offsets.bmp.altitude / validBmp) : 0);

  num = 10;
  uint8_t i = 0;
  uint32_t timeout = millis() + 30000;

  while (i < num || millis() < timeout)
  {
    if (gps.location.isValid() && gps.altitude.isValid())
    {
      offsets.gps.lat += gps.location.lat();
      offsets.gps.lng += gps.location.lng();
      offsets.gps.altiM += gps.altitude.meters();
      offsets.gps.altiF += gps.altitude.feet();
      offsets.gps.speed += gps.speed.mps();
      i++;
    }
    delay(200);
  }

  /// czy odejmować tylko całości czy całkowitą wartość
  offsets.gps.lat = int32_t(i ? (offsets.gps.lat / i) : 0);
  offsets.gps.lng = int32_t(i ? (offsets.gps.lng / i) : 0);
  offsets.gps.altiM = (i ? (offsets.gps.altiM / i) : 0);
  offsets.gps.altiF = (i ? (offsets.gps.altiF / i) : 0);
  offsets.gps.speed = (i ? (offsets.gps.speed / i) : 0);

  /// trzeba odkomentować debugf i usunąć to co poniżej
  /*
  debugf("Valid num: Lsm: %d Adxl: %d Bmp: %d GPS: %d\n", validLsm, validAdxl, validBmp, i);
  debugf("Offsets:\n\tLsm: {ax: %.6f ay: %.6f az: %.6f gx: %.6f gy: %.6f gz: %.6f}\n", offsets.lsm.ax, offsets.lsm.ay, offsets.lsm.az, offsets.lsm.gx, offsets.lsm.gy, offsets.lsm.gz);
  debugf("\tAdxl: {ax: %.6f ay: %.6f az: %.6f}\n\tBmp: alti: %.6f\n", offsets.adxl.ax, offsets.adxl.ay, offsets.adxl.az, offsets.bmp.altitude);
  debugf("\tGPS: {lat: %.6f lng: %.6f altiM: %.6f altiF: %.6f speed: %.6f}\n", offsets.gps.lat, offsets.gps.lng, offsets.gps.altiM, offsets.gps.altiF, offsets.gps.speed);
  */

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
    data.adxl.ax = event.acceleration.x - offsets.adxl.ax;
    data.adxl.ay = event.acceleration.y - offsets.adxl.ay;
    data.adxl.az = event.acceleration.z - offsets.adxl.az;

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
    debugln("Błąd odczytu MAX31855!");
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

void Rakieta::parashuteOpen()  /// trzeba dokończyć
{
  debugln("Opening parachute!");
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
    case State::ready:  /// możliwe że zrobię to komendą z LoRa
      // przyśpieszenie powyżej 1 m/s^2 przez minimum 0,15s -> przechodzi na burn
      if (startTime == 0)
        startTime = now;
      break;
    case State::burn:
    {
      // przyśpieszenie mniejsze niż 5 m/s^2 przez minimum 0,3s ? -> przechodzi na rising
      if (max(data.lsm.lastTotalAccel, data.adxl.lastTotalAccel) < BURN_ACCEL_THRESHOLD)
      {
        if (afterburnStartTime == 0)
          afterburnStartTime = now;
        else if (now - afterburnStartTime >= BURN_ACCEL_CHECK_TIME)
          status = State::rising;
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
        status = State::apogee;
      break;
    }
    case State::apogee:
    {
      // vertical speed < thresh, po upływie czasu -> przechodzi na falling
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
        parashuteOpen();
      }
      break;
    }
    case State::falling:
    {
      if (status == State::falling && now - startTime < MIN_PARACHUTE_TIME) return;
        
      // przyśpieszenie = 0, żyro = 0, alti się nie zmienia, vertical speed = 0 -> przechodzi na touchdown
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
    parashuteOpen();
  }
}

void Rakieta::setupServo()
{
  /*
  ledc_timer_config_t timer_conf = {
      .speed_mode = SERVO_PWM_MODE,
      .duty_resolution = SERVO_PWM_RESOLUTION,
      .timer_num = SERVO_PWM_TIMER,
      .freq_hz = SERVO_PWM_FREQ,
      .clk_cfg = LEDC_AUTO_CLK
  };
  ledc_timer_config(&timer_conf);

  ledc_channel_config_t channel_conf = {
      .gpio_num = SERVO,
      .speed_mode = SERVO_PWM_MODE,
      .channel = SERVO_PWM_CHANNEL,
      .intr_type = LEDC_INTR_DISABLE,
      .timer_sel = SERVO_PWM_TIMER,
      .duty = 0,
      .hpoint = 0
  };
  ledc_channel_config(&channel_conf);
  */
}

void Rakieta::servoDeg(uint8_t deg)
{
  /*
  uint32_t pulse_width = map(deg, 0, 180, SERVO_MIN_PULSE_WIDTH, SERVO_MAX_PULSE_WIDTH);
  uint32_t duty = (pulse_width * (1 << SERVO_PWM_RESOLUTION)) / 20000; // 20ms period
  
  ledc_set_duty(SERVO_PWM_MODE, SERVO_PWM_CHANNEL, duty);
  ledc_update_duty(SERVO_PWM_MODE, SERVO_PWM_CHANNEL);
  */
}











