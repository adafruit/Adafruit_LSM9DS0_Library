/*!
 * @file Adafruit_LSM9DS0.cpp
 *
 * @mainpage Adafruit LSM9DS0 Accelerometer
 *
 * @section intro_sec Introduction
 *
 * This is a library for the LSM9DS0 Accelerometer and magnentometer/compass
 *
 * Designed specifically to work with the Adafruit LSM9DS0 Breakouts
 *
 * These sensors use I2C to communicate, 2 pins are required to interface.
 *
 * Adafruit invests time and resources providing this open source code,
 * please support Adafruit andopen-source hardware by purchasing products
 * from Adafruit!
 *
 * @section author Author
 *
 * Written by Kevin Townsend for Adafruit Industries.
 *
 * @section license License
 *
 * BSD license, all text above must be included in any redistribution
 *
 */
#include <Adafruit_LSM9DS0.h>

/***************************************************************************
 CONSTRUCTOR
 ***************************************************************************/

void Adafruit_LSM9DS0::initI2C(TwoWire *wireBus, int32_t sensorID) {
  _i2c = true;
  _wire = wireBus;
  _lsm9dso_sensorid_accel = sensorID + 1;
  _lsm9dso_sensorid_mag = sensorID + 2;
  _lsm9dso_sensorid_gyro = sensorID + 3;
  _lsm9dso_sensorid_temp = sensorID + 4;
  _accelSensor = Sensor(this, &Adafruit_LSM9DS0::readAccel,
                        &Adafruit_LSM9DS0::getAccelEvent,
                        &Adafruit_LSM9DS0::getAccelSensor);
  _magSensor =
      Sensor(this, &Adafruit_LSM9DS0::readMag, &Adafruit_LSM9DS0::getMagEvent,
             &Adafruit_LSM9DS0::getMagSensor);
  _gyroSensor =
      Sensor(this, &Adafruit_LSM9DS0::readGyro, &Adafruit_LSM9DS0::getGyroEvent,
             &Adafruit_LSM9DS0::getGyroSensor);
  _tempSensor =
      Sensor(this, &Adafruit_LSM9DS0::readTemp, &Adafruit_LSM9DS0::getTempEvent,
             &Adafruit_LSM9DS0::getTempSensor);
}

/**************************************************************************/
/*!
    @brief Instantiate with default hardware I2C interface
    @param sensorID Unique identifier you'd like for the sensors
*/
/**************************************************************************/
Adafruit_LSM9DS0::Adafruit_LSM9DS0(int32_t sensorID) {
  initI2C(&Wire, sensorID);
}

/**************************************************************************/
/*!
    @brief Instantiate with hardware I2C interface
    @param wireBus The I2C wire interface you'd like to use
    @param sensorID Unique identifier you'd like for the sensors
*/
/**************************************************************************/
Adafruit_LSM9DS0::Adafruit_LSM9DS0(TwoWire *wireBus, int32_t sensorID) {
  initI2C(wireBus, sensorID);
}

/**************************************************************************/
/*!
    @brief Instantiate with hardware SPI interface
    @param xmcs SPI CS pin for accelerometer/mag subchip
    @param gcs SPI CS pin for gyro subchip
    @param sensorID Unique identifier you'd like for the sensors
*/
/**************************************************************************/
Adafruit_LSM9DS0::Adafruit_LSM9DS0(int8_t xmcs, int8_t gcs, int32_t sensorID) {
  _i2c = false;
  // hardware SPI!
  _csg = gcs;
  _csxm = xmcs;
  _mosi = _miso = _clk = -1;
  _lsm9dso_sensorid_accel = sensorID + 1;
  _lsm9dso_sensorid_mag = sensorID + 2;
  _lsm9dso_sensorid_gyro = sensorID + 3;
  _lsm9dso_sensorid_temp = sensorID + 4;
  _accelSensor = Sensor(this, &Adafruit_LSM9DS0::readAccel,
                        &Adafruit_LSM9DS0::getAccelEvent,
                        &Adafruit_LSM9DS0::getAccelSensor);
  _magSensor =
      Sensor(this, &Adafruit_LSM9DS0::readMag, &Adafruit_LSM9DS0::getMagEvent,
             &Adafruit_LSM9DS0::getMagSensor);
  _gyroSensor =
      Sensor(this, &Adafruit_LSM9DS0::readGyro, &Adafruit_LSM9DS0::getGyroEvent,
             &Adafruit_LSM9DS0::getGyroSensor);
  _tempSensor =
      Sensor(this, &Adafruit_LSM9DS0::readTemp, &Adafruit_LSM9DS0::getTempEvent,
             &Adafruit_LSM9DS0::getTempSensor);
}

/**************************************************************************/
/*!
    @brief Instantiate with software SPI interface
    @param clk SPI clock pin
    @param miso SPI MISO pin
    @param mosi SPI MOSI pin
    @param xmcs SPI CS pin for accelerometer/mag subchip
    @param gcs SPI CS pin for gyro subchip
    @param sensorID Unique identifier you'd like for the sensors
*/
/**************************************************************************/
Adafruit_LSM9DS0::Adafruit_LSM9DS0(int8_t clk, int8_t miso, int8_t mosi,
                                   int8_t xmcs, int8_t gcs, int32_t sensorID) {
  _i2c = false;
  // software SPI!
  _csg = gcs;
  _csxm = xmcs;
  _mosi = mosi;
  _miso = miso;
  _clk = clk;
  _lsm9dso_sensorid_accel = sensorID + 1;
  _lsm9dso_sensorid_mag = sensorID + 2;
  _lsm9dso_sensorid_gyro = sensorID + 3;
  _lsm9dso_sensorid_temp = sensorID + 4;
  _accelSensor = Sensor(this, &Adafruit_LSM9DS0::readAccel,
                        &Adafruit_LSM9DS0::getAccelEvent,
                        &Adafruit_LSM9DS0::getAccelSensor);
  _magSensor =
      Sensor(this, &Adafruit_LSM9DS0::readMag, &Adafruit_LSM9DS0::getMagEvent,
             &Adafruit_LSM9DS0::getMagSensor);
  _gyroSensor =
      Sensor(this, &Adafruit_LSM9DS0::readGyro, &Adafruit_LSM9DS0::getGyroEvent,
             &Adafruit_LSM9DS0::getGyroSensor);
  _tempSensor =
      Sensor(this, &Adafruit_LSM9DS0::readTemp, &Adafruit_LSM9DS0::getTempEvent,
             &Adafruit_LSM9DS0::getTempSensor);
}

/**************************************************************************/
/*!
    @brief Initialize I2C or SPI and detect/initialize subsensors
    @returns True if both subsensors were detected on the desired interface
*/
/**************************************************************************/
bool Adafruit_LSM9DS0::begin() {
  if (_i2c) {
    _wire->begin();
  } else if (_clk == -1) {
    // Hardware SPI
    pinMode(_csxm, OUTPUT);
    pinMode(_csg, OUTPUT);
    digitalWrite(_csxm, HIGH);
    digitalWrite(_csg, HIGH);
    SPI.begin();
  } else {
    // Sofware SPI
    pinMode(_clk, OUTPUT);
    pinMode(_mosi, OUTPUT);
    pinMode(_csxm, OUTPUT);
    pinMode(_csg, OUTPUT);
    digitalWrite(_csxm, HIGH);
    digitalWrite(_csg, HIGH);
    digitalWrite(_clk, HIGH);
  }

  uint8_t id = read8(XMTYPE, LSM9DS0_REGISTER_WHO_AM_I_XM);
  //  Serial.print ("XM whoami: 0x");
  //   Serial.println(id, HEX);
  if (id != LSM9DS0_XM_ID)
    return false;

  id = read8(GYROTYPE, LSM9DS0_REGISTER_WHO_AM_I_G);
  //   Serial.print ("G whoami: 0x");
  //   Serial.println(id, HEX);
  if (id != LSM9DS0_G_ID)
    return false;

  // Enable the accelerometer continous
  write8(XMTYPE, LSM9DS0_REGISTER_CTRL_REG1_XM, 0x67); // 100hz XYZ
  write8(XMTYPE, LSM9DS0_REGISTER_CTRL_REG5_XM, 0b11110000);
  // enable mag continuous
  write8(XMTYPE, LSM9DS0_REGISTER_CTRL_REG7_XM, 0b00000000);
  // enable gyro continuous
  write8(GYROTYPE, LSM9DS0_REGISTER_CTRL_REG1_G, 0x0F); // on XYZ
  // enable the temperature sensor (output rate same as the mag sensor)
  uint8_t tempReg = read8(XMTYPE, LSM9DS0_REGISTER_CTRL_REG5_XM);
  write8(XMTYPE, LSM9DS0_REGISTER_CTRL_REG5_XM, tempReg | (1 << 7));

  /*
  for (uint8_t i=0; i<0x30; i++) {
    Serial.print("$"); Serial.print(i, HEX);
    Serial.print(" = 0x");
    Serial.println(read8(XMTYPE, i), HEX);
  }
  */

  // Set default ranges for the various sensors
  setupAccel(LSM9DS0_ACCELRANGE_2G);
  setupMag(LSM9DS0_MAGGAIN_2GAUSS);
  setupGyro(LSM9DS0_GYROSCALE_245DPS);

  return true;
}

/***************************************************************************
 PUBLIC FUNCTIONS
 ***************************************************************************/

/**************************************************************************/
/*!
    @brief Read all four sensor subcomponents
*/
/**************************************************************************/
void Adafruit_LSM9DS0::read() {
  /* Read all the sensors. */
  readAccel();
  readMag();
  readGyro();
  readTemp();
}

/**************************************************************************/
/*!
    @brief Read the sensor accelerometer sensor component
*/
/**************************************************************************/
void Adafruit_LSM9DS0::readAccel() {
  // Read the accelerometer
  byte buffer[6];
  readBuffer(XMTYPE, 0x80 | LSM9DS0_REGISTER_OUT_X_L_A, 6, buffer);

  uint8_t xlo = buffer[0];
  int16_t xhi = buffer[1];
  uint8_t ylo = buffer[2];
  int16_t yhi = buffer[3];
  uint8_t zlo = buffer[4];
  int16_t zhi = buffer[5];

  // Shift values to create properly formed integer (low byte first)
  xhi <<= 8;
  xhi |= xlo;
  yhi <<= 8;
  yhi |= ylo;
  zhi <<= 8;
  zhi |= zlo;
  accelData.x = xhi;
  accelData.y = yhi;
  accelData.z = zhi;
}

/**************************************************************************/
/*!
    @brief Read the sensor magnetometer sensor component
*/
/**************************************************************************/
void Adafruit_LSM9DS0::readMag() {
  // Read the magnetometer
  byte buffer[6];
  readBuffer(XMTYPE, 0x80 | LSM9DS0_REGISTER_OUT_X_L_M, 6, buffer);

  uint8_t xlo = buffer[0];
  int16_t xhi = buffer[1];
  uint8_t ylo = buffer[2];
  int16_t yhi = buffer[3];
  uint8_t zlo = buffer[4];
  int16_t zhi = buffer[5];

  // Shift values to create properly formed integer (low byte first)
  xhi <<= 8;
  xhi |= xlo;
  yhi <<= 8;
  yhi |= ylo;
  zhi <<= 8;
  zhi |= zlo;
  magData.x = xhi;
  magData.y = yhi;
  magData.z = zhi;
}

/**************************************************************************/
/*!
    @brief Read the sensor gyroscope sensor component
*/
/**************************************************************************/
void Adafruit_LSM9DS0::readGyro() {
  // Read gyro
  byte buffer[6];
  readBuffer(GYROTYPE, 0x80 | LSM9DS0_REGISTER_OUT_X_L_G, 6, buffer);

  uint8_t xlo = buffer[0];
  int16_t xhi = buffer[1];
  uint8_t ylo = buffer[2];
  int16_t yhi = buffer[3];
  uint8_t zlo = buffer[4];
  int16_t zhi = buffer[5];

  // Shift values to create properly formed integer (low byte first)
  xhi <<= 8;
  xhi |= xlo;
  yhi <<= 8;
  yhi |= ylo;
  zhi <<= 8;
  zhi |= zlo;

  gyroData.x = xhi;
  gyroData.y = yhi;
  gyroData.z = zhi;
}

/**************************************************************************/
/*!
    @brief Read the sensor temperature sensor component
*/
/**************************************************************************/
void Adafruit_LSM9DS0::readTemp() {
  // Read temp sensor
  byte buffer[2];
  readBuffer(XMTYPE, 0x80 | LSM9DS0_REGISTER_TEMP_OUT_L_XM, 2, buffer);
  uint8_t xlo = buffer[0];
  int16_t xhi = buffer[1];

  xhi <<= 8;
  xhi |= xlo;

  // Shift values to create properly formed integer (low byte first)
  temperature = xhi;
}

/**************************************************************************/
/*!
    @brief Configure the accelerometer ranging
    @param range Can be LSM9DS0_ACCELRANGE_2G, LSM9DS0_ACCELRANGE_4G,
    LSM9DS0_ACCELRANGE_6G, LSM9DS0_ACCELRANGE_8G, LSM9DS0_ACCELRANGE_16G
*/
/**************************************************************************/
void Adafruit_LSM9DS0::setupAccel(lsm9ds0AccelRange_t range) {
  uint8_t reg = read8(XMTYPE, LSM9DS0_REGISTER_CTRL_REG2_XM);
  reg &= ~(0b00111000);
  reg |= range;
  write8(XMTYPE, LSM9DS0_REGISTER_CTRL_REG2_XM, reg);

  switch (range) {
  case LSM9DS0_ACCELRANGE_2G:
    _accel_mg_lsb = LSM9DS0_ACCEL_MG_LSB_2G;
    break;
  case LSM9DS0_ACCELRANGE_4G:
    _accel_mg_lsb = LSM9DS0_ACCEL_MG_LSB_4G;
    break;
  case LSM9DS0_ACCELRANGE_6G:
    _accel_mg_lsb = LSM9DS0_ACCEL_MG_LSB_6G;
    break;
  case LSM9DS0_ACCELRANGE_8G:
    _accel_mg_lsb = LSM9DS0_ACCEL_MG_LSB_8G;
    break;
  case LSM9DS0_ACCELRANGE_16G:
    _accel_mg_lsb = LSM9DS0_ACCEL_MG_LSB_16G;
    break;
  }
}

/**************************************************************************/
/*!
    @brief Configure the magnetometer gain
    @param gain Can be LSM9DS0_MAGGAIN_2GAUSS, LSM9DS0_MAGGAIN_4GAUSS,
    LSM9DS0_MAGGAIN_8GAUSS, LSM9DS0_MAGGAIN_12GAUSS
*/
/**************************************************************************/
void Adafruit_LSM9DS0::setupMag(lsm9ds0MagGain_t gain) {
  uint8_t reg = read8(XMTYPE, LSM9DS0_REGISTER_CTRL_REG6_XM);
  reg &= ~(0b01100000);
  reg |= gain;
  write8(XMTYPE, LSM9DS0_REGISTER_CTRL_REG6_XM, reg);

  switch (gain) {
  case LSM9DS0_MAGGAIN_2GAUSS:
    _mag_mgauss_lsb = LSM9DS0_MAG_MGAUSS_2GAUSS;
    break;
  case LSM9DS0_MAGGAIN_4GAUSS:
    _mag_mgauss_lsb = LSM9DS0_MAG_MGAUSS_4GAUSS;
    break;
  case LSM9DS0_MAGGAIN_8GAUSS:
    _mag_mgauss_lsb = LSM9DS0_MAG_MGAUSS_8GAUSS;
    break;
  case LSM9DS0_MAGGAIN_12GAUSS:
    _mag_mgauss_lsb = LSM9DS0_MAG_MGAUSS_12GAUSS;
    break;
  }
}

/**************************************************************************/
/*!
    @brief Configure the gyroscope scaling
    @param scale Can be LSM9DS0_GYROSCALE_245DPS, LSM9DS0_GYROSCALE_500DPS
    or LSM9DS0_GYROSCALE_2000DPS
*/
/**************************************************************************/
void Adafruit_LSM9DS0::setupGyro(lsm9ds0GyroScale_t scale) {
  uint8_t reg = read8(GYROTYPE, LSM9DS0_REGISTER_CTRL_REG4_G);
  reg &= ~(0b00110000);
  reg |= scale;
  write8(GYROTYPE, LSM9DS0_REGISTER_CTRL_REG4_G, reg);

  switch (scale) {
  case LSM9DS0_GYROSCALE_245DPS:
    _gyro_dps_digit = LSM9DS0_GYRO_DPS_DIGIT_245DPS;
    break;
  case LSM9DS0_GYROSCALE_500DPS:
    _gyro_dps_digit = LSM9DS0_GYRO_DPS_DIGIT_500DPS;
    break;
  case LSM9DS0_GYROSCALE_2000DPS:
    _gyro_dps_digit = LSM9DS0_GYRO_DPS_DIGIT_2000DPS;
    break;
  }
}

/***************************************************************************
 UNIFIED SENSOR FUNCTIONS
 ***************************************************************************/

/**************************************************************************/
/*!
    @brief  Gets the most recent accel sensor events for all 4 sensors
    @param accelEvent The accelerometer event object we will fill, pass NULL to
   skip
    @param magEvent The magnetometer event object we will fill, pass NULL to
   skip
    @param gyroEvent The gyroscope event object we will fill, pass NULL to skip
    @param tempEvent The temperature event object we will fill, pass NULL to
   skip
    @returns True on successful reads
*/
/**************************************************************************/
bool Adafruit_LSM9DS0::getEvent(sensors_event_t *accelEvent,
                                sensors_event_t *magEvent,
                                sensors_event_t *gyroEvent,
                                sensors_event_t *tempEvent) {
  /* Grab new sensor reading and timestamp. */
  read();
  uint32_t timestamp = millis();

  /* Update appropriate sensor events. */
  if (accelEvent)
    getAccelEvent(accelEvent, timestamp);
  if (magEvent)
    getMagEvent(magEvent, timestamp);
  if (gyroEvent)
    getGyroEvent(gyroEvent, timestamp);
  if (tempEvent)
    getTempEvent(tempEvent, timestamp);

  return true;
}

/**************************************************************************/
/*!
    @brief Gets the sensor_t data for all 4 sub-sensors at once call
    @param accel The accelerometer sensor_t object we will fill, pass NULL to
   skip
    @param mag The magnetometer sensor_t object we will fill, pass NULL to skip
    @param gyro The gyroscope sensor_t object we will fill, pass NULL to skip
    @param temp The temperature sensor_t object we will fill, pass NULL to skip
*/
/**************************************************************************/
void Adafruit_LSM9DS0::getSensor(sensor_t *accel, sensor_t *mag, sensor_t *gyro,
                                 sensor_t *temp) {
  /* Update appropriate sensor metadata. */
  if (accel)
    getAccelSensor(accel);
  if (mag)
    getMagSensor(mag);
  if (gyro)
    getGyroSensor(gyro);
  if (temp)
    getTempSensor(temp);
}

/***************************************************************************
 PRIVATE FUNCTIONS
 ***************************************************************************/
void Adafruit_LSM9DS0::write8(boolean type, byte reg, byte value) {
  byte address, _cs;

  if (type == GYROTYPE) {
    address = LSM9DS0_ADDRESS_GYRO;
    _cs = _csg;
  } else {
    address = LSM9DS0_ADDRESS_ACCELMAG;
    _cs = _csxm;
  }
  if (_i2c) {
    _wire->beginTransmission(address);
    _wire->write(reg);
    _wire->write(value);
    _wire->endTransmission();
  } else {
    SPI.beginTransaction(SPISettings(200000, MSBFIRST, SPI_MODE0));
    digitalWrite(_cs, LOW);
    // set address
    spixfer(reg | 0x40); // write multiple
    spixfer(value);
    digitalWrite(_cs, HIGH);
    SPI.endTransaction();
  }
}

byte Adafruit_LSM9DS0::read8(boolean type, byte reg) {
  uint8_t value;

  readBuffer(type, reg, 1, &value);

  return value;
}

byte Adafruit_LSM9DS0::readBuffer(boolean type, byte reg, byte len,
                                  uint8_t *buffer) {
  byte address, _cs;

  if (type == GYROTYPE) {
    address = LSM9DS0_ADDRESS_GYRO;
    _cs = _csg;
  } else {
    address = LSM9DS0_ADDRESS_ACCELMAG;
    _cs = _csxm;
  }

  if (_i2c) {
#ifdef __SAM3X8E__
    _wire->requestFrom(
        address, len, reg, 1,
        true); // see
               // http://forum.arduino.cc/index.php?topic=385377.msg2947227#msg2947227
#else
    _wire->beginTransmission(address);
    _wire->write(reg);
    _wire->endTransmission();
    _wire->requestFrom(address, (byte)len);
#endif

    for (uint8_t i = 0; i < len; i++) {
      buffer[i] = _wire->read();
    }
  } else {
    SPI.beginTransaction(SPISettings(200000, MSBFIRST, SPI_MODE0));
    digitalWrite(_cs, LOW);
    // set address
    spixfer(reg | 0x80 | 0x40); // read multiple
    for (uint8_t i = 0; i < len; i++) {
      buffer[i] = spixfer(0);
    }
    digitalWrite(_cs, HIGH);
    SPI.endTransaction();
  }

  return len;
}

uint8_t Adafruit_LSM9DS0::spixfer(uint8_t data) {

  if (_clk == -1) {
    // Serial.println("Hardware SPI");
    return SPI.transfer(data);
  } else {
    // Serial.println("Software SPI");
    uint8_t reply = 0;
    for (int i = 7; i >= 0; i--) {
      reply <<= 1;
      digitalWrite(_clk, LOW);
      digitalWrite(_mosi, data & (1 << i));
      digitalWrite(_clk, HIGH);
      if (digitalRead(_miso))
        reply |= 1;
    }
    return reply;
  }
}

/**************************************************************************/
/*!
    @brief Fill in the details about the most recent accelerometer data read
    @param event The sensor_event_t object we will fill!
    @param timestamp Unused
*/
/**************************************************************************/
void Adafruit_LSM9DS0::getAccelEvent(sensors_event_t *event,
                                     uint32_t timestamp) {
  memset(event, 0, sizeof(sensors_event_t));
  event->version = sizeof(sensors_event_t);
  event->sensor_id = _lsm9dso_sensorid_accel;
  event->type = SENSOR_TYPE_ACCELEROMETER;
  event->timestamp = timestamp;
  event->acceleration.x = accelData.x * _accel_mg_lsb;
  event->acceleration.x /= 1000;
  event->acceleration.x *= SENSORS_GRAVITY_STANDARD;
  event->acceleration.y = accelData.y * _accel_mg_lsb;
  event->acceleration.y /= 1000;
  event->acceleration.y *= SENSORS_GRAVITY_STANDARD;
  event->acceleration.z = accelData.z * _accel_mg_lsb;
  event->acceleration.z /= 1000;
  event->acceleration.z *= SENSORS_GRAVITY_STANDARD;
}

/**************************************************************************/
/*!
    @brief Fill in the details about the most recent magnetometer data read
    @param event The sensor_event_t object we will fill!
    @param timestamp Unused
*/
/**************************************************************************/
void Adafruit_LSM9DS0::getMagEvent(sensors_event_t *event, uint32_t timestamp) {
  memset(event, 0, sizeof(sensors_event_t));
  event->version = sizeof(sensors_event_t);
  event->sensor_id = _lsm9dso_sensorid_mag;
  event->type = SENSOR_TYPE_MAGNETIC_FIELD;
  event->timestamp = timestamp;
  event->magnetic.x = magData.x * _mag_mgauss_lsb / 10;
  event->magnetic.y = magData.y * _mag_mgauss_lsb / 10;
  event->magnetic.z = magData.z * _mag_mgauss_lsb / 10;
}

/**************************************************************************/
/*!
    @brief Fill in the details about the most recent gyroscope data read
    @param event The sensor_event_t object we will fill!
    @param timestamp The millis timestamp when the read occured
*/
/**************************************************************************/
void Adafruit_LSM9DS0::getGyroEvent(sensors_event_t *event,
                                    uint32_t timestamp) {
  memset(event, 0, sizeof(sensors_event_t));
  event->version = sizeof(sensors_event_t);
  event->sensor_id = _lsm9dso_sensorid_accel;
  event->type = SENSOR_TYPE_GYROSCOPE;
  event->timestamp = timestamp;
  event->gyro.x = gyroData.x * _gyro_dps_digit * SENSORS_DPS_TO_RADS;
  event->gyro.y = gyroData.y * _gyro_dps_digit * SENSORS_DPS_TO_RADS;
  event->gyro.z = gyroData.z * _gyro_dps_digit * SENSORS_DPS_TO_RADS;
}

/**************************************************************************/
/*!
    @brief Fill in the details about the most recent temperature data read
    @param event The sensor_event_t object we will fill!
    @param timestamp The millis timestamp when the read occured
*/
/**************************************************************************/
void Adafruit_LSM9DS0::getTempEvent(sensors_event_t *event,
                                    uint32_t timestamp) {
  memset(event, 0, sizeof(sensors_event_t));
  event->version = sizeof(sensors_event_t);
  event->sensor_id = _lsm9dso_sensorid_temp;
  event->type = SENSOR_TYPE_AMBIENT_TEMPERATURE;
  event->timestamp = timestamp;
  // This is just a guess since the staring point (21C here) isn't documented :(
  event->temperature = 21.0 + (float)temperature / 8;
  // event->temperature /= LSM9DS0_TEMP_LSB_DEGREE_CELSIUS;
}

/**************************************************************************/
/*!
    @brief Fill in the details about the accelerometer sensor component
    @param sensor The sensor_t object we will fill!
*/
/**************************************************************************/
void Adafruit_LSM9DS0::getAccelSensor(sensor_t *sensor) {
  memset(sensor, 0, sizeof(sensor_t));
  strncpy(sensor->name, "LSM9DS0_A", sizeof(sensor->name) - 1);
  sensor->name[sizeof(sensor->name) - 1] = 0;
  sensor->version = 1;
  sensor->sensor_id = _lsm9dso_sensorid_accel;
  sensor->type = SENSOR_TYPE_ACCELEROMETER;
  sensor->min_delay = 0;
  sensor->max_value = 0.0;  // ToDo
  sensor->min_value = 0.0;  // ToDo
  sensor->resolution = 0.0; // ToDo
}

/**************************************************************************/
/*!
    @brief Fill in the details about the magnetometer sensor component
    @param sensor The sensor_t object we will fill!
*/
/**************************************************************************/
void Adafruit_LSM9DS0::getMagSensor(sensor_t *sensor) {
  memset(sensor, 0, sizeof(sensor_t));
  strncpy(sensor->name, "LSM9DS0_M", sizeof(sensor->name) - 1);
  sensor->name[sizeof(sensor->name) - 1] = 0;
  sensor->version = 1;
  sensor->sensor_id = _lsm9dso_sensorid_mag;
  sensor->type = SENSOR_TYPE_MAGNETIC_FIELD;
  sensor->min_delay = 0;
  sensor->max_value = 0.0;  // ToDo
  sensor->min_value = 0.0;  // ToDo
  sensor->resolution = 0.0; // ToDo
}

/**************************************************************************/
/*!
    @brief Fill in the details about the gyroscope sensor component
    @param sensor The sensor_t object we will fill!
*/
/**************************************************************************/
void Adafruit_LSM9DS0::getGyroSensor(sensor_t *sensor) {
  memset(sensor, 0, sizeof(sensor_t));
  strncpy(sensor->name, "LSM9DS0_G", sizeof(sensor->name) - 1);
  sensor->name[sizeof(sensor->name) - 1] = 0;
  sensor->version = 1;
  sensor->sensor_id = _lsm9dso_sensorid_gyro;
  sensor->type = SENSOR_TYPE_GYROSCOPE;
  sensor->min_delay = 0;
  sensor->max_value = 0.0;  // ToDo
  sensor->min_value = 0.0;  // ToDo
  sensor->resolution = 0.0; // ToDo
}

/**************************************************************************/
/*!
    @brief Fill in the details about the temperature sensor component
    @param sensor The sensor_t object we will fill!
*/
/**************************************************************************/
void Adafruit_LSM9DS0::getTempSensor(sensor_t *sensor) {
  memset(sensor, 0, sizeof(sensor_t));
  strncpy(sensor->name, "LSM9DS0_T", sizeof(sensor->name) - 1);
  sensor->name[sizeof(sensor->name) - 1] = 0;
  sensor->version = 1;
  sensor->sensor_id = _lsm9dso_sensorid_temp;
  sensor->type = SENSOR_TYPE_AMBIENT_TEMPERATURE;
  sensor->min_delay = 0;
  sensor->max_value = 0.0;  // ToDo
  sensor->min_value = 0.0;  // ToDo
  sensor->resolution = 0.0; // ToDo
}
