/***************************************************************************
  This is a library for the LSM9DS0 Accelerometer and magnentometer/compass

  Designed specifically to work with the Adafruit LSM9DS0 Breakouts

  These sensors use I2C to communicate, 2 pins are required to interface.

  Adafruit invests time and resources providing this open source code,
  please support Adafruit andopen-source hardware by purchasing products
  from Adafruit!

  Written by Kevin Townsend for Adafruit Industries.  
  BSD license, all text above must be included in any redistribution
 ***************************************************************************/
#include <Adafruit_LSM9DS0.h>

/***************************************************************************
 CONSTRUCTOR
 ***************************************************************************/
// default
Adafruit_LSM9DS0::Adafruit_LSM9DS0( int32_t sensorID ) {
  _i2c = true;
  _lsm9dso_sensorid_accel = sensorID + 1;
  _lsm9dso_sensorid_mag = sensorID + 2;
  _lsm9dso_sensorid_gyro = sensorID + 3;
  _lsm9dso_sensorid_temp = sensorID + 4;
}

Adafruit_LSM9DS0::Adafruit_LSM9DS0(int8_t xmcs, int8_t gcs, int32_t sensorID ) {
  _i2c = false;
  // hardware SPI!
  _csg = gcs;
  _csxm = xmcs;
  _mosi = _miso = _clk = -1;
  _lsm9dso_sensorid_accel = sensorID + 1;
  _lsm9dso_sensorid_mag = sensorID + 2;
  _lsm9dso_sensorid_gyro = sensorID + 3;
  _lsm9dso_sensorid_temp = sensorID + 4;
}

Adafruit_LSM9DS0::Adafruit_LSM9DS0(int8_t clk, int8_t miso, int8_t mosi, int8_t xmcs, int8_t gcs, int32_t sensorID ) {
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
}

bool Adafruit_LSM9DS0::begin()
{
  if (_i2c) {
    Wire.begin();
  } else if (_clk == -1) {
    // Hardware SPI
    pinMode(_csxm, OUTPUT);
    pinMode(_csg, OUTPUT);
    digitalWrite(_csxm, HIGH);
    digitalWrite(_csg, HIGH);
    SPCRback = SPCR;
    SPI.begin();
    SPI.setClockDivider(SPI_CLOCK_DIV8);
    SPI.setDataMode(SPI_MODE0);
    mySPCR = SPCR;
    SPCR = SPCRback;
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
  // Serial.print ("XM whoami: 0x");
  // Serial.println(id, HEX);
  if (id != LSM9DS0_XM_ID) 
    return false;

  id = read8(GYROTYPE, LSM9DS0_REGISTER_WHO_AM_I_G);
  // Serial.print ("G whoami: 0x");
  // Serial.println(id, HEX);
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
  write8(XMTYPE, LSM9DS0_REGISTER_CTRL_REG5_XM, tempReg | (1<<7));
  
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
void Adafruit_LSM9DS0::read()
{
  if (_i2c) {
    // Read the accelerometer
    Wire.beginTransmission((byte)LSM9DS0_ADDRESS_ACCELMAG);
    Wire.write(0x80 | LSM9DS0_REGISTER_OUT_X_L_A);
    Wire.endTransmission();
    Wire.requestFrom((byte)LSM9DS0_ADDRESS_ACCELMAG, (byte)6);
    
    // Wait around until enough data is available
    while (Wire.available() < 6);
    
    uint8_t xlo = Wire.read();
    uint8_t xhi = Wire.read();
    uint8_t ylo = Wire.read();
    uint8_t yhi = Wire.read();
    uint8_t zlo = Wire.read();
    uint8_t zhi = Wire.read();
    
    // Shift values to create properly formed integer (low byte first)
    accelData.x = (xlo | (xhi << 8)) >> 4;
    accelData.y = (ylo | (yhi << 8)) >> 4;
    accelData.z = (zlo | (zhi << 8)) >> 4;
    
    // Read the magnetometer
    Wire.beginTransmission((byte)LSM9DS0_ADDRESS_ACCELMAG);
    Wire.write(0x80 | LSM9DS0_REGISTER_OUT_X_L_M);
    Wire.endTransmission();
    Wire.requestFrom((byte)LSM9DS0_ADDRESS_ACCELMAG, (byte)6);
    
    // Wait around until enough data is available
    while (Wire.available() < 6);
    
    // Note high before low (different than accel)  
    xlo = Wire.read();
    xhi = Wire.read();
    ylo = Wire.read();
    yhi = Wire.read();
    zlo = Wire.read();
    zhi = Wire.read();
    
    // Shift values to create properly formed integer (low byte first)
    magData.x = (xlo | (xhi << 8));
    magData.y = (ylo | (yhi << 8));
    magData.z = (zlo | (zhi << 8));  

    // Read gyro
    Wire.beginTransmission((byte)LSM9DS0_ADDRESS_GYRO);
    Wire.write(0x80 | LSM9DS0_REGISTER_OUT_X_L_G);
    Wire.endTransmission();
    Wire.requestFrom((byte)LSM9DS0_ADDRESS_GYRO, (byte)6);
    
    // Wait around until enough data is available
    while (Wire.available() < 6);
    
    xlo = Wire.read();
    xhi = Wire.read();
    ylo = Wire.read();
    yhi = Wire.read();
    zlo = Wire.read();
    zhi = Wire.read();
    gyroData.x = (xlo | (xhi << 8));
    gyroData.y = (ylo | (yhi << 8));
    gyroData.z = (zlo | (zhi << 8));
    
    // Read temp sensor
    Wire.beginTransmission((byte)LSM9DS0_ADDRESS_ACCELMAG);
    Wire.write(0x80 | LSM9DS0_REGISTER_TEMP_OUT_L_XM);
    Wire.endTransmission();
    Wire.requestFrom((byte)LSM9DS0_ADDRESS_ACCELMAG, (byte)2);

    // Wait around until enough data is available
    while (Wire.available() < 2);
    
    xlo = Wire.read();
    xhi = Wire.read();
    
    // Shift values to create properly formed integer (low byte first)
    temperature = (xlo | (xhi << 8));
  }

  // ToDo: Move to Unified object, but for now convert to SI direct
  accelData.x *= _accel_mg_lsb;
  accelData.y *= _accel_mg_lsb;
  accelData.z *= _accel_mg_lsb;
  magData.x *= _mag_mgauss_lsb;
  magData.y *= _mag_mgauss_lsb;
  magData.z *= _mag_mgauss_lsb;
  gyroData.x *= _gyro_dps_digit;
  gyroData.y *= _gyro_dps_digit;
  gyroData.z *= _gyro_dps_digit;
  temperature /= LSM9DS0_TEMP_LSB_DEGREE_CELSIUS;

  // ToDo: Calculate orientation
}

void Adafruit_LSM9DS0::setupAccel ( lsm9ds0AccelRange_t range )
{
  uint8_t reg = read8(XMTYPE, LSM9DS0_REGISTER_CTRL_REG2_XM);
  reg &= ~(0b00111000);
  reg |= range;
  write8(XMTYPE, LSM9DS0_REGISTER_CTRL_REG2_XM, reg );
  
  switch (range)
  {
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

void Adafruit_LSM9DS0::setupMag ( lsm9ds0MagGain_t gain )
{
  uint8_t reg = read8(XMTYPE, LSM9DS0_REGISTER_CTRL_REG6_XM);
  reg &= ~(0b01100000);
  reg |= gain;
  write8(XMTYPE, LSM9DS0_REGISTER_CTRL_REG6_XM, reg );

  switch(gain)
  {
    case LSM9DS0_MAGGAIN_2GAUSS:
      _mag_mgauss_lsb = LSM9DS0_MAG_MGAUSS_LSB_2GAUSS;
      break;
    case LSM9DS0_MAGGAIN_4GAUSS:
      _mag_mgauss_lsb = LSM9DS0_MAG_MGAUSS_LSB_4GAUSS;
      break;
    case LSM9DS0_MAGGAIN_8GAUSS:
      _mag_mgauss_lsb = LSM9DS0_MAG_MGAUSS_LSB_8GAUSS;
      break;
    case LSM9DS0_MAGGAIN_12GAUSS:
      _mag_mgauss_lsb = LSM9DS0_MAG_MGAUSS_LSB_12GAUSS;
      break;
  }
}

void Adafruit_LSM9DS0::setupGyro ( lsm9ds0GyroScale_t scale )
{
  uint8_t reg = read8(GYROTYPE, LSM9DS0_REGISTER_CTRL_REG4_G);
  reg &= ~(0b00110000);
  reg |= scale;
  write8(GYROTYPE, LSM9DS0_REGISTER_CTRL_REG4_G, reg );

  switch(scale)
  {
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
    @brief  Gets the most recent accel sensor event
*/
/**************************************************************************/
void Adafruit_LSM9DS0::getEvent(sensors_event_t *accelEvent,
                                sensors_event_t *magEvent,
                                sensors_event_t *gyroEvent,
                                sensors_event_t *tempEvent )
{
  /* Clear the events */
  memset(accelEvent, 0, sizeof(sensors_event_t));
  memset(magEvent, 0, sizeof(sensors_event_t));
  memset(gyroEvent, 0, sizeof(sensors_event_t));
  memset(tempEvent, 0, sizeof(sensors_event_t));

  /* update the sensor data */
  read();
  uint32_t timestamp = millis();

  /* Update the accelerometer data */  
  accelEvent->version   = sizeof(sensors_event_t);
  accelEvent->sensor_id = _lsm9dso_sensorid_accel;
  accelEvent->type      = SENSOR_TYPE_ACCELEROMETER;
  accelEvent->timestamp = timestamp;
  accelEvent->acceleration.x = accelData.x;
  accelEvent->acceleration.y = accelData.y;
  accelEvent->acceleration.z = accelData.z;
  
  /* Update the magnetometer data */  
  magEvent->version   = sizeof(sensors_event_t);
  magEvent->sensor_id = _lsm9dso_sensorid_mag;
  magEvent->type      = SENSOR_TYPE_MAGNETIC_FIELD;
  magEvent->timestamp = timestamp;
  magEvent->magnetic.x = magData.x;
  magEvent->magnetic.y = magData.y;
  magEvent->magnetic.z = magData.z;
  
  /* Update the gyroscope data */  
  gyroEvent->version   = sizeof(sensors_event_t);
  gyroEvent->sensor_id = _lsm9dso_sensorid_accel;
  gyroEvent->type      = SENSOR_TYPE_GYROSCOPE;
  gyroEvent->timestamp = timestamp;
  gyroEvent->gyro.x = gyroData.x;
  gyroEvent->gyro.y = gyroData.y;
  gyroEvent->gyro.z = gyroData.z;
  
  /* Update the temperature data */
  tempEvent->version   = sizeof(sensors_event_t);
  tempEvent->sensor_id = _lsm9dso_sensorid_temp;
  tempEvent->type      = SENSOR_TYPE_AMBIENT_TEMPERATURE;
  tempEvent->timestamp = timestamp;
  tempEvent->temperature = temperature;
}

/**************************************************************************/
/*!
    @brief  Gets the sensor_t data
*/
/**************************************************************************/
void Adafruit_LSM9DS0::getSensor(sensor_t *accel, sensor_t *mag,
                                 sensor_t *gyro, sensor_t *temp )
{
  /* Clear the sensor_t objects */
  memset(accel, 0, sizeof(sensor_t));
  memset(mag, 0, sizeof(sensor_t));
  memset(gyro, 0, sizeof(sensor_t));
  memset(temp, 0, sizeof(sensor_t));

  /* Insert the sensor name in the fixed length char array */
  strncpy (accel->name, "LSM9DS0_A", sizeof(accel->name) - 1);
  accel->name[sizeof(accel->name)- 1] = 0;
  accel->version     = 1;
  accel->sensor_id   = _lsm9dso_sensorid_accel;
  accel->type        = SENSOR_TYPE_ACCELEROMETER;
  accel->min_delay   = 0;
  accel->max_value   = 0.0;  // ToDo
  accel->min_value   = 0.0;  // ToDo
  accel->resolution  = 0.0;  // ToDo

  /* Insert the sensor name in the fixed length char array */
  strncpy (mag->name, "LSM9DS0_M", sizeof(mag->name) - 1);
  mag->name[sizeof(mag->name)- 1] = 0;
  mag->version     = 1;
  mag->sensor_id   = _lsm9dso_sensorid_mag;
  mag->type        = SENSOR_TYPE_MAGNETIC_FIELD;
  mag->min_delay   = 0;
  mag->max_value   = 0.0;  // ToDo
  mag->min_value   = 0.0;  // ToDo
  mag->resolution  = 0.0;  // ToDo

  /* Insert the sensor name in the fixed length char array */
  strncpy (gyro->name, "LSM9DS0_G", sizeof(gyro->name) - 1);
  gyro->name[sizeof(gyro->name)- 1] = 0;
  gyro->version     = 1;
  gyro->sensor_id   = _lsm9dso_sensorid_gyro;
  gyro->type        = SENSOR_TYPE_GYROSCOPE;
  gyro->min_delay   = 0;
  gyro->max_value   = 0.0;  // ToDo
  gyro->min_value   = 0.0;  // ToDo
  gyro->resolution  = 0.0;  // ToDo

  /* Insert the sensor name in the fixed length char array */
  strncpy (temp->name, "LSM9DS0_T", sizeof(temp->name) - 1);
  temp->name[sizeof(temp->name)- 1] = 0;
  temp->version     = 1;
  temp->sensor_id   = _lsm9dso_sensorid_temp;
  temp->type        = SENSOR_TYPE_AMBIENT_TEMPERATURE;
  temp->min_delay   = 0;
  temp->max_value   = 0.0;  // ToDo
  temp->min_value   = 0.0;  // ToDo
  temp->resolution  = 0.0;  // ToDo
}

/***************************************************************************
 PRIVATE FUNCTIONS
 ***************************************************************************/
void Adafruit_LSM9DS0::write8(boolean type, byte reg, byte value)
{
  byte address, _cs;

  if (type == GYROTYPE) {
    address = LSM9DS0_ADDRESS_GYRO;
    _cs = _csg;
  } else {
    address = LSM9DS0_ADDRESS_ACCELMAG;
    _cs = _csxm;
  }
  if (_i2c) {
    Wire.beginTransmission(address);
    Wire.write(reg);
    Wire.write(value);
    Wire.endTransmission();
  } else {
    if (_clk == -1) {
      SPCRback = SPCR;
      SPCR = mySPCR;
    }
    digitalWrite(_cs, LOW);
    // set address
    spixfer(reg | 0x40); // write multiple
    spixfer(value); 
    digitalWrite(_cs, HIGH);
    if (_clk == -1) {
      SPCR = SPCRback;
    }
  } 
}

byte Adafruit_LSM9DS0::read8(boolean type, byte reg)
{
  byte address, _cs;
  byte value;

  if (type == GYROTYPE) {
    address = LSM9DS0_ADDRESS_GYRO;
    _cs = _csg;
  } else {
    address = LSM9DS0_ADDRESS_ACCELMAG;
    _cs = _csxm;
  }
  if (_i2c) {
    Wire.beginTransmission(address);
    Wire.write(reg);
    Wire.endTransmission();
    Wire.requestFrom(address, (byte)1);
    value = Wire.read();
    Wire.endTransmission();
  } else {
    if (_clk == -1) {
      SPCRback = SPCR;
      SPCR = mySPCR;
    }
    digitalWrite(_cs, LOW);
    // set address
    spixfer(reg | 0x80 | 0x40); // read multiple
    value = spixfer(0); 
    digitalWrite(_cs, HIGH);
    if (_clk == -1) {
      SPCR = SPCRback;
    }
  }

  return value;
}

uint8_t Adafruit_LSM9DS0::spixfer(uint8_t data) {
  if (_clk == -1) {
    //Serial.println("Hardware SPI");
    return SPI.transfer(data);
  } else {
    //Serial.println("Software SPI");
    uint8_t reply = 0;
    for (int i=7; i>=0; i--) {
      reply <<= 1;
      digitalWrite(_clk, LOW);
      digitalWrite(_mosi, data & (1<<i));
      digitalWrite(_clk, HIGH);
      if (digitalRead(_miso)) 
	reply |= 1;
    }
    return reply;
  }
}
