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
Adafruit_LSM9DS0::Adafruit_LSM9DS0() {
  _i2c = true;

}

Adafruit_LSM9DS0::Adafruit_LSM9DS0(int8_t xmcs, int8_t gcs) {
  _i2c = false;
  // hardware SPI!
  _csg = gcs;
  _csxm = xmcs;
  _mosi = _miso = _clk = -1;
}

Adafruit_LSM9DS0::Adafruit_LSM9DS0(int8_t clk, int8_t miso, int8_t mosi, int8_t xmcs, int8_t gcs) {
  _i2c = false;
  // software SPI!
  _csg = gcs;
  _csxm = xmcs;
  _mosi = mosi;
  _miso = miso;
  _clk = clk;
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
  Serial.print ("XM whoami: 0x");
  Serial.println(id, HEX);
  if (id != LSM9DS0_XM_ID) 
    return false;

   id = read8(GYROTYPE, LSM9DS0_REGISTER_WHO_AM_I_G);
  Serial.print ("G whoami: 0x");
  Serial.println(id, HEX);
  if (id != LSM9DS0_G_ID) 
    return false;


  // Enable the accelerometer continous
  write8(XMTYPE, LSM9DS0_REGISTER_CTRL_REG1_XM, 0x67); // 100hz XYZ
  write8(XMTYPE, LSM9DS0_REGISTER_CTRL_REG5_XM, 0b11110000);
  // enable mag continuous
  write8(XMTYPE, LSM9DS0_REGISTER_CTRL_REG7_XM, 0b00000000);
  // enable gyro continuous
  write8(GYROTYPE, LSM9DS0_REGISTER_CTRL_REG1_G, 0x0F); // on XYZ
  
  /*
  for (uint8_t i=0; i<0x30; i++) {
    Serial.print("$"); Serial.print(i, HEX);
    Serial.print(" = 0x"); 
    Serial.println(read8(XMTYPE, i), HEX);
  }
  */

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
    
  }

  // ToDo: Calculate orientation
  magData.orientation = 0.0;
}

void Adafruit_LSM9DS0::setMagGain(lsm9ds0MagGain gain)
{

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
