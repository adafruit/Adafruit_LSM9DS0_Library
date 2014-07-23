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
#ifndef __LSM9DS0_H__
#define __LSM9DS0_H__

#if (ARDUINO >= 100)
 #include "Arduino.h"
#else
 #include "WProgram.h"
#endif
#include "Wire.h"
#include <SPI.h>

#define LSM9DS0_ADDRESS_ACCELMAG       0x1D         // 3B >> 1 = 7bit default
#define LSM9DS0_ADDRESS_GYRO           0x6B         // D6 >> 1 = 7bit default
#define LSM9DS0_XM_ID                  (0b01001001)
#define LSM9DS0_G_ID                   (0b11010100)


#define GYROTYPE true
#define XMTYPE false

class Adafruit_LSM9DS0
{
  public:
  Adafruit_LSM9DS0();
  Adafruit_LSM9DS0(int8_t xmcs, int8_t gcs);
  Adafruit_LSM9DS0(int8_t clk, int8_t miso, int8_t mosi, int8_t xmcs, int8_t gcs);

   typedef enum
    {         // DEFAULT    TYPE
        LSM9DS0_REGISTER_WHO_AM_I_G = 0x0F,
	LSM9DS0_REGISTER_CTRL_REG1_G = 0x20,
	LSM9DS0_REGISTER_CTRL_REG3_G = 0x22,


	LSM9DS0_REGISTER_OUT_X_L_G           = 0x28,
	LSM9DS0_REGISTER_OUT_X_H_G           = 0x29,
	LSM9DS0_REGISTER_OUT_Y_L_G           = 0x2A,
	LSM9DS0_REGISTER_OUT_Y_H_G           = 0x2B,
	LSM9DS0_REGISTER_OUT_Z_L_G           = 0x2C,
	LSM9DS0_REGISTER_OUT_Z_H_G           = 0x2D,
    } lsm9ds0GyroRegisters_t;

    typedef enum
    {         // DEFAULT    TYPE
      LSM9DS0_REGISTER_TEMP_OUT_L_XM          = 0x05,
      LSM9DS0_REGISTER_TEMP_OUT_H_XM          = 0x06,
      LSM9DS0_REGISTER_STATUS_REG_M = 0x07,
      LSM9DS0_REGISTER_OUT_X_L_M             = 0x08,
      LSM9DS0_REGISTER_OUT_X_H_M             = 0x09,
      LSM9DS0_REGISTER_OUT_Y_L_M             = 0x0A,
      LSM9DS0_REGISTER_OUT_Y_H_M             = 0x0B,
      LSM9DS0_REGISTER_OUT_Z_L_M             = 0x0C,
      LSM9DS0_REGISTER_OUT_Z_H_M             = 0x0D,

      LSM9DS0_REGISTER_WHO_AM_I_XM = 0x0F,

      LSM9DS0_REGISTER_INT_CTRL_REG_M             = 0x12,
      LSM9DS0_REGISTER_INT_SRC_REG_M             = 0x13,

      LSM9DS0_REGISTER_CTRL_REG1_XM = 0x20,
      LSM9DS0_REGISTER_CTRL_REG5_XM = 0x24,
      LSM9DS0_REGISTER_CTRL_REG7_XM = 0x26,

      LSM9DS0_REGISTER_OUT_X_L_A           = 0x28,
      LSM9DS0_REGISTER_OUT_X_H_A           = 0x29,
      LSM9DS0_REGISTER_OUT_Y_L_A           = 0x2A,
      LSM9DS0_REGISTER_OUT_Y_H_A           = 0x2B,
      LSM9DS0_REGISTER_OUT_Z_L_A           = 0x2C,
      LSM9DS0_REGISTER_OUT_Z_H_A           = 0x2D,

      /*
      LSM9DS0_REGISTER_ACCEL_CTRL_REG1_A         = 0x20,   // 00000111   rw
      LSM9DS0_REGISTER_ACCEL_CTRL_REG2_A         = 0x21,   // 00000000   rw
      LSM9DS0_REGISTER_ACCEL_CTRL_REG3_A         = 0x22,   // 00000000   rw
      LSM9DS0_REGISTER_ACCEL_CTRL_REG4_A         = 0x23,   // 00000000   rw
      LSM9DS0_REGISTER_ACCEL_CTRL_REG5_A         = 0x24,   // 00000000   rw
      LSM9DS0_REGISTER_ACCEL_CTRL_REG6_A         = 0x25,   // 00000000   rw
      LSM9DS0_REGISTER_ACCEL_REFERENCE_A         = 0x26,   // 00000000   r
      LSM9DS0_REGISTER_ACCEL_STATUS_REG_A        = 0x27,   // 00000000   r
      LSM9DS0_REGISTER_ACCEL_FIFO_CTRL_REG_A     = 0x2E,
      LSM9DS0_REGISTER_ACCEL_FIFO_SRC_REG_A      = 0x2F,
      LSM9DS0_REGISTER_ACCEL_INT1_CFG_A          = 0x30,
      LSM9DS0_REGISTER_ACCEL_INT1_SOURCE_A       = 0x31,
      LSM9DS0_REGISTER_ACCEL_INT1_THS_A          = 0x32,
      LSM9DS0_REGISTER_ACCEL_INT1_DURATION_A     = 0x33,
      LSM9DS0_REGISTER_ACCEL_INT2_CFG_A          = 0x34,
      LSM9DS0_REGISTER_ACCEL_INT2_SOURCE_A       = 0x35,
      LSM9DS0_REGISTER_ACCEL_INT2_THS_A          = 0x36,
      LSM9DS0_REGISTER_ACCEL_INT2_DURATION_A     = 0x37,
      LSM9DS0_REGISTER_ACCEL_CLICK_CFG_A         = 0x38,
      LSM9DS0_REGISTER_ACCEL_CLICK_SRC_A         = 0x39,
      LSM9DS0_REGISTER_ACCEL_CLICK_THS_A         = 0x3A,
      LSM9DS0_REGISTER_ACCEL_TIME_LIMIT_A        = 0x3B,
      LSM9DS0_REGISTER_ACCEL_TIME_LATENCY_A      = 0x3C,
      LSM9DS0_REGISTER_ACCEL_TIME_WINDOW_A       = 0x3D


	LSM9DS0_REGISTER_MAG_CRB_REG_M             = 0x01,
	LSM9DS0_REGISTER_MAG_MR_REG_M              = 0x02,

	LSM9DS0_REGISTER_MAG_SR_REG_Mg             = 0x09,
	LSM9DS0_REGISTER_MAG_IRA_REG_M             = 0x0A,
	LSM9DS0_REGISTER_MAG_IRB_REG_M             = 0x0B,
	LSM9DS0_REGISTER_MAG_IRC_REG_M             = 0x0C,
*/
    } lsm9ds0MagAccelRegisters_t;

    
    typedef enum
      {
	/*
	LSM9DS0_MAGGAIN_1_3                        = 0x20,  // +/- 1.3
	LSM9DS0_MAGGAIN_1_9                        = 0x40,  // +/- 1.9
	LSM9DS0_MAGGAIN_2_5                        = 0x60,  // +/- 2.5
	LSM9DS0_MAGGAIN_4_0                        = 0x80,  // +/- 4.0
	LSM9DS0_MAGGAIN_4_7                        = 0xA0,  // +/- 4.7
	LSM9DS0_MAGGAIN_5_6                        = 0xC0,  // +/- 5.6
	LSM9DS0_MAGGAIN_8_1                        = 0xE0   // +/- 8.1
	*/
      } lsm9ds0MagGain;	
    
    typedef struct lsm9ds0AccelData_s
    {
      float x;
      float y;
      float z;
    } lsm9ds0AccelData;
    
    typedef struct lsm9ds0GyroData_s
    {
      float x;
      float y;
      float z;
    } lsm9ds0GyroData;
    
    typedef struct lsm9ds0MagData_s
    {
      float x;
      float y;
      float z;
      float orientation;
    } lsm9ds0MagData;
    
    bool begin(void);
    void read(void);
    void setMagGain(lsm9ds0MagGain gain);
    
    lsm9ds0AccelData accelData;    // Last read accelerometer data will be available here
    lsm9ds0MagData magData;        // Last read magnetometer data will be available here
    lsm9ds0GyroData gyroData;      // Last read gyroscope data will be available here
    void write8(boolean type, byte reg, byte value);
    byte read8(boolean type, byte reg);
    uint8_t spixfer(uint8_t data);

  private:
    boolean _i2c;
    int8_t _csg, _csxm, _mosi, _miso, _clk;
    uint8_t mySPCR, SPCRback;
};


#endif
