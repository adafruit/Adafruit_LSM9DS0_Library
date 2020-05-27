/*!
 * @file Adafruit_LSM9DS0.h
 *
 */

#ifndef __LSM9DS0_H__
#define __LSM9DS0_H__

#if (ARDUINO >= 100)
#include "Arduino.h"
#else
#include "WProgram.h"
#endif
#include "Wire.h"
#include <Adafruit_Sensor.h>
#include <SPI.h>

#define LSM9DS0_ADDRESS_ACCELMAG (0x1D) //!< 3B >> 1 = 7bit default
#define LSM9DS0_ADDRESS_GYRO (0x6B)     //!< D6 >> 1 = 7bit default
#define LSM9DS0_XM_ID                                                          \
  (0b01001001) //!< Accelerometer & magnetometer ID register, WHO_AM_I_XM
#define LSM9DS0_G_ID (0b11010100) //!< Gyroscope ID register, WHO_AM_I_G

// Linear Acceleration: mg per LSB
#define LSM9DS0_ACCEL_MG_LSB_2G                                                \
  (0.061F) //!< Linear acceleration sensitivity FS=+-2g
#define LSM9DS0_ACCEL_MG_LSB_4G                                                \
  (0.122F) //!< Linear acceleration sensitivity FS=+-4g

#define LSM9DS0_ACCEL_MG_LSB_6G                                                \
  (0.183F) //!< Linear acceleration sensitivity FS=+-6g
#define LSM9DS0_ACCEL_MG_LSB_8G                                                \
  (0.244F) //!< Linear acceleration sensitivity FS=+-8g
#define LSM9DS0_ACCEL_MG_LSB_16G                                               \
  (0.732F) //!< Linear acceleration sensitivity FS=+-16g

// Magnetic Field Strength: gauss range
#define LSM9DS0_MAG_MGAUSS_2GAUSS (0.08F) //!< Magnetic sensitivity FS=+-2 gauss
#define LSM9DS0_MAG_MGAUSS_4GAUSS (0.16F) //!< Magnetic sensitivity FS=+-4 gauss
#define LSM9DS0_MAG_MGAUSS_8GAUSS (0.32F) //!< Magnetic sensitivity FS=+-8 gauss
#define LSM9DS0_MAG_MGAUSS_12GAUSS                                             \
  (0.48F) //!< Magnetic sensitivity FS=+-12 gauss

// Angular Rate: dps per LSB
#define LSM9DS0_GYRO_DPS_DIGIT_245DPS                                          \
  (0.00875F) //!< Angular rate sensitivity FS=+-245dps
#define LSM9DS0_GYRO_DPS_DIGIT_500DPS                                          \
  (0.01750F) //!< Angular rate sensitivity FS=+-500dps
#define LSM9DS0_GYRO_DPS_DIGIT_2000DPS                                         \
  (0.07000F) //!< Angular rate sensitivity FS=+-2000dps

// Temperature: LSB per degree celsius
#define LSM9DS0_TEMP_LSB_DEGREE_CELSIUS                                        \
  (8) //!< Temperature sensor output change vs. temperature. 1°C = 8, 25° = 200,
      //!< etc.

#define GYROTYPE (true) //!< Used to enable gyroscope device
#define XMTYPE (false)  //!< Used to enable accellerometer & magnetometer device

/* Forward reference required for function pointers below. */
class Adafruit_LSM9DS0;

/* Pointer to member functions for read, get event, and get sensor.  These are
 * used */
/* by the Adafruit_LSM9DS0::Sensor class to read and retrieve individual
 * sensors. */
typedef void (Adafruit_LSM9DS0::*lsm9ds0_read_func)(
    void); //!< Pointer to member functions to read. Used by
           //!< Adafruit_LSM9DS0::Sensor class.
typedef void (Adafruit_LSM9DS0::*lsm9ds0_get_event_func)(
    sensors_event_t *, uint32_t); //!< Pointer to member functions to get event.
                                  //!< Used by Adafruit_LSM9DS0::Sensor class.
typedef void (Adafruit_LSM9DS0::*lsm9ds0_get_sensor_func)(
    sensor_t *); //!< Pointer to member functions to get sensor function. Used
                 //!< by Adafruit_LSM9DS0::Sensor class.

/**! Interface object for LSM9DS0 9-DoF sensor */
class Adafruit_LSM9DS0 {
public:
  Adafruit_LSM9DS0(int32_t sensorID = 0);
  Adafruit_LSM9DS0(TwoWire *wireBus, int32_t sensorID = 0);
  Adafruit_LSM9DS0(int8_t xmcs, int8_t gcs, int32_t sensorID = 0);
  Adafruit_LSM9DS0(int8_t clk, int8_t miso, int8_t mosi, int8_t xmcs,
                   int8_t gcs, int32_t sensorID = 0);

  /**! Register mapping for gyro component */
  typedef enum {
    LSM9DS0_REGISTER_WHO_AM_I_G = 0x0F,
    LSM9DS0_REGISTER_CTRL_REG1_G = 0x20,
    LSM9DS0_REGISTER_CTRL_REG3_G = 0x22,
    LSM9DS0_REGISTER_CTRL_REG4_G = 0x23,
    LSM9DS0_REGISTER_OUT_X_L_G = 0x28,
    LSM9DS0_REGISTER_OUT_X_H_G = 0x29,
    LSM9DS0_REGISTER_OUT_Y_L_G = 0x2A,
    LSM9DS0_REGISTER_OUT_Y_H_G = 0x2B,
    LSM9DS0_REGISTER_OUT_Z_L_G = 0x2C,
    LSM9DS0_REGISTER_OUT_Z_H_G = 0x2D,
  } lsm9ds0GyroRegisters_t;

  /**! Register mapping for accelerometer/mag component */
  typedef enum {
    LSM9DS0_REGISTER_TEMP_OUT_L_XM = 0x05,
    LSM9DS0_REGISTER_TEMP_OUT_H_XM = 0x06,
    LSM9DS0_REGISTER_STATUS_REG_M = 0x07,
    LSM9DS0_REGISTER_OUT_X_L_M = 0x08,
    LSM9DS0_REGISTER_OUT_X_H_M = 0x09,
    LSM9DS0_REGISTER_OUT_Y_L_M = 0x0A,
    LSM9DS0_REGISTER_OUT_Y_H_M = 0x0B,
    LSM9DS0_REGISTER_OUT_Z_L_M = 0x0C,
    LSM9DS0_REGISTER_OUT_Z_H_M = 0x0D,
    LSM9DS0_REGISTER_WHO_AM_I_XM = 0x0F,
    LSM9DS0_REGISTER_INT_CTRL_REG_M = 0x12,
    LSM9DS0_REGISTER_INT_SRC_REG_M = 0x13,
    LSM9DS0_REGISTER_CTRL_REG1_XM = 0x20,
    LSM9DS0_REGISTER_CTRL_REG2_XM = 0x21,
    LSM9DS0_REGISTER_CTRL_REG5_XM = 0x24,
    LSM9DS0_REGISTER_CTRL_REG6_XM = 0x25,
    LSM9DS0_REGISTER_CTRL_REG7_XM = 0x26,
    LSM9DS0_REGISTER_OUT_X_L_A = 0x28,
    LSM9DS0_REGISTER_OUT_X_H_A = 0x29,
    LSM9DS0_REGISTER_OUT_Y_L_A = 0x2A,
    LSM9DS0_REGISTER_OUT_Y_H_A = 0x2B,
    LSM9DS0_REGISTER_OUT_Z_L_A = 0x2C,
    LSM9DS0_REGISTER_OUT_Z_H_A = 0x2D,
  } lsm9ds0MagAccelRegisters_t;

  /**! Enumeration for accelerometer range (2/4/6/8/16 g) */
  typedef enum {
    LSM9DS0_ACCELRANGE_2G = (0b000 << 3),
    LSM9DS0_ACCELRANGE_4G = (0b001 << 3),
    LSM9DS0_ACCELRANGE_6G = (0b010 << 3),
    LSM9DS0_ACCELRANGE_8G = (0b011 << 3),
    LSM9DS0_ACCELRANGE_16G = (0b100 << 3)
  } lsm9ds0AccelRange_t;

  /**! Enumeration for accelerometer data rage 3.125 - 1600 Hz */
  typedef enum {
    LSM9DS0_ACCELDATARATE_POWERDOWN = (0b0000 << 4),
    LSM9DS0_ACCELDATARATE_3_125HZ = (0b0001 << 4),
    LSM9DS0_ACCELDATARATE_6_25HZ = (0b0010 << 4),
    LSM9DS0_ACCELDATARATE_12_5HZ = (0b0011 << 4),
    LSM9DS0_ACCELDATARATE_25HZ = (0b0100 << 4),
    LSM9DS0_ACCELDATARATE_50HZ = (0b0101 << 4),
    LSM9DS0_ACCELDATARATE_100HZ = (0b0110 << 4),
    LSM9DS0_ACCELDATARATE_200HZ = (0b0111 << 4),
    LSM9DS0_ACCELDATARATE_400HZ = (0b1000 << 4),
    LSM9DS0_ACCELDATARATE_800HZ = (0b1001 << 4),
    LSM9DS0_ACCELDATARATE_1600HZ = (0b1010 << 4)
  } lm9ds0AccelDataRate_t;

  /**! Enumeration for magnetometer scaling (2/4/8/12 gauss) */
  typedef enum {
    LSM9DS0_MAGGAIN_2GAUSS = (0b00 << 5), // +/- 2 gauss
    LSM9DS0_MAGGAIN_4GAUSS = (0b01 << 5), // +/- 4 gauss
    LSM9DS0_MAGGAIN_8GAUSS = (0b10 << 5), // +/- 8 gauss
    LSM9DS0_MAGGAIN_12GAUSS = (0b11 << 5) // +/- 12 gauss
  } lsm9ds0MagGain_t;

  /**! Enumeration for magnetometer data rate: 3.125 Hz - 100 Hz */
  typedef enum {
    LSM9DS0_MAGDATARATE_3_125HZ = (0b000 << 2),
    LSM9DS0_MAGDATARATE_6_25HZ = (0b001 << 2),
    LSM9DS0_MAGDATARATE_12_5HZ = (0b010 << 2),
    LSM9DS0_MAGDATARATE_25HZ = (0b011 << 2),
    LSM9DS0_MAGDATARATE_50HZ = (0b100 << 2),
    LSM9DS0_MAGDATARATE_100HZ = (0b101 << 2)
  } lsm9ds0MagDataRate_t;

  /**! Enumeration for gyroscope scaling (245/500/2000 dps) */
  typedef enum {
    LSM9DS0_GYROSCALE_245DPS =
        (0b00 << 4), // +/- 245 degrees per second rotation
    LSM9DS0_GYROSCALE_500DPS =
        (0b01 << 4), // +/- 500 degrees per second rotation
    LSM9DS0_GYROSCALE_2000DPS =
        (0b10 << 4) // +/- 2000 degrees per second rotation
  } lsm9ds0GyroScale_t;

  /**! 3D floating point vector with X Y Z components */
  typedef struct vector_s {
    float x; ///< X component
    float y; ///< Y component
    float z; ///< Z component
  } lsm9ds0Vector_t;

  lsm9ds0Vector_t
      accelData; ///< Last read accelerometer data will be available here
  lsm9ds0Vector_t
      magData; ///< Last read magnetometer data will be available here
  lsm9ds0Vector_t gyroData; ///< Last read gyroscope data will be available here
  int16_t temperature; ///< Last read temperzture data will be available here

  bool begin(void);
  void read(void);
  void readAccel(void);
  void readMag(void);
  void readGyro(void);
  void readTemp(void);
  void setupAccel(lsm9ds0AccelRange_t range);
  void setupMag(lsm9ds0MagGain_t gain);
  void setupGyro(lsm9ds0GyroScale_t scale);

  /* Adafruit Unified Sensor Functions (not standard yet ... the current base
   * class only */
  /* supports one sensor type, and we need to update the unified base class to
   * support   */
  /* multiple sensors in a single driver, returning an array */
  bool getEvent(sensors_event_t *accel, sensors_event_t *mag,
                sensors_event_t *gyro, sensors_event_t *temp);
  void getSensor(sensor_t *accel, sensor_t *mag, sensor_t *gyro,
                 sensor_t *temp);

  /**! Subclass to expose each sensor on the LSM9DS1 as an Adafruit_Sensor
   * instance. */
  class Sensor : public Adafruit_Sensor {
  public:
    /*! @brief Basic instatiator */
    Sensor() {}
    /*! @brief Instantiate based on existing sensor
        @param copy The Sensor object to 'copy' from */
    Sensor(const Sensor &copy)
        : _parent(copy._parent), _readFunc(copy._readFunc),
          _eventFunc(copy._eventFunc), _sensorFunc(copy._sensorFunc) {}

    /*! @brief Instantiate based on existing sensor
        @param parent The LSM9DS1 parent object
        @param readFunc The no-argument function to call to read data
        @param eventFunc The sensor_event_t function to call to get event data
        @param sensorFunc The sensor_t function to call to get sensor metadata
     */
    Sensor(Adafruit_LSM9DS0 *parent, lsm9ds0_read_func readFunc,
           lsm9ds0_get_event_func eventFunc, lsm9ds0_get_sensor_func sensorFunc)
        : _parent(parent), _readFunc(readFunc), _eventFunc(eventFunc),
          _sensorFunc(sensorFunc) {}
    /*! @brief Get sensor event data
        @param event Pointer to sensor_event_t to fill in
        @returns True on successful read */
    virtual bool getEvent(sensors_event_t *event) {
      /* Take new reading. */
      (_parent->*_readFunc)();
      /* Fill in event data. */
      (_parent->*_eventFunc)(event, millis());
      return true;
    }

    /*! @brief Get sensor metadata - type and range information
        @param sensor Pointer to sensor_t to fill in */
    virtual void getSensor(sensor_t *sensor) {
      /* Fill in sensor metadata. */
      (_parent->*_sensorFunc)(sensor);
    }

  private:
    Adafruit_LSM9DS0 *_parent;
    lsm9ds0_read_func _readFunc;
    lsm9ds0_get_event_func _eventFunc;
    lsm9ds0_get_sensor_func _sensorFunc;
  };

  /*! @brief Return Adafruit_Sensor compatible interface for accelerometer
      @returns Reference to Adafruit_Sensor subclassed object  */
  Sensor &getAccel(void) { return _accelSensor; }
  /*! @brief Return Adafruit_Sensor compatible interface for gyroscope
      @returns Reference to Adafruit_Sensor subclassed object  */
  Sensor &getGyro(void) { return _gyroSensor; }
  /*! @brief Return Adafruit_Sensor compatible interface for temperature
      @returns Reference to Adafruit_Sensor subclassed object  */
  Sensor &getTemp(void) { return _tempSensor; }
  /*! @brief Return Adafruit_Sensor compatible interface for magnetometer
      @returns Reference to Adafruit_Sensor subclassed object  */
  Sensor &getMag(void) { return _magSensor; }

private:
  void write8(boolean type, byte reg, byte value);
  byte read8(boolean type, byte reg);
  byte readBuffer(boolean type, byte reg, byte len, uint8_t *buffer);
  uint8_t spixfer(uint8_t data);
  void initI2C(TwoWire *wireBus, int32_t sensorID);

  boolean _i2c;
  TwoWire *_wire;
  int8_t _csg, _csxm, _mosi, _miso, _clk;
  uint8_t mySPCR, SPCRback;
  float _accel_mg_lsb;
  float _mag_mgauss_lsb;
  float _gyro_dps_digit;
  int32_t _lsm9dso_sensorid_accel;
  int32_t _lsm9dso_sensorid_mag;
  int32_t _lsm9dso_sensorid_gyro;
  int32_t _lsm9dso_sensorid_temp;
  Sensor _accelSensor;
  Sensor _magSensor;
  Sensor _gyroSensor;
  Sensor _tempSensor;

  /* Functions to get individual sensor measurements and metadata. */
  /* Note that these functions will NOT update the sensor state before getting
   */
  /* a new reading.  You MUST call read() manually to update the sensor state */
  /* before calling these functions! */
  void getAccelEvent(sensors_event_t *event, uint32_t timestamp);
  void getMagEvent(sensors_event_t *event, uint32_t timestamp);
  void getGyroEvent(sensors_event_t *event, uint32_t timestamp);
  void getTempEvent(sensors_event_t *event, uint32_t timestamp);
  void getAccelSensor(sensor_t *sensor);
  void getMagSensor(sensor_t *sensor);
  void getGyroSensor(sensor_t *sensor);
  void getTempSensor(sensor_t *sensor);
};

#endif
