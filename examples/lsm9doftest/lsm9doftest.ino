#include <Wire.h>
#include <SPI.h>
#include <Adafruit_LSM9DS0.h>

// i2c
Adafruit_LSM9DS0 lsm;

// SPI
//Adafruit_LSM9DS0 lsm = Adafruit_LSM9DS0(A5, 12, A4, 10, 9);

void setup() 
{
  while (!Serial); // flora & leonardo
  
  Serial.begin(9600);
  Serial.println("LSM demo");
  
  // Try to initialise and warn if we couldn't detect the chip
  if (!lsm.begin())
  {
    Serial.println("Oops ... unable to initialize the LSM9DS0. Check your wiring!");
    while (1);
  }
  Serial.println("Found LMS9DS0 9DOF");
}

void loop() 
{
  
  
  lsm.read();

  Serial.print("Accel X: "); Serial.print((int)lsm.accelData.x); Serial.print(" ");
  Serial.print("Y: "); Serial.print((int)lsm.accelData.y);       Serial.print(" ");
  Serial.print("Z: "); Serial.println((int)lsm.accelData.z);     Serial.print(" ");
  Serial.print("Mag X: "); Serial.print((int)lsm.magData.x);     Serial.print(" ");
  Serial.print("Y: "); Serial.print((int)lsm.magData.y);         Serial.print(" ");
  Serial.print("Z: "); Serial.println((int)lsm.magData.z);       Serial.print(" ");
  Serial.print("Gyro X: "); Serial.print((int)lsm.gyroData.x);     Serial.print(" ");
  Serial.print("Y: "); Serial.print((int)lsm.gyroData.y);         Serial.print(" ");
  Serial.print("Z: "); Serial.println((int)lsm.gyroData.z);       Serial.println(" ");
  delay(200);
}