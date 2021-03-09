/*
  This file is part of the AHRS_Nano33BLE_LSM9DS1 library.

  Referenced https://raw.githubusercontent.com/kriswiner/LSM9DS1/master/LSM9DS1_BasicAHRS_Nano33.ino

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
*/

#include <Arduino.h>
#include <Wire.h>

class AHRS_Nano33BLE_LSM9DS1Class {
  public:
    AHRS_Nano33BLE_LSM9DS1Class(TwoWire& wire);
    virtual ~AHRS_Nano33BLE_LSM9DS1Class();

    //=========================================================================
    // For interfacing with LSM9DS1

    int start();
    void stop();

    void initLSM9DS1();
    int selftestLSM9DS1();

    float accelerationSensitivity()
    {
      return 1./(1000. * aRes); // LSB/mg
    }

    float gyroscopeSensitivity()
    {
      return 1./(1000. * gRes); // LSB/mdps
    }

    float magnometerSensitivity()
    {
      return 1./(1000. * mRes); // LSB/mGauss
    }

    bool accelerometerReady();
    bool gyroscopeReady();
    bool magnometerReady();

    void readAccel(float& ax, float& ay, float& az)
    {
      readAccelData(accelCount);  // Read the x/y/z adc values

      // Now we'll calculate the accleration value into actual g's
      ax = (float)accelCount[0]*aRes - accelBias[0];  // get actual g value, this depends on scale being set
      ay = (float)accelCount[1]*aRes - accelBias[1];   
      az = (float)accelCount[2]*aRes - accelBias[2]; 
    }

    void readGyro(float& gx, float& gy, float& gz)
    {
      readGyroData(gyroCount);  // Read the x/y/z adc values

      // Calculate the gyro value into actual degrees per second
      gx = (float)gyroCount[0]*gRes - gyroBias[0];  // get actual gyro value, this depends on scale being set
      gy = (float)gyroCount[1]*gRes - gyroBias[1];  
      gz = (float)gyroCount[2]*gRes - gyroBias[2];  
    }

    void readMag(float& mx, float& my, float& mz)
    {
      readMagData(magCount);  // Read the x/y/z adc values

      // Calculate the magnetometer values in milliGauss
      // Include factory calibration per data sheet and user environmental corrections
      mx = (float)magCount[0]*mRes; // - magBias[0];  // get actual magnetometer value, this depends on scale being set
      my = (float)magCount[1]*mRes; // - magBias[1];  
      mz = (float)magCount[2]*mRes; // - magBias[2];   
    }

    float readTempC()
    {
      tempCount = readTempData();  // Read the gyro adc values
      temperature = ((float) tempCount/256. + 25.0); // Gyro chip temperature in degrees Centigrade
      return temperature;
    }

    float* getAccelBias() 
    {
      memcpy(_copyaccelBias, accelBias, 3 * sizeof *accelBias);
      return _copyaccelBias;
    }

    float* getGyroBias()
    {
      memcpy(_copygyroBias, gyroBias, 3 * sizeof *gyroBias);
      return _copygyroBias;
    }

    float* getMagBias()
    {
      memcpy(_copymagBias, magBias, 3 * sizeof *magBias);
      return _copymagBias;
    }

    void calibrateAccelGyro()
    {
      accelgyrocalLSM9DS1(gyroBias, accelBias);
    }

    void calibrateMag()
    {
      magcalLSM9DS1(magBias);
    }

    //=========================================================================
    // For Basic AHRS

    float updateDeltat()
    {
      Now = micros();
      deltat = ((Now - lastUpdate) / 1000000.0f); // set integration time by time elapsed since last filter update
      lastUpdate = Now;
      return deltat;
    }

    void MadgwickQuaternionUpdate(float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz, float deltat);

    float rollDegrees()
    {
      if (!anglesComputed) computeRollPitchYaw();
      return roll;
    }

    float pitchDegrees()
    {
      if (!anglesComputed) computeRollPitchYaw();
      return pitch;
    }
    float yawDegrees()
    {
      if (!anglesComputed) computeRollPitchYaw();
      return yaw;
    }

    float rollRadians()
    {
      if (!anglesComputed) computeRollPitchYaw();
      return roll * DEG_TO_RAD;
    }

    float pitchRadians()
    {
      if (!anglesComputed) computeRollPitchYaw();
      return pitch * DEG_TO_RAD;
    }

    float yawRadians()
    {
      if (!anglesComputed) computeRollPitchYaw();
      return yaw * DEG_TO_RAD;
    }

    float* getQ() {
      memcpy(_copyq, q, 4 * sizeof *q);
      return _copyq;
    }

  private:
    // From Basic AHRS::
    void getAres();
    void getGres();
    void getMres();
    void accelgyrocalLSM9DS1(float * dest1, float * dest2);
    void magcalLSM9DS1(float * dest1);
    void readAccelData(int16_t * destination);
    void readGyroData(int16_t * destination);
    void readMagData(int16_t * destination);
    int16_t readTempData();
    void writeByte(uint8_t address, uint8_t subAddress, uint8_t data);
    uint8_t readByte(uint8_t address, uint8_t subAddress);
    void readBytes(uint8_t address, uint8_t subAddress, uint8_t count, uint8_t * dest);
    void computeRollPitchYaw();
 
    // Set initial input parameters
    enum Ascale {  // set of allowable accel full scale settings
      AFS_2G = 0,
      AFS_16G,
      AFS_4G,
      AFS_8G
    };

    enum Aodr {  // set of allowable gyro sample rates
      AODR_PowerDown = 0,
      AODR_10Hz,
      AODR_50Hz,
      AODR_119Hz,
      AODR_238Hz,
      AODR_476Hz,
      AODR_952Hz
    };

    enum Abw {  // set of allowable accewl bandwidths
      ABW_408Hz = 0,
      ABW_211Hz,
      ABW_105Hz,
      ABW_50Hz
    };

    enum Gscale {  // set of allowable gyro full scale settings
      GFS_245DPS = 0,
      GFS_500DPS,
      GFS_NoOp,
      GFS_2000DPS
    };

    enum Godr {  // set of allowable gyro sample rates
      GODR_PowerDown = 0,
      GODR_14_9Hz,
      GODR_59_5Hz,
      GODR_119Hz,
      GODR_238Hz,
      GODR_476Hz,
      GODR_952Hz
    };

    enum Gbw {   // set of allowable gyro data bandwidths
      GBW_low = 0,  // 14 Hz at Godr = 238 Hz,  33 Hz at Godr = 952 Hz
      GBW_med,      // 29 Hz at Godr = 238 Hz,  40 Hz at Godr = 952 Hz
      GBW_high,     // 63 Hz at Godr = 238 Hz,  58 Hz at Godr = 952 Hz
      GBW_highest   // 78 Hz at Godr = 238 Hz, 100 Hz at Godr = 952 Hz
    };

    enum Mscale {  // set of allowable mag full scale settings
      MFS_4G = 0,
      MFS_8G,
      MFS_12G,
      MFS_16G
    };

    enum Mmode {
      MMode_LowPower = 0, 
      MMode_MedPerformance,
      MMode_HighPerformance,
      MMode_UltraHighPerformance
    };

    enum Modr {  // set of allowable mag sample rates
      MODR_0_625Hz = 0,
      MODR_1_25Hz,
      MODR_2_5Hz,
      MODR_5Hz,
      MODR_10Hz,
      MODR_20Hz,
      MODR_80Hz
    };

  private:
    bool anglesComputed;
    float Now,lastUpdate,deltat;
    float roll, pitch, yaw;

    
    // Specify sensor full scale
    uint8_t Gscale, Godr, Gbw;  //gyro full scale, gyro data sample rate, gyro data bandwidth
    uint8_t Ascale, Aodr, Abw;  // accel full scale, accel data sample rate, accel data bandwidth
    uint8_t Mscale, Modr, Mmode;  // mag full scale, mag data sample rate, magnetometer operation mode
    float aRes, gRes, mRes;      // scale resolutions per LSB for the sensors

    float q[4];    // vector to hold quaternion
    float _copyq[4];    // vector to hold quaternion copy
    int16_t accelCount[3], gyroCount[3], magCount[3];  // Stores the 16-bit signed accelerometer, gyro, and mag sensor output
    float gyroBias[3], accelBias[3], magBias[3]; // Bias corrections for gyro, accelerometer, and magnetometer
    float _copygyroBias[3], _copyaccelBias[3], _copymagBias[3]; // Copy of bias corrections for gyro, accelerometer, and magnetometer
    int16_t tempCount;            // temperature raw count output
    float   temperature;          // Stores the LSM9DS1gyro internal chip temperature in degrees Celsius

    // global constants for 9 DoF fusion and AHRS (Attitude and Heading Reference System)
    float GyroMeasError;   // gyroscope measurement error in rads/s (start at 40 deg/s)
    float GyroMeasDrift;   // gyroscope measurement drift in rad/s/s (start at 0.0 deg/s/s)

    float beta;   // free parameter in the Madgwick scheme
    float zeta;   // compute zeta, the other free parameter in the Madgwick scheme usually set to a small or zero value

  private:
    TwoWire* _wire;
};

extern AHRS_Nano33BLE_LSM9DS1Class IMU;
