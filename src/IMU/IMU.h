/**
 * @file IMU.h
 *
 * @brief Header file for IMU class.
 *
 * @author Michael Beskid
 * Contact: mjbeskid@wpi.edu
 *
 */

#ifndef _IMU_H_
#define _IMU_H_

#include "Arduino.h"
#include "MPU6050/MPU6050.h"

// Set gyro full scale range to 250 (deg/sec)
#define GYRO_SCALE MPU6050_GYRO_FS_250
#define GYRO_SCALE_FACTOR 131.0

// Set accelerometer full scale range to 2 (G's)
#define ACCEL_SCALE MPU6050_ACCEL_FS_2
#define ACCEL_SCALE_FACTOR 16384.0

class IMU {
public:
	
	IMU();

    void init();
    float getGyroX();
    float getGyroY();
    float getGyroZ();
    float getAccX();
    float getAccY();
    float getAccZ();
    float getRoll();
    float getPitch();
    float getYaw();
    void readData();
    void calculateError();
    void Madgwick(float gx, float gy, float gz, float ax, float ay, float az, float invSampleFreq);
    void printGyroData();
    void printAccelData();
    void printRollPitchYaw();

private:
    float AccX, AccY, AccZ;
    float AccX_prev, AccY_prev, AccZ_prev;
    float GyroX, GyroY, GyroZ;
    float GyroX_prev, GyroY_prev, GyroZ_prev;
    float roll_IMU, pitch_IMU, yaw_IMU;
    float roll_IMU_prev, pitch_IMU_prev;
    float q0 = 1.0f; //Initialize quaternion for madgwick filter
    float q1 = 0.0f;
    float q2 = 0.0f;
    float q3 = 0.0f;

    //Filter parameters - Defaults tuned for 2kHz loop rate; Do not touch unless you know what you are doing:
    float B_madgwick = 0.04;  //Madgwick filter parameter
    float B_accel = 0.14;     //Accelerometer LP filter paramter, (MPU6050 default: 0.14. MPU9250 default: 0.2)
    float B_gyro = 0.1;       //Gyro LP filter paramter, (MPU6050 default: 0.1. MPU9250 default: 0.17)

    //IMU calibration parameters - calibrate IMU using calculate_IMU_error() in the void setup() to get these values, then comment out calculate_IMU_error()
    float AccErrorX = 0.04;
    float AccErrorY = 0.05;
    float AccErrorZ = -0.10;
    float GyroErrorX = -2.46;
    float GyroErrorY = 0.54;
    float GyroErrorZ = -2.05;

    float invSqrt(float x);

};

#endif