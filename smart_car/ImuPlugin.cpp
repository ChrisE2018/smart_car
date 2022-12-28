/*
 * ImuPlugin.cpp
 *
 *  Created on: Dec 26, 2022
 *      Author: cre
 */

#include "Arduino.h"
#include "ImuPlugin.hpp"

ImuPlugin::ImuPlugin () : Plugin(IMU_PLUGIN), mpu(0x68)
{
}

void ImuPlugin::setup_imu ()
{
    Wire.begin();
    Wire.beginTransmission(MPU_addr);
    Wire.write(0x6B);  // PWR_MGMT_1 register
    Wire.write(0);     // set to zero (wakes up the MPU-6050)
    Wire.endTransmission(true);
    mpu.initialize();
    if (mpu.testConnection())
    {
        Serial.println("imu calibration...please wait 15 seconds");
        // 17:08:16: >...............>...............-1152.00000,   -3917.00000,    1930.00000, 9.00000,    -78.00000,  -17.00000
        mpu.CalibrateGyro(); // Fine tune after setting offsets with less Loops.
        mpu.CalibrateAccel(); // Fine tune after setting offsets with less Loops.
        mpu.PrintActiveOffsets(); // See the results of the Calibration

        // This long calibration is not required.
        // The built in mpu calibration above does well enough and is much faster.
        Serial.println("Long imu calibration...please wait 60 seconds");
        calibrateSensor();
        Serial.println("imu setup");
    }
    else
    {
        Serial.println("imu failed");
    }
}
//--------------------------------------------//
// Read the raw sensor values
//--------------------------------------------//
void ImuPlugin::readIMU ()
{
    // Using the MotionApps library
    accX = mpu.getAccelerationX();
    accY = mpu.getAccelerationY();
    accZ = mpu.getAccelerationZ();
    gyroX = mpu.getRotationX();
    gyroY = mpu.getRotationY();
    gyroZ = mpu.getRotationZ();
}

//----------------------------------------------------//
// Calibrate bias of the accelerometer and gyroscope
// Sensor needs to be calibrated at each power cycle.
//----------------------------------------------------//
void ImuPlugin::calibrateSensor ()
{
    // Initialize the offset values
    setOffsets(-1152.00000, -3917.00000, 1931.00000, 7.00000, -78.00000, -17.00000);

    Serial.println("Reading sensors for first time...");
    meanSensors_();
    delay(1000);

    Serial.println("Calculating offsets...");
    calibrate_();
    log_mean_values();
    log_offset_values();
}

void ImuPlugin::setOffsets (int16_t _ax_offset, int16_t _ay_offset, int16_t _az_offset,
        int16_t _gx_offset, int16_t _gy_offset, int16_t _gz_offset)
{
    ax_offset = _ax_offset;
    ay_offset = _ay_offset;
    az_offset = _az_offset;
    gx_offset = _gx_offset;
    az_offset = _az_offset;
    gx_offset = _gx_offset;
    gy_offset = _gy_offset;
    gz_offset = _gz_offset;
}

//--------------------------------------------//
// Get the mean values from the sensor
//--------------------------------------------//
void ImuPlugin::meanSensors_ ()
{
    // Discard first 100 readings
    for (int i = 0; i < 100; i++)
    {
        // read raw accel/gyro measurements from device
        readIMU();
        delay(2);
    }
    const int bufferSize = 1000;     // Number of readings to average
    long buff_ax = 0, buff_ay = 0, buff_az = 0, buff_gx = 0, buff_gy = 0, buff_gz = 0;

    for (int i = 0; i < bufferSize; i++)
    {
        // read raw accel/gyro measurements from device
        readIMU();

        buff_ax = buff_ax + accX;
        buff_ay = buff_ay + accY;
        buff_az = buff_az + accZ;
        buff_gx = buff_gx + gyroX;
        buff_gy = buff_gy + gyroY;
        buff_gz = buff_gz + gyroZ;
        delay(2); //Needed so we don't get repeated measures
    }
    mean_ax = buff_ax / bufferSize;
    mean_ay = buff_ay / bufferSize;
    mean_az = buff_az / bufferSize;
    mean_gx = buff_gx / bufferSize;
    mean_gy = buff_gy / bufferSize;
    mean_gz = buff_gz / bufferSize;
    log_mean_values();
}

//--------------------------------------------//
// Calibrate sensor
//--------------------------------------------//
void ImuPlugin::calibrate_ ()
{
    ax_offset = ax_offset - mean_ax / acel_deadzone;
    ay_offset = ay_offset - mean_ay / acel_deadzone;
    az_offset = az_offset - (16384 - mean_az) / acel_deadzone;

    gx_offset = gx_offset - mean_gx / (giro_deadzone + 1);
    gy_offset = gy_offset - mean_gy / (giro_deadzone + 1);
    gz_offset = gz_offset - mean_gz / (giro_deadzone + 1);
    for (int i = 0; i < 100 && (abs(mean_ax) > acel_deadzone || abs(mean_ay) > acel_deadzone ||
    abs(16384-mean_az) > acel_deadzone ||
    abs(mean_gx) > giro_deadzone ||
    abs(mean_gy) > giro_deadzone ||
    abs(mean_gz) > giro_deadzone); i++)
    {
        mpu.setXAccelOffset(ax_offset);
        mpu.setYAccelOffset(ay_offset);
        mpu.setZAccelOffset(az_offset);

        mpu.setXGyroOffset(gx_offset);
        mpu.setYGyroOffset(gy_offset);
        mpu.setZGyroOffset(gz_offset);

        // Get the mean values from the sensor
        meanSensors_();
        ax_offset = ax_offset - mean_ax / acel_deadzone;
        ay_offset = ay_offset - mean_ay / acel_deadzone;
        az_offset = az_offset + (16384 - mean_az) / acel_deadzone;
        gx_offset = gx_offset - mean_gx / (giro_deadzone + 1);
        gy_offset = gy_offset - mean_gy / (giro_deadzone + 1);
        gz_offset = gz_offset - mean_gz / (giro_deadzone + 1);
        log_mean_values();
    }
}

void ImuPlugin::log_mean_values ()
{
    Serial.print("mean_ax = ");
    Serial.print(mean_ax);
    Serial.print(" mean_ay = ");
    Serial.print(mean_ay);
    Serial.print(" mean_az = ");
    Serial.print(16384 - mean_az);

    Serial.print(" mean_gx = ");
    Serial.print(mean_gx);
    Serial.print(" mean_gy = ");
    Serial.print(mean_gy);
    Serial.print(" mean_gz = ");
    Serial.println(mean_gz);
}

void ImuPlugin::log_offset_values ()
{
    Serial.print("ax_offset = ");
    Serial.print(ax_offset);
    Serial.print(" ay_offset = ");
    Serial.print(ay_offset);
    Serial.print(" az_offset = ");
    Serial.print(az_offset);

    Serial.print(" gx_offset = ");
    Serial.print(gx_offset);
    Serial.print(" gy_offset = ");
    Serial.print(gy_offset);
    Serial.print(" gz_offset = ");
    Serial.println(gz_offset);
}

void ImuPlugin::cycle ()
{
    if (is_enabled())
    {
        read_imu();
    }
}

void ImuPlugin::read_imu ()
{
    Wire.beginTransmission(MPU_addr);
    Wire.write(0x3B);  // starting with register 0x3B (ACCEL_XOUT_H)
    Wire.endTransmission(false);
    Wire.requestFrom(MPU_addr, 14, true);  // request a total of 14 registers
    AcX = Wire.read() << 8 | Wire.read();  // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)
    AcY = Wire.read() << 8 | Wire.read();  // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
    AcZ = Wire.read() << 8 | Wire.read();  // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
    Tmp = Wire.read() << 8 | Wire.read();  // 0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)
    GyX = Wire.read() << 8 | Wire.read();  // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
    GyY = Wire.read() << 8 | Wire.read();  // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
    GyZ = Wire.read() << 8 | Wire.read();  // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)

    Serial.print("AcX = "); // Accelerator
    Serial.print(AcX);
    Serial.print(" | AcY = ");
    Serial.print(AcY);
    Serial.print(" | AcZ = ");
    Serial.print(AcZ);
    Serial.print(" | Tmp = ");
    Serial.print(Tmp / 340.00 + 36.53);  //equation for temperature in degrees C from datasheet
    Serial.print(" | GyX = "); // Gyro
    Serial.print(GyX);
    Serial.print(" | GyY = ");
    Serial.print(GyY);
    Serial.print(" | GyZ = ");
    Serial.print(GyZ);
    Serial.println();
}
