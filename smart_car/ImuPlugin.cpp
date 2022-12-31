/*
 * ImuPlugin.cpp
 *
 *  Created on: Dec 26, 2022
 *      Author: cre
 */

#include "Arduino.h"
#include "ImuPlugin.hpp"

ImuPlugin::ImuPlugin () : Plugin(IMU_PLUGIN), mpu(MPU_addr)
{
}

bool ImuPlugin::setup ()
{
    Wire.begin();
    Wire.beginTransmission(MPU_addr);
    Wire.write(0x6B);  // PWR_MGMT_1 register
    Wire.write(0);     // set to zero (wakes up the MPU-6050)
    Wire.endTransmission(true);
    mpu.initialize();
    if (mpu.testConnection())
    {
        Serial.println("imu setup");
        return true;
    }
    else
    {
        Serial.println("imu failed");
        return false;
    }
}

void ImuPlugin::calibrate ()
{
    Serial.println("imu calibration...please wait 15 seconds");
    // 17:08:16: >...............>...............-1152.00000,   -3917.00000,    1930.00000, 9.00000,    -78.00000,  -17.00000
    mpu.CalibrateGyro(7); // Fine tune after setting offsets with less Loops.
    mpu.CalibrateAccel(7); // Fine tune after setting offsets with less Loops.
    mpu.PrintActiveOffsets(); // See the results of the Calibration
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
