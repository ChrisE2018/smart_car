/*
 * ImuPlugin.hpp
 *
 *  Created on: Dec 26, 2022
 *      Author: cre
 */

#pragma once

#include "Plugin.hpp"
#include <MPU6050.h>
#include <Wire.h> // Needed for IMU

class ImuPlugin: public Plugin
{
    public:
        ImuPlugin ();
        void setup_imu ();
        void setOffsets (int16_t ax_offset, int16_t ay_offset, int16_t az_offset, int16_t gx_offset,
                int16_t gy_offset, int16_t gz_offset);
        void log_mean_values();
        void log_offset_values();
        void readIMU ();
        void calibrateSensor ();
        void meanSensors_ ();
        void calibrate_ ();
        void read_imu ();
        void cycle () override;

    private:
        MPU6050 mpu;
        const int MPU_addr = 0x68;  // I2C address of the MPU-6050
        int16_t AcX = 0;
        int16_t AcY = 0;
        int16_t AcZ = 0;
        int16_t Tmp = 0;
        int16_t GyX = 0;
        int16_t GyY = 0;
        int16_t GyZ = 0;

        int acel_deadzone = 8;     //Accelerometer error allowed
        int giro_deadzone = 1;     //Giro error allowed

        float mean_ax = 0;
        float mean_ay = 0;
        float mean_az = 0;
        float mean_gx = 0;
        float mean_gy = 0;
        float mean_gz = 0;

        int16_t ax_offset = 0;
        int16_t ay_offset = 0;
        int16_t az_offset = 0;
        int16_t gx_offset = 0;
        int16_t gy_offset = 0;
        int16_t gz_offset = 0;

        int16_t accX = 0;
        int16_t accY = 0;
        int16_t accZ = 0;
        int16_t gyroX = 0;
        int16_t gyroY = 0;
        int16_t gyroZ = 0;

};

