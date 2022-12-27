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

class ImuPlugin : public Plugin
{
    public:
        ImuPlugin ();
        void setup_imu ();
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
};

