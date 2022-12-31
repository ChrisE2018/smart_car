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
        bool setup () override;
        void calibrate ();
        void read_imu ();
        void print_imu () const;
        void cycle () override;

        float get_Ax();
        float get_Ay();
        float get_Az();

        float get_Gx();
        float get_Gy();
        float get_Gz();

        float get_temp();

    private:
        // Use AD0_HIGH address after wiring pin ad0 to vcc = 3.3 v
        // This is done to avoid conflict with the I2C address of the RTC (clock).
        const int MPU_addr = MPU6050_ADDRESS_AD0_HIGH;  // I2C address of the MPU-6050

        MPU6050 mpu;
        int16_t AcX = 0;
        int16_t AcY = 0;
        int16_t AcZ = 0;
        int16_t Tmp = 0;
        int16_t GyX = 0;
        int16_t GyY = 0;
        int16_t GyZ = 0;
};

