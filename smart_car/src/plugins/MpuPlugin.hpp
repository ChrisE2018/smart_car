/*
 * MpuPlugin.hpp
 *
 *  Created on: Dec 31, 2022
 *      Author: cre
 */

#pragma once

#include "Plugin.hpp"
#include <MPU6050_6Axis_MotionApps612.h>
#include <Wire.h> // Needed for IMU

class MpuPlugin : public Plugin
{
    public:
        MpuPlugin ();
        bool setup () override;
        virtual int get_preferred_interval () const;
        virtual int get_expected_ms () const;
        void calibrate ();
        void print_imu ();
        void cycle () override;
        void readFifoBuffer ();

        float get_Ax ();
        float get_Ay ();
        float get_Az ();

        float get_yaw () const;
        float get_pitch () const;
        float get_roll () const;

        float get_temp ();

    private:
        // Use AD0_HIGH address after wiring pin ad0 to vcc = 3.3 v
        // This is done to avoid conflict with the I2C address of the RTC (clock).
        const int MPU_addr = MPU6050_ADDRESS_AD0_HIGH;  // I2C address of the MPU-6050

        MPU6050_6Axis_MotionApps612 mpu;
        uint16_t fifoCount = 0;
        uint8_t fifoBuffer[40];

        float euler[3];
        int packetSize = 0;

        const int interval = 10;
        long deadline = 0;
};

