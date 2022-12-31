/*
 * MpuPlugin.cpp
 *
 *  Created on: Dec 26, 2022
 *      Author: cre
 */

#include "Arduino.h"
#include "MpuPlugin.hpp"
#include "smart_car.hpp"

MpuPlugin::MpuPlugin () : Plugin(MPU_PLUGIN), mpu(MPU_addr)
{
}

bool MpuPlugin::setup ()
{
    mpu.initialize();

    // verify connection
    Serial.println(F("Testing device connections..."));
    if (mpu.testConnection())
    {
        Serial.println("MPU6050 connection successful");

        // load and configure the DMP
        Serial.println(F("Initializing DMP..."));
        const uint8_t devStatus = mpu.dmpInitialize();

        // make sure it worked (returns 0 if so)
        if (devStatus == 0)
        {
            // supply your own gyro offsets here, scaled for min sensitivity
            mpu.setXAccelOffset(-1084);
            mpu.setYAccelOffset(-3907);
            mpu.setZAccelOffset(1942);

            mpu.setXGyroOffset(5);
            mpu.setYGyroOffset(-79);
            mpu.setZGyroOffset(-20);

            // turn on the DMP, now that it's ready
            Serial.println(F("Enabling DMP..."));
            mpu.setDMPEnabled(true);

            // get expected DMP packet size for later comparison
            packetSize = mpu.dmpGetFIFOPacketSize();
            return true;
        }
        else
        {
            // ERROR!
            // 1 = initial memory load failed
            // 2 = DMP configuration updates failed
            // (if it's going to break, usually the code will be 1)
            Serial.print("DMP Initialization failed with devStatus = ");
            Serial.println(devStatus);
        }
    }
    else
    {
        Serial.println("MPU6050 connection failed");
    }
    Serial.println("MMU Initialization failed");
    return false;
}

void MpuPlugin::calibrate ()
{
    Serial.println("imu calibration...please wait 15 seconds");
    // 17:08:16: >...............>...............-1152.00000,   -3917.00000,    1930.00000, 9.00000,    -78.00000,  -17.00000
    mpu.CalibrateGyro(7);     // Fine tune after setting offsets with less Loops.
    mpu.CalibrateAccel(7);     // Fine tune after setting offsets with less Loops.
    mpu.PrintActiveOffsets();     // See the results of the Calibration
}

void MpuPlugin::cycle ()
{
    if (is_enabled())
    {
        readFifoBuffer();

        cout << "Yaw: " << get_yaw() << " Pitch: " << get_pitch() << " Roll: " << get_roll()
                << " Ax: " << get_Ax() << " Ay: " << get_Ay() << " Az: " << get_Az() << "\n";
    }
}

void MpuPlugin::readFifoBuffer ()
{
    // Clear the buffer so as we can get fresh values
    // The sensor is running a lot faster than our sample period
    mpu.resetFIFO();

    // get current FIFO count
    fifoCount = mpu.getFIFOCount();

    // wait for correct available data length, should be a VERY short wait
    while (fifoCount < packetSize)
        fifoCount = mpu.getFIFOCount();

    // read a packet from FIFO
    mpu.getFIFOBytes(fifoBuffer, packetSize);
    Quaternion q;
    mpu.dmpGetQuaternion(&q, fifoBuffer);

    mpu.dmpGetEuler(euler, &q);
}

float MpuPlugin::get_Ax ()
{
    return mpu.getAccelerationX() / 16384.0;
}

float MpuPlugin::get_Ay ()
{
    return mpu.getAccelerationY() / 16384.0;
}

float MpuPlugin::get_Az ()
{
    return (mpu.getAccelerationZ() - 16384) / 16384.0;
}

float MpuPlugin::get_yaw ()
{
    return euler[0];
}

float MpuPlugin::get_pitch ()
{
    return euler[1];
}

float MpuPlugin::get_roll ()
{
    return euler[2];
}

float MpuPlugin::get_temp ()
{
    return mpu.getTemperature() / 340.00 + 36.53;
}

void MpuPlugin::print_imu ()
{
    Serial.print("AcX = "); // Accelerator
    Serial.print(get_Ax());
    Serial.print(" | AcY = ");
    Serial.print(get_Ay());
    Serial.print(" | AcZ = ");
    Serial.print(get_Az());
    Serial.print(" | Tmp = ");
    Serial.print(get_temp());
    Serial.print(" | pitch = "); // Gyro
    Serial.print(get_pitch());
    Serial.print(" | roll = ");
    Serial.print(get_roll());
    Serial.print(" | yaw = ");
    Serial.print(get_yaw());
    Serial.println();
}
