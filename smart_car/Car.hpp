/*
 * Car.hpp
 *
 *  Created on: Dec 23, 2022
 *      Author: cre
 */

#pragma once

#include <SR04.h>
#include <MPU6050.h>
#include <Wire.h> // Needed for IMU
#include <vector>
#include "DemoPlugin.hpp"
#include "DrivePlugin.hpp"

#include "Motor.hpp"
#include "Parser.hpp"
#include "Mode.hpp"
#include "Plugin.hpp"
#include "WallPlugin.hpp"

const int MOTOR_COUNT = 2;

const int ULTRASOUND_TRIGGER = 12;  // blue
const int ULTRASOUND_ECHO = 11;     // green

class Car: public Executor
{
    public:
        Car ();
        ~Car ();

        void setup ();
        Mode get_mode ();
        bool is_mode (Mode mode);
        void set_mode (Mode mode);
        void cycle ();
        void demo_drive_leds ();
        void all_stop ();
        void drive_stop (MotorLocation motor);
        void drive_forward (MotorLocation motor, int speed);
        void drive_reverse (MotorLocation motor, int speed);
        long get_distance ();

        WallPlugin* get_wall_mode ();
        DemoPlugin* get_demo_mode ();
        DrivePlugin* get_forward_mode ();
        DrivePlugin* get_reverse_mode ();
        DrivePlugin* get_clockwise_mode ();
        DrivePlugin* get_counterclockwise_mode ();

    private:
        long cycle_count = 0;
        long total_cycle_us = 0;

        Parser parser;
        Parser parser1;

        WallPlugin *wall_mode;
        DemoPlugin *demo_mode;
        DrivePlugin *forward_mode;
        DrivePlugin *reverse_mode;
        DrivePlugin *clockwise_mode;
        DrivePlugin *counterclockwise_mode;
        std::vector<Plugin*> plugins;

        // pins
        // 2 yellow = in1
        // 3 orange = in2
        // 4 purple = in3
        // 5 blue = in4
        // 6 blue = enA
        // 7 green = enB

        // LED_0 = 22;  // red led
        // LED_1 = 24;  // green led
        // LED_2 = 26;  // red led
        // LED_3 = 28;  // green led
        Motor motors[MOTOR_COUNT] =
        { Motor(6, 2, 3, 26, 28), Motor(7, 5, 4, 22, 24) };

        Mode mode = COMMAND_MODE;

        SR04 sr04;
        bool show_distance = false;
        bool show_imu = false;

        MPU6050 mpu;
        const int MPU_addr = 0x68;  // I2C address of the MPU-6050
        int16_t AcX = 0;
        int16_t AcY = 0;
        int16_t AcZ = 0;
        int16_t Tmp = 0;
        int16_t GyX = 0;
        int16_t GyY = 0;
        int16_t GyZ = 0;

        void setup_imu ();
        void read_imu ();
        void handle_command ();
        void execute_command (const int n, const String words[]);
        void help_command ();
        void print_distance ();
};

