/*
 * Car.hpp
 *
 *  Created on: Dec 23, 2022
 *      Author: cre
 */

#pragma once

#include <arduino.h>
#include <vector>
#include "../plugins/KalmanPlugin.hpp"

#include "Mode.hpp"
#include "Motor.hpp"
#include "Parser.hpp"

#include "ClockPlugin.hpp"
#include "DemoPlugin.hpp"
#include "DrivePlugin.hpp"
#include "MpuPlugin.hpp"
#include "OdomPlugin.hpp"
#include "UltrasoundPlugin.hpp"
#include "WallPlugin.hpp"

const int MOTOR_COUNT = 2;

class Car : public Executor
{
    public:
        Car ();
        ~Car ();
        friend std::ostream& operator<<(std::ostream& os, const Car& car);

        void setup ();
        Mode get_mode ();
        bool is_mode (const Mode mode);
        void set_mode (const Mode mode);
        void cycle ();
        void demo_drive_leds ();

        Motor& get_motor (const MotorLocation motor);
        void all_stop ();
        void drive_stop (const MotorLocation motor);
        void drive_forward (const MotorLocation motor, const int speed);
        void drive_reverse (const MotorLocation motor, const int speed);

        DemoPlugin* get_demo_plugin ();
        DrivePlugin* get_forward_plugin ();
        DrivePlugin* get_reverse_plugin ();
        DrivePlugin* get_clockwise_plugin ();
        DrivePlugin* get_counterclockwise_plugin ();
        MpuPlugin* get_mpu_plugin ();
        KalmanPlugin* get_navigation_plugin ();
        OdomPlugin* get_odom_plugin ();
        UltrasoundPlugin* get_ultrasound_plugin ();
        WallPlugin* get_wall_plugin ();

    private:
        long cycle_count = 0;
        long total_cycle_us = 0;

        Parser serial_parser;
        Parser bluetooth_parser;

        ClockPlugin *clock_plugin;
        DrivePlugin *clockwise_plugin;
        DrivePlugin *counterclockwise_plugin;
        DemoPlugin *demo_plugin;
        DrivePlugin *forward_plugin;
        MpuPlugin* mpu_plugin;
        KalmanPlugin *kalman_plugin;
        OdomPlugin* odom_plugin;
        DrivePlugin *reverse_plugin;
        UltrasoundPlugin *ultrasound_plugin;
        WallPlugin *wall_plugin;

        std::vector<Plugin*> available_plugins;
        std::vector<Plugin*> plugins;

        // pins
        // 3 yellow = in1
        // 4 orange = in2
        // 5 purple = in3
        // 6 blue = in4
        // 7 blue = enA
        // 8 green = enB

        // LED_0 = 22;  // red led
        // LED_1 = 24;  // green led
        // LED_2 = 26;  // red led
        // LED_3 = 28;  // green led
        Motor motors[MOTOR_COUNT] =
        { Motor(7, 3, 4, 26, 28), Motor(8, 6, 5, 22, 24) };

        Mode mode = COMMAND_MODE;

        void handle_command ();
        void execute_command (const std::vector<String> words);
        void help_command ();
        void print_distance ();
};

