/*
 * Car.hpp
 *
 *  Created on: Dec 23, 2022
 *      Author: cre
 */

#pragma once

#include <arduino.h>
#include <vector>
#include "../plugins/MotorPlugin.hpp"
#include "Mode.hpp"
#include "Parser.hpp"

class ClockPlugin;
class DemoPlugin;

#include "../plugins/KalmanPlugin.hpp"
#include "../plugins/ClockPlugin.hpp"
#include "../plugins/DemoPlugin.hpp"
#include "../plugins/DrivePlugin.hpp"
#include "../plugins/GoalPlugin.hpp"
#include "../plugins/MpuPlugin.hpp"
#include "../plugins/OdomPlugin.hpp"
#include "../plugins/UltrasoundPlugin.hpp"
#include "../plugins/WallPlugin.hpp"

const int MOTOR_COUNT = 2;

class Car: public Executor
{
    public:
        Car ();
        ~Car ();
        friend std::ostream& operator<< (std::ostream &os, const Car &car);

        void setup ();
        void set_mode (const Mode mode);
        void cycle ();
        void demo_drive_leds ();
        void execute_command (const std::vector<String> words);

        void all_stop ();
        const MotorPlugin& get_motor (const MotorLocation motor) const;
        MotorPlugin& get_motor (const MotorLocation motor);
        void drive_stop (const MotorLocation motor);
        void drive (const MotorLocation motor, const int speed);
        int get_drive_speed (const MotorLocation motor) const;
        float get_drive_velocity (const MotorLocation motor) const;
        MotorDirection get_drive_direction (const MotorLocation motor) const;
        void drive_forward (const MotorLocation motor, const int speed);
        void drive_reverse (const MotorLocation motor, const int speed);

        ClockPlugin* get_clock_plugin () const;
        DemoPlugin* get_demo_plugin () const;
        DrivePlugin* get_forward_plugin () const;
        GoalPlugin* get_goal_plugin () const;
        DrivePlugin* get_reverse_plugin () const;
        DrivePlugin* get_clockwise_plugin () const;
        DrivePlugin* get_counterclockwise_plugin () const;
        MpuPlugin* get_mpu_plugin () const;
        KalmanPlugin* get_kalman_plugin () const;
        OdomPlugin* get_odom_plugin () const;
        UltrasoundPlugin* get_ultrasound_plugin () const;
        WallPlugin* get_wall_plugin () const;
        Plugin* get_plugin (PluginId id) const;

    private:
        unsigned long cycle_count = 0;
        unsigned long total_cycle_us = 0;

        Parser serial_parser;
        Parser bluetooth_parser;

        ClockPlugin *clock_plugin;
        DrivePlugin *clockwise_plugin;
        DrivePlugin *counterclockwise_plugin;
        DemoPlugin *demo_plugin;
        DrivePlugin *forward_plugin;
        GoalPlugin *goal_plugin;
        MpuPlugin *mpu_plugin;
        KalmanPlugin *kalman_plugin;
        OdomPlugin *odom_plugin;
        DrivePlugin *reverse_plugin;
        UltrasoundPlugin *ultrasound_plugin;
        WallPlugin *wall_plugin;

        std::vector<Plugin*> available_plugins;
        std::vector<Plugin*> plugins;

        const int right_speed_counter_pin = 2; // right
        const int left_speed_counter_pin = 3; // left

        const int in1 = 49; // yellow = in1 left
        const int in2 = 48; // orange = in2 left
        const int in3 = 47; // purple = in3 right
        const int in4 = 46; // blue = in4 right
        const int enA = 45; // blue = enA left
        const int enB = 44; // green = enB right

        const int LED_0 = 22; // red led right
        const int LED_1 = 24; // green led right
        const int LED_2 = 26; // red led left
        const int LED_3 = 28; // green led left
        MotorPlugin motors[MOTOR_COUNT] =
        { MotorPlugin(PluginId::MOTOR_RIGHT_PLUGIN, MotorLocation::RIGHT, enB, in4, in3, right_speed_counter_pin, LED_0,
                LED_1), MotorPlugin(PluginId::MOTOR_LEFT_PLUGIN, MotorLocation::LEFT, enA, in1, in2, left_speed_counter_pin,
                LED_2, LED_3) };

        PluginId schedule[100] =
        { };
        const int ms_per_cycle = 10;
        int last_cycle = -1;
        Mode mode = Mode::COMMAND_MODE;

        void handle_command ();
        void help_command () const;
        int get_actual_interval (int cycle, PluginId id) const;
};

