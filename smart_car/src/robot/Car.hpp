/*
 * Car.hpp
 *
 *  Created on: Dec 23, 2022
 *      Author: cre
 */

#pragma once

#include <arduino.h>
#include <vector>
#include "../logging/Logger.hpp"
#include "../plugins/MotorPlugin.hpp"
#include "Mode.hpp"
#include "Parser.hpp"

class ClockPlugin;
class DemoPlugin;
class DrivePlugin;
class GoalPlugin;
class KalmanPlugin;
class MpuPlugin;
class OdomPlugin;
class UltrasoundPlugin;
class WallPlugin;

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
        virtual void execute_command (const std::vector<String> &words) override;

        void all_stop ();
        const MotorPlugin& get_motor (const MotorLocation motor) const;
        MotorPlugin& get_motor (const MotorLocation motor);
        void drive_stop (const MotorLocation motor);
        void drive (const MotorLocation motor, const int speed);
        int get_drive_speed (const MotorLocation motor) const;
        float get_measured_velocity (const MotorLocation motor) const;
        float get_desired_velocity (const MotorLocation motor) const;
        void set_desired_velocity (const MotorLocation motor, const float velocity);
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

    private:
        Logger logger;
        unsigned long cycle_count = 0;
        unsigned long total_cycle_us = 0;
        const bool use_simple_cycle = true;

        Parser serial_parser;
        Parser bluetooth_parser;

        ClockPlugin *const clock_plugin;
        DrivePlugin *const clockwise_plugin;
        DrivePlugin *const counterclockwise_plugin;
        DemoPlugin *const demo_plugin;
        DrivePlugin *const forward_plugin;
        GoalPlugin *const goal_plugin;
        MpuPlugin *const mpu_plugin;
        KalmanPlugin *const kalman_plugin;
        OdomPlugin *const odom_plugin;
        DrivePlugin *const reverse_plugin;
        UltrasoundPlugin *const ultrasound_plugin;
        WallPlugin *const wall_plugin;

        std::vector<Plugin*> available_plugins;
        std::vector<Plugin*> plugins;

        static constexpr int right_speed_counter_pin = 2; // right
        static constexpr int left_speed_counter_pin = 3; // left

        static constexpr int in1 = 49; // yellow = in1 left
        static constexpr int in2 = 48; // orange = in2 left
        static constexpr int in3 = 47; // purple = in3 right
        static constexpr int in4 = 46; // blue = in4 right
        static constexpr int enA = 45; // blue = enA left
        static constexpr int enB = 44; // green = enB right

        static constexpr int RIGHT_LED_FORWARD = 22; // red led right
        static constexpr int RIGHT_LED_REVERSE = 24; // green led right
        static constexpr int LEFT_LED_FORWARD = 26; // red led left
        static constexpr int LEFT_LED_REVERSE = 28; // green led left
        MotorPlugin motors[MOTOR_COUNT] =
        { MotorPlugin(PluginId::MOTOR_RIGHT_PLUGIN, MotorLocation::RIGHT, enB, in4, in3, right_speed_counter_pin,
                RIGHT_LED_FORWARD, RIGHT_LED_REVERSE), MotorPlugin(PluginId::MOTOR_LEFT_PLUGIN, MotorLocation::LEFT,
                enA, in1, in2, left_speed_counter_pin, LEFT_LED_FORWARD, LEFT_LED_REVERSE) };

        static constexpr int schedule_size = 70;
        PluginId schedule[schedule_size];
        Mode mode = Mode::COMMAND_MODE;

        void schedule_cycle ();
        void simple_cycle ();
        void command_cycle ();
        void help_command () const;
        Plugin* get_plugin (const PluginId id) const;
        int get_actual_interval (const int cycle, const PluginId id) const;
};

