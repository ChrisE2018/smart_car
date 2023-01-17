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
#include "board_pins.hpp"

class ClockPlugin;
class DemoPlugin;
class DrivePlugin;
class GoalPlugin;
class KalmanPlugin;
class MpuPlugin;
class OdomPlugin;
class UltrasoundPlugin;
class WallPlugin;

const int MOTOR_COUNT = 4;

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
        DrivePlugin* get_forward_plugin () const;
        GoalPlugin* get_goal_plugin () const;
        DrivePlugin* get_reverse_plugin () const;
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

        // Left side
        static constexpr int left_rear_in1 = PIN_49_DIG; // yellow = in1
        static constexpr int left_rear_in2 = PIN_48_DIG; // orange = in2
        static constexpr int left_front_in3 = PIN_47_DIG; // purple = in3
        static constexpr int left_front_in4 = PIN_46_PWM; // blue = in4 (this does not require a PWM pin)
        static constexpr int left_rear_enA = PIN_45_PWM; // blue = enA
        static constexpr int left_front_enB = PIN_44_PWM; // green = enB

        // Right side
        static constexpr int right_front_in1 = PIN_43_DIG; // yellow = in1
        static constexpr int right_front_in2 = PIN_42_DIG; // orange = in2
        static constexpr int right_rear_in3 = PIN_41_DIG; // purple = in3
        static constexpr int right_rear_in4 = PIN_40_DIG; // gray = in4
        static constexpr int right_front_enA = PIN_5_PWM; // gray = enA
        static constexpr int right_rear_enB = PIN_6_PWM; // green = enB

        static constexpr int DISABLED_LED = 99;

        static constexpr int RIGHT_LED_FORWARD = PIN_22_DIG; // red led right
        static constexpr int RIGHT_LED_REVERSE = PIN_24_DIG; // green led right
        static constexpr int LEFT_LED_FORWARD = PIN_26_DIG; // red led left
        static constexpr int LEFT_LED_REVERSE = PIN_28_DIG; // green led left
        MotorPlugin motors[MOTOR_COUNT] =
        { MotorPlugin(PluginId::MOTOR_RIGHT_FRONT_PLUGIN, MotorLocation::RIGHT_FRONT, right_front_enA, right_front_in2,
                right_front_in1, RIGHT_LED_FORWARD, RIGHT_LED_REVERSE),
        //
                MotorPlugin(PluginId::MOTOR_LEFT_FRONT_PLUGIN, MotorLocation::LEFT_FRONT, left_front_enB,
                        left_front_in3, left_front_in4, LEFT_LED_FORWARD, LEFT_LED_REVERSE),
                //
                MotorPlugin(PluginId::MOTOR_RIGHT_REAR_PLUGIN, MotorLocation::RIGHT_REAR, right_rear_enB,
                        right_rear_in3, right_rear_in4, DISABLED_LED, DISABLED_LED),
                //
                MotorPlugin(PluginId::MOTOR_LEFT_REAR_PLUGIN, MotorLocation::LEFT_REAR, left_rear_enA, left_rear_in2,
                        left_rear_in1, DISABLED_LED, DISABLED_LED) };

        static constexpr int schedule_size = 70;
        PluginId schedule[schedule_size];
        Mode mode = Mode::COMMAND_MODE;

        void schedule_cycle ();
        void simple_cycle ();
        void command_cycle ();
        void help_command ();
        Plugin* get_plugin (const PluginId id) const;
        int get_actual_interval (const int cycle, const PluginId id) const;
};

