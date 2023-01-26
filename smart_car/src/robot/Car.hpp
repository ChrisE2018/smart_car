/*
 * Car.hpp
 *
 *  Created on: Dec 23, 2022
 *      Author: cre
 */

#pragma once

#include <arduino.h>
#include <vector>
#include "Logger.hpp"
#include "../plugins/MotorPlugin.hpp"
#include "../plugins/PidPlugin.hpp"
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
        void set_speed (const MotorLocation motor, const int speed);
        void set_right_speed (const int speed);
        void set_left_speed (const int speed);
        int get_drive_speed (const MotorLocation motor) const;
        float get_desired_velocity (const MotorLocation motor) const;
        void set_desired_velocity (const MotorLocation motor, const float velocity);
        float get_measured_velocity (const MotorLocation motor) const;
        float get_cumulative_velocity_error (const MotorLocation motor) const;

        ClockPlugin* get_clock_plugin () const;
        DrivePlugin* get_forward_plugin () const;
        GoalPlugin* get_goal_plugin () const;
        DrivePlugin* get_reverse_plugin () const;
        MpuPlugin* get_mpu_plugin () const;
        KalmanPlugin* get_kalman_plugin () const;
        OdomPlugin* get_odom_plugin () const;
        const PidPlugin& get_pid_plugin (const MotorLocation location) const;
        PidPlugin& get_pid_plugin (const MotorLocation location);
        UltrasoundPlugin* get_ultrasound_plugin () const;
        WallPlugin* get_wall_plugin () const;

    private:
        logging::Logger logger;
        unsigned long cycle_count = 0;
        unsigned long total_cycle_us = 0;
        unsigned long command_cycle_count = 0;
        unsigned long total_command_cycle_us = 0;

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
        std::vector<Plugin*> cyclic_plugins;

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
        { MotorPlugin(*this, PluginId::MOTOR_RIGHT_FRONT_PLUGIN, MotorLocation::RIGHT_FRONT, right_front_enA,
                right_front_in2, right_front_in1, RIGHT_LED_FORWARD, RIGHT_LED_REVERSE),
        //
                MotorPlugin(*this, PluginId::MOTOR_LEFT_FRONT_PLUGIN, MotorLocation::LEFT_FRONT, left_front_enB,
                        left_front_in3, left_front_in4, LEFT_LED_FORWARD, LEFT_LED_REVERSE),
                //
                MotorPlugin(*this, PluginId::MOTOR_RIGHT_REAR_PLUGIN, MotorLocation::RIGHT_REAR, right_rear_enB,
                        right_rear_in3, right_rear_in4, DISABLED_LED, DISABLED_LED),
                //
                MotorPlugin(*this, PluginId::MOTOR_LEFT_REAR_PLUGIN, MotorLocation::LEFT_REAR, left_rear_enA,
                        left_rear_in2, left_rear_in1, DISABLED_LED, DISABLED_LED) };

        PidPlugin pid_controls[MOTOR_COUNT] =
        { PidPlugin(PluginId::PID_RIGHT_FRONT_PLUGIN, motors[0]), PidPlugin(PluginId::PID_LEFT_FRONT_PLUGIN, motors[1]),
                PidPlugin(PluginId::PID_RIGHT_REAR_PLUGIN, motors[2]), PidPlugin(PluginId::PID_LEFT_REAR_PLUGIN,
                        motors[3]) };
        Mode mode = Mode::COMMAND_MODE;

        void command_cycle ();
        void help_command ();
        Plugin* get_plugin (const PluginId id) const;
};

