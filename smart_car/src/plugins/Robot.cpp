/*
 * Robot.cpp
 *
 *  Created on: Jan 23, 2023
 *      Author: cre
 */

#include "Robot.hpp"

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

static constexpr int RIGHT_LED_FORWARD = PIN_22_DIG; // red led right
static constexpr int RIGHT_LED_REVERSE = PIN_24_DIG; // green led right
static constexpr int LEFT_LED_FORWARD = PIN_26_DIG; // red led left
static constexpr int LEFT_LED_REVERSE = PIN_28_DIG; // green led left

Plug plug;

ClockPlugin clock_plugin;
DrivePlugin forward_plugin(PluginId::FORWARD_PLUGIN, 500, MotorDirection::FORWARD, MotorDirection::FORWARD);
DrivePlugin reverse_plugin(PluginId::REVERSE_PLUGIN, 500, MotorDirection::REVERSE, MotorDirection::REVERSE);
GoalPlugin goal_plugin;
MpuPlugin mpu_plugin;

KalmanPlugin kalman_plugin;
OdomPlugin odom_plugin;
UltrasoundPlugin ultrasound_plugin;
WallPlugin wall_plugin;

MotorPlugin motors[MOTOR_COUNT] =
{ MotorPlugin(PluginId::MOTOR_RIGHT_FRONT_PLUGIN, MotorLocation::RIGHT_FRONT, right_front_enA, right_front_in2,
        right_front_in1, RIGHT_LED_FORWARD, RIGHT_LED_REVERSE),
//
        MotorPlugin(PluginId::MOTOR_LEFT_FRONT_PLUGIN, MotorLocation::LEFT_FRONT, left_front_enB, left_front_in3,
                left_front_in4, LEFT_LED_FORWARD, LEFT_LED_REVERSE),
        //
        MotorPlugin(PluginId::MOTOR_RIGHT_REAR_PLUGIN, MotorLocation::RIGHT_REAR, right_rear_enB, right_rear_in3,
                right_rear_in4, DISABLED_LED, DISABLED_LED),
        //
        MotorPlugin(PluginId::MOTOR_LEFT_REAR_PLUGIN, MotorLocation::LEFT_REAR, left_rear_enA, left_rear_in2,
                left_rear_in1, DISABLED_LED, DISABLED_LED) };

PidPlugin pid_controls[MOTOR_COUNT] =
{ PidPlugin(PluginId::PID_RIGHT_FRONT_PLUGIN, motors[0]), PidPlugin(PluginId::PID_LEFT_FRONT_PLUGIN, motors[1]),
        PidPlugin(PluginId::PID_RIGHT_REAR_PLUGIN, motors[2]), PidPlugin(PluginId::PID_LEFT_REAR_PLUGIN, motors[3]) };
Car car;

void setup_robot ()
{
    plug.add_plugin(&clock_plugin);
    plug.add_plugin(&forward_plugin);
    plug.add_plugin(&reverse_plugin);
    plug.add_plugin(&goal_plugin);
    plug.add_plugin(&mpu_plugin);
    plug.add_plugin(&kalman_plugin);
    plug.add_plugin(&odom_plugin);
    plug.add_plugin(&ultrasound_plugin);
    plug.add_plugin(&wall_plugin);
    for (int i = 0; i < MOTOR_COUNT; i++)
    {

        plug.add_plugin(&motors[i]);
        plug.add_plugin(&pid_controls[i]);
    }
}

MotorPlugin& get_motor (const MotorLocation motor)
{
    return motors[static_cast<int>(motor)];
}

PidPlugin& get_pid_plugin (const MotorLocation location)
{
    return pid_controls[static_cast<int>(location)];
}
