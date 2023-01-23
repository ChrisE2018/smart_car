/*
 * Robot.cpp
 *
 *  Created on: Jan 23, 2023
 *      Author: cre
 */

#include "Robot.hpp"

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

Car car;

void setup_robot()
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
}

