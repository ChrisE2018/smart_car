/*
 * Robot.hpp
 *
 *  Created on: Jan 23, 2023
 *      Author: cre
 */

#pragma once

#include "src/robot/Car.hpp"
#include "src/robot/speed_counter.hpp"

#include "Plug.hpp"
#include "Plugin.hpp"
#include "ClockPlugin.hpp"
#include "DrivePlugin.hpp"
#include "GoalPlugin.hpp"
#include "KalmanPlugin.hpp"
#include "MotorPlugin.hpp"
#include "MpuPlugin.hpp"
#include "PidPlugin.hpp"
#include "OdomPlugin.hpp"
#include "UltrasoundPlugin.hpp"
#include "WallPlugin.hpp"

const int MOTOR_COUNT = 4;

extern Plug plug;
extern Car car;
extern ClockPlugin clock_plugin;
extern DrivePlugin forward_plugin;
extern DrivePlugin reverse_plugin;
extern GoalPlugin goal_plugin;
extern KalmanPlugin kalman_plugin;
extern OdomPlugin odom_plugin;
extern MpuPlugin mpu_plugin;
extern UltrasoundPlugin ultrasound_plugin;
extern WallPlugin wall_plugin;
extern MotorPlugin motors[MOTOR_COUNT];
extern PidPlugin pid_controls[MOTOR_COUNT];

void setup_robot ();

MotorPlugin& get_motor(const MotorLocation location);
PidPlugin& get_pid_plugin (const MotorLocation location);

