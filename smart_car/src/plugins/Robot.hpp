/*
 * Robot.hpp
 *
 *  Created on: Jan 23, 2023
 *      Author: cre
 */

#pragma once

#include "src/robot/Car.hpp"
#include "src/robot/speed_counter.hpp"
#include "src/robot/heap.hpp"
#include "src/plugins/Plug.hpp"
#include "src/plugins/ClockPlugin.hpp"
#include "src/plugins/DrivePlugin.hpp"
#include "src/plugins/GoalPlugin.hpp"
#include "src/plugins/KalmanPlugin.hpp"
#include "src/plugins/MpuPlugin.hpp"
#include "src/plugins/OdomPlugin.hpp"
#include "src/plugins/UltrasoundPlugin.hpp"
#include "src/plugins/WallPlugin.hpp"

extern Plug plug;
extern Car car;
extern ClockPlugin clock_plugin;
extern DrivePlugin forward_plugin;
extern DrivePlugin reverse_plugin;
extern GoalPlugin goal_plugin;
extern KalmanPlugin kalman_plugin;
extern WallPlugin wall_plugin;

void setup_robot ();

