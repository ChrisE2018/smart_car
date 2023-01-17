/*
 * speed_counter.hpp
 *
 *  Created on: Jan 9, 2023
 *      Author: cre
 */

#pragma once

#include "../plugins/MotorPlugin.hpp"

void setup_speed_counter ();

// 55 mm wheels = 0.55 meters
// 20 encoder slots per revolution
// meters-per-micro = PI * diameter / encoder_slots
constexpr double count_to_meters = M_PI * 0.055 / 20.0;

unsigned long get_speed_counter_value (const MotorLocation location);

void set_speed_counter_limit (const MotorLocation location, const unsigned long limit, const int pin, const int value);
void set_speed_counter_delta_limit (const MotorLocation location, const unsigned long delta, const int pin, const int value);
