/*
 * Copyright (c) 2023 by Christopher Eliot.
 *
 * speed_counter.hpp
 *
 *  Created on: Jan 9, 2023
 *      Author: cre
 */

#pragma once

#include "../plugins/MotorPlugin.hpp"

void setup_speed_counter ();

// 55 mm wheels = 0.55 meters
// Wheel perimeter = 172.78759594743863 mm
// 20 encoder slots per revolution
// meters-per-micro = PI * diameter / encoder_slots
// Distance moved per wheel encoder tick = 8.639379797371932 mm = 0.008639379797371931 m
constexpr double wheel_diameter = 0.055;

// There are 20 wheel encoder slots. Currently, interrupts are only generating on RISING signals.
// It should be possible to get 40 signals per revolution by generating interrupts on
// both RISING and FALLING signals.
constexpr double encoder_slots = 20.0;
constexpr double count_to_meters = M_PI * wheel_diameter / encoder_slots;

constexpr double wheel_spacing_lengthwise = 0.11; // meters
constexpr double wheel_spacing_sideways = 0.13; // meters

unsigned long get_speed_counter_value (const MotorLocation location);
unsigned long get_speed_counter_micros (const MotorLocation location);

void set_speed_counter_limit (const MotorLocation location, const unsigned long limit, const int pin, const int value);
void set_speed_counter_delta_limit (const MotorLocation location, const unsigned long delta, const int pin, const int value);
