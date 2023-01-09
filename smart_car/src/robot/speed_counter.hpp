/*
 * speed_counter.hpp
 *
 *  Created on: Jan 9, 2023
 *      Author: cre
 */

#pragma once

const int right_speed_counter_pin = 2;
const int left_speed_counter_pin = 3;

void setup_speed_counter ();
unsigned long get_right_speed_counter ();
unsigned long get_left_speed_counter ();
