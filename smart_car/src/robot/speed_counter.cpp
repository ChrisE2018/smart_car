/*
 * speed_counter.cpp
 *
 *  Created on: Jan 9, 2023
 *      Author: cre
 */

#include "Arduino.h"
#include "speed_counter.hpp"

static constexpr int right_speed_counter_pin = 2;
static constexpr int left_speed_counter_pin = 3;

static unsigned long speed_counter_right = 0;
static unsigned long speed_counter_left = 0;

static void isr_right ()
{
    speed_counter_right++;
}

static void isr_left ()
{
    speed_counter_left++;
}

void setup_speed_counter ()
{
    attachInterrupt(digitalPinToInterrupt(right_speed_counter_pin), isr_right, RISING);
    attachInterrupt(digitalPinToInterrupt(left_speed_counter_pin), isr_left, RISING);
}

unsigned long get_right_speed_counter ()
{
    return speed_counter_right;
}

unsigned long get_left_speed_counter ()
{
    return speed_counter_left;
}
