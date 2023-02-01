/*
 * speed_counter.cpp
 *
 *  Created on: Jan 9, 2023
 *      Author: cre
 */

#include "Arduino.h"
#include "speed_counter.hpp"
#include "board_pins.hpp"

static constexpr int front_right_speed_counter_pin = PIN_2_INT4_PWM;
static constexpr int front_left_speed_counter_pin = PIN_3_INT5_PWM;

static constexpr int rear_left_speed_counter_pin = PIN_18_INT3_TX1;
static constexpr int rear_right_speed_counter_pin = PIN_19_INT2_RX1;

static unsigned long speed_counter_right_front = 0;
static unsigned long speed_counter_left_front = 0;
static unsigned long speed_counter_right_rear = 0;
static unsigned long speed_counter_left_rear = 0;

static unsigned long speed_counter_right_front_micros = 0;
static unsigned long speed_counter_left_front_micros = 0;
static unsigned long speed_counter_right_rear_micros = 0;
static unsigned long speed_counter_left_rear_micros = 0;

static unsigned long right_front_limit = 0;
static unsigned long left_front_limit = 0;
static unsigned long right_rear_limit = 0;
static unsigned long left_rear_limit = 0;

static int right_front_pin = 0;
static int left_front_pin = 0;
static int right_rear_pin = 0;
static int left_rear_pin = 0;

static int right_front_value = 0;
static int left_front_value = 0;
static int right_rear_value = 0;
static int left_rear_value = 0;

static void isr_right_front ()
{
    speed_counter_right_front++;
    speed_counter_right_front_micros = micros();
    // Limit value zero will never match since speed counter increments before comparison.
    if (speed_counter_right_front == right_front_limit)
    {
        analogWrite(right_front_pin, right_front_value);
    }
}

static void isr_left_front ()
{
    speed_counter_left_front++;
    speed_counter_left_front_micros = micros();
    // Limit value zero will never match since speed counter increments before comparison.
    if (speed_counter_left_front == left_front_limit)
    {
        analogWrite(left_front_pin, left_front_value);
    }
}

static void isr_right_rear ()
{
    speed_counter_right_rear++;
    speed_counter_right_rear_micros = micros();
    // Limit value zero will never match since speed counter increments before comparison.
    if (speed_counter_right_rear == right_rear_limit)
    {
        analogWrite(right_rear_pin, right_rear_value);
    }
}

static void isr_left_rear ()
{
    speed_counter_left_rear++;
    speed_counter_left_rear_micros = micros();
    // Limit value zero will never match since speed counter increments before comparison.
    if (speed_counter_left_rear == left_rear_limit)
    {
        analogWrite(left_rear_pin, left_rear_value);
    }
}

void setup_speed_counter ()
{
    attachInterrupt(digitalPinToInterrupt(front_right_speed_counter_pin), isr_right_front, RISING);
    attachInterrupt(digitalPinToInterrupt(front_left_speed_counter_pin), isr_left_front, RISING);
    attachInterrupt(digitalPinToInterrupt(rear_right_speed_counter_pin), isr_right_rear, RISING);
    attachInterrupt(digitalPinToInterrupt(rear_left_speed_counter_pin), isr_left_rear, RISING);
}

unsigned long get_speed_counter_value (const MotorLocation location)
{
    switch (location)
    {
        case RIGHT_FRONT:
            return speed_counter_right_front;
        case LEFT_FRONT:
            return speed_counter_left_front;
        case RIGHT_REAR:
            return speed_counter_right_rear;
        case LEFT_REAR:
            return speed_counter_left_rear;
        default:
            return 0;
    }
}

unsigned long get_speed_counter_micros (const MotorLocation location)
{
    switch (location)
    {
        case RIGHT_FRONT:
            return speed_counter_right_front_micros;
        case LEFT_FRONT:
            return speed_counter_left_front_micros;
        case RIGHT_REAR:
            return speed_counter_right_rear_micros;
        case LEFT_REAR:
            return speed_counter_left_rear_micros;
        default:
            return 0;
    }
}

void set_speed_counter_limit (const MotorLocation location, const unsigned long limit, const int pin, const int value)
{
    switch (location)
    {
        case RIGHT_FRONT:
            right_front_limit = 0;
            right_front_pin = pin;
            right_front_value = value;
            right_front_limit = limit;
            break;
        case LEFT_FRONT:
            left_front_limit = 0;
            left_front_pin = pin;
            left_front_value = value;
            left_front_limit = limit;
            break;
        case RIGHT_REAR:
            right_rear_limit = 0;
            right_rear_pin = pin;
            right_rear_value = value;
            right_rear_limit = limit;
            break;
        case LEFT_REAR:
            left_rear_limit = 0;
            left_rear_pin = pin;
            left_rear_value = value;
            left_rear_limit = limit;
            break;
    }
}

void set_speed_counter_delta_limit (const MotorLocation location, const unsigned long delta, const int pin,
        const int value)
{
    switch (location)
    {
        case RIGHT_FRONT:
            right_front_limit = 0;
            right_front_pin = pin;
            right_front_value = value;
            right_front_limit = speed_counter_right_front + delta;
            break;
        case LEFT_FRONT:
            left_front_limit = 0;
            left_front_pin = pin;
            left_front_value = value;
            left_front_limit = speed_counter_left_front + delta;
            break;
        case RIGHT_REAR:
            right_rear_limit = 0;
            right_rear_pin = pin;
            right_rear_value = value;
            right_rear_limit = speed_counter_right_rear + delta;
            break;
        case LEFT_REAR:
            left_rear_limit = 0;
            left_rear_pin = pin;
            left_rear_value = value;
            left_rear_limit = speed_counter_left_rear + delta;
            break;
    }
}
