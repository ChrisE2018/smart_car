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

static void isr_right_front ()
{
    speed_counter_right_front++;
}

static void isr_left_front ()
{
    speed_counter_left_front++;
}

static void isr_right_rear ()
{
    speed_counter_right_rear++;
}

static void isr_left_rear ()
{
    speed_counter_left_rear++;
}

void setup_speed_counter ()
{
    attachInterrupt(digitalPinToInterrupt(front_right_speed_counter_pin), isr_right_front, RISING);
    attachInterrupt(digitalPinToInterrupt(front_left_speed_counter_pin), isr_left_front, RISING);
    attachInterrupt(digitalPinToInterrupt(rear_right_speed_counter_pin), isr_right_rear, RISING);
    attachInterrupt(digitalPinToInterrupt(rear_left_speed_counter_pin), isr_left_rear, RISING);
}

unsigned long get_right_front_speed_counter ()
{
    return speed_counter_right_front;
}

unsigned long get_left_front_speed_counter ()
{
    return speed_counter_left_front;
}

unsigned long get_right_rear_speed_counter ()
{
    return speed_counter_right_rear;
}

unsigned long get_left_rear_speed_counter ()
{
    return speed_counter_left_rear;
}
