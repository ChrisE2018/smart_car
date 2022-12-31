/*
 * Motor.hpp
 *
 *  Created on: Dec 23, 2022
 *      Author: cre
 */

#pragma once

#include "Arduino.h"
#include <ostream>

const int SPEED_FULL = 255;
const int SPEED_Q3 = 196;
const int SPEED_160 = 175;
const int SPEED_HALF = 128;
const int SPEED_Q1 = 64;
const int SPEED_CRAWL = 32;
const int SPEED_STOP = 0;

enum MotorDirection
{
    STOP, FORWARD, REVERSE
};

std::ostream& operator<< (std::ostream &lhs, MotorDirection direction);

enum MotorLocation
{
    RIGHT = 0, LEFT = 1
};

std::ostream& operator<< (std::ostream &lhs, MotorLocation location);

class Motor
{
    public:
        Motor (int enable, int forward, int reverse, int f_led, int r_led) : enable_pin(enable), forward_pin(
                forward), reverse_pin(reverse), forward_led(f_led), reverse_led(r_led)
        {
        }

        void setup ();
        void led_demo (const int duration) const;
        void drive_forward (const int speed);
        void drive_reverse (const int speed);
        void drive_stop ();
        MotorDirection get_direction () const;
        int get_speed () const;
        float get_velocity () const;

    private:
        int enable_pin;
        int forward_pin;
        int reverse_pin;
        int forward_led;
        int reverse_led;
        MotorDirection direction = STOP;
        int speed = 0;
};

