/*
 * Motor.hpp
 *
 *  Created on: Dec 23, 2022
 *      Author: cre
 */

#pragma once

#include "Arduino.h"

const int MOTOR_COUNT = 2;

class Motor
{
        int enable_pin;
        int forward_pin;
        int reverse_pin;
        int forward_led;
        int reverse_led;
    public:
        Motor (int enable, int forward, int reverse, int f_led, int r_led) :
                enable_pin(enable), forward_pin(forward), reverse_pin(reverse), forward_led(f_led), reverse_led(
                        r_led)
        {

        }

        void setup ();

        void led_demo (int duration);

        void drive_forward (int speed);

        void drive_reverse (int speed);

        void drive_stop ();
};

