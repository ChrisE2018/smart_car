/*
 * Car.hpp
 *
 *  Created on: Dec 23, 2022
 *      Author: cre
 */

#pragma once

#include "Motor.hpp"
#include "Parser.hpp"

const int MOTOR_COUNT = 2;

class Car : public Executor
{
    public:
        Car ();

        void setup ();
        void demo_drive_leds ();
        void all_stop();
        void drive_stop(int motor);
        void drive_forward(int motor, int speed);
        void drive_reverse(int motor, int speed);
        void forward (const int speed, const int duration);
        void reverse (const int speed, const int duration);
        void turn_clockwise (const int speed, const int duration);
        void turn_counterclockwise (const int speed, const int duration);

    private:
        // pins
        // 2 yellow = in1
        // 3 orange = in2
        // 4 purple = in3
        // 5 blue = in4
        // 6 blue = enA
        // 7 green = enB

        // LED_0 = 22;  // red led
        // LED_1 = 24;  // green led
        // LED_2 = 26;  // red led
        // LED_3 = 28;  // green led
        Motor motors[MOTOR_COUNT] =
        { Motor(6, 2, 3, 26, 28), Motor(7, 5, 4, 22, 24) };

        Parser parser;
};

