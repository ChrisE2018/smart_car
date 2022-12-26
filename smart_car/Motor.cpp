/*
 * Motor.cpp
 *
 *  Created on: Dec 23, 2022
 *      Author: cre
 */

#include "Motor.hpp"

std::ostream& operator<< (std::ostream &lhs, MotorDirection direction)
{
    switch (direction)
    {
        case STOP:
            lhs << "STOP";
            break;
        case FORWARD:
            lhs << "FORWARD";
            break;
        case REVERSE:
            lhs << "REVERSE";
            break;
    }
    return lhs;
}

std::ostream& operator<< (std::ostream &lhs, MotorLocation location)
{
    switch (location)
    {
        case RIGHT:
            lhs << "RIGHT";
            break;
        case LEFT:
            lhs << "LEFT";
            break;
    }
    return lhs;
}

void Motor::setup ()
{
    pinMode(enable_pin, OUTPUT);
    pinMode(forward_pin, OUTPUT);
    pinMode(reverse_pin, OUTPUT);
    pinMode(forward_led, OUTPUT);
    pinMode(reverse_led, OUTPUT);
    drive_stop();
}

void Motor::led_demo (int duration)
{
    digitalWrite(forward_led, HIGH);
    delay(duration);
    digitalWrite(forward_led, LOW);
    delay(duration);
    digitalWrite(reverse_led, HIGH);
    delay(duration);
    digitalWrite(reverse_led, LOW);
    delay(duration);
}

void Motor::drive_forward (int speed)
{
    digitalWrite(forward_led, HIGH);
    digitalWrite(reverse_led, LOW);
    digitalWrite(reverse_pin, LOW);
    digitalWrite(forward_pin, HIGH);
    analogWrite(enable_pin, speed);
}

void Motor::drive_reverse (int speed)
{
    digitalWrite(forward_led, LOW);
    digitalWrite(reverse_led, HIGH);
    digitalWrite(forward_pin, LOW);
    digitalWrite(reverse_pin, HIGH);
    analogWrite(enable_pin, speed);
}

void Motor::drive_stop ()
{
    digitalWrite(forward_led, LOW);
    digitalWrite(reverse_led, LOW);
    digitalWrite(forward_pin, LOW);
    digitalWrite(reverse_pin, LOW);
    analogWrite(enable_pin, LOW);
}
