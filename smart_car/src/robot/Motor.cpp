/*
 * Motor.cpp
 *
 *  Created on: Dec 23, 2022
 *      Author: cre
 */

#include "Motor.hpp"
#include "smart_car.hpp"

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

void Motor::led_demo (const int duration) const
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

void Motor::drive_forward (const int _speed)
{
    if (direction != FORWARD || speed != _speed)
    {
        speed = _speed;
        direction = FORWARD;
        digitalWrite(forward_led, HIGH);
        digitalWrite(reverse_led, LOW);
        digitalWrite(reverse_pin, LOW);
        digitalWrite(forward_pin, HIGH);
        analogWrite(enable_pin, _speed);
        cout << "drive " << location << " " << direction << " at " << speed << std::endl;
    }
}

void Motor::drive_reverse (const int _speed)
{
    if (direction != REVERSE || speed != _speed)
    {
        speed = _speed;
        direction = REVERSE;
        digitalWrite(forward_led, LOW);
        digitalWrite(reverse_led, HIGH);
        digitalWrite(forward_pin, LOW);
        digitalWrite(reverse_pin, HIGH);
        analogWrite(enable_pin, _speed);
        cout << "drive " << location << " " << direction << " at " << speed << std::endl;
    }
}

void Motor::drive_stop ()
{
    if (direction != STOP || speed != 0)
    {
        direction = STOP;
        speed = 0;
        digitalWrite(forward_led, LOW);
        digitalWrite(reverse_led, LOW);
        digitalWrite(forward_pin, LOW);
        digitalWrite(reverse_pin, LOW);
        analogWrite(enable_pin, LOW);
        cout << "drive " << location << " " << direction << " at " << speed << std::endl;
    }
}

MotorDirection Motor::get_direction () const
{
    return direction;
}

int Motor::get_speed () const
{
    return speed;
}

float Motor::get_velocity () const
{
    if (direction == REVERSE)
    {
        return -(speed / 256.0);
    }
    else
    {
        return (speed / 256.0);
    }
}
