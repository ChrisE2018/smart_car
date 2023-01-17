/*
 * Motor.cpp
 *
 *  Created on: Dec 23, 2022
 *      Author: cre
 */

#include "MotorPlugin.hpp"

#include "../logging/RobotAppender.hpp"
#include "smart_car.hpp"
#include "../robot/speed_counter.hpp"

#include "../logging/Logger.hpp"

static Logger logger(__FILE__, Level::info);

const std::string stringify (const MotorDirection direction)
{
    switch (direction)
    {
        case MotorDirection::STOP:
            return "STOP";
        case MotorDirection::FORWARD:
            return "FORWARD";
        case MotorDirection::REVERSE:
            return "REVERSE";
        default:
            return "?MotorDirection?";
    }
}

std::ostream& operator<< (std::ostream &lhs, const MotorDirection direction)
{
    lhs << stringify(direction);
    return lhs;
}

const std::string stringify (const MotorLocation location)
{
    switch (location)
    {
        case MotorLocation::RIGHT_FRONT:
            return "RIGHT_FRONT";
        case MotorLocation::LEFT_FRONT:
            return "LEFT_FRONT";
        case MotorLocation::RIGHT_REAR:
            return "RIGHT_REAR";
        case MotorLocation::LEFT_REAR:
            return "LEFT_REAR";
        default:
            return "?MotorLocation?";
    }
}
std::ostream& operator<< (std::ostream &lhs, const MotorLocation location)
{
    lhs << stringify(location);
    return lhs;
}

std::ostream& operator<< (std::ostream &lhs, const MotorPlugin &motor)
{
    return lhs << "#[motor " << motor.location << " " << motor.speed << "]";
}

MotorPlugin::MotorPlugin (const PluginId id, const MotorLocation location, int enable, int forward, int reverse,
        int forward_led, int reverse_led) :
        location(location), enable_pin(enable), forward_pin(forward), reverse_pin(reverse), forward_led(forward_led), reverse_led(
                reverse_led), Plugin(id)
{
}

bool MotorPlugin::setup ()
{
    pinMode(enable_pin, OUTPUT);
    pinMode(forward_pin, OUTPUT);
    pinMode(reverse_pin, OUTPUT);
    if (forward_led != DISABLED_LED)
    {
        pinMode(forward_led, OUTPUT);
    }
    if (reverse_led != DISABLED_LED)
    {
        pinMode(reverse_led, OUTPUT);
    }
    set_speed(0);
    return true;
}

bool MotorPlugin::is_cyclic () const
{
    return false;
}

void MotorPlugin::led_demo (const int duration) const
{
    if (forward_led != DISABLED_LED)
    {
        digitalWrite(forward_led, HIGH);
        delay(duration);
        digitalWrite(forward_led, LOW);
        delay(duration);
    }
    if (reverse_led != DISABLED_LED)
    {
        digitalWrite(reverse_led, HIGH);
        delay(duration);
        digitalWrite(reverse_led, LOW);
        delay(duration);
    }
}

MotorLocation MotorPlugin::get_location () const
{
    return location;
}

MotorDirection MotorPlugin::get_direction () const
{
    return direction;
}

int MotorPlugin::get_speed () const
{
    return speed;
}

void MotorPlugin::set_speed (const int speed)
{
    if (speed > 0)
    {
        drive_forward(std::min(SPEED_FULL, speed));
    }
    else if (speed < 0)
    {
        drive_reverse(std::max(-SPEED_FULL, speed));
    }
    else
    {
        drive_zero_speed();
    }
}

void MotorPlugin::set_limit (const unsigned long limit,  const int value)
{
    set_speed_counter_limit(location, limit, enable_pin, value);
}

void MotorPlugin::set_delta_limit (const unsigned long delta, const int value)
{
    set_speed_counter_delta_limit(location, delta, enable_pin, value);
}

void MotorPlugin::drive_forward (const int _speed)
{
    if (direction != MotorDirection::FORWARD || speed != _speed)
    {
        speed = _speed;
        direction = MotorDirection::FORWARD;
        if (forward_led != DISABLED_LED)
        {
            digitalWrite(forward_led, HIGH);
        }
        if (reverse_led != DISABLED_LED)
        {
            digitalWrite(reverse_led, LOW);
        }
        digitalWrite(reverse_pin, LOW);
        digitalWrite(forward_pin, HIGH);
        analogWrite(enable_pin, _speed);
        cout << location << "_" << direction << ": " << speed << " " << location
//                << "_mps: " << measured_velocity
//                << " cve: " << cumulative_velocity_error
                << std::endl;
    }
}

void MotorPlugin::drive_reverse (const int _speed)
{
    if (direction != MotorDirection::REVERSE || speed != _speed)
    {
        speed = _speed;
        direction = MotorDirection::REVERSE;
        if (forward_led != DISABLED_LED)
        {
            digitalWrite(forward_led, LOW);
        }
        if (reverse_led != DISABLED_LED)
        {
            digitalWrite(reverse_led, HIGH);
        }
        digitalWrite(forward_pin, LOW);
        digitalWrite(reverse_pin, HIGH);
        analogWrite(enable_pin, -speed);
        cout << location << "_" << direction << ": " << speed << " " << location
//                << "_mps: " << measured_velocity
//                << " cve: " << cumulative_velocity_error
                << std::endl;
    }
}

void MotorPlugin::drive_zero_speed ()
{
    if (direction != MotorDirection::STOP || speed != 0)
    {
        direction = MotorDirection::STOP;
        speed = 0;
        if (forward_led != DISABLED_LED)
        {
            digitalWrite(forward_led, LOW);
        }
        if (reverse_led != DISABLED_LED)
        {
            digitalWrite(reverse_led, LOW);
        }
        digitalWrite(forward_pin, LOW);
        digitalWrite(reverse_pin, LOW);
        analogWrite(enable_pin, LOW);
        cout << "drive " << location << " " << direction << " at " << speed << " power "
//                << measured_velocity << " mps"
//                << " cve: " << cumulative_velocity_error
                << std::endl;
    }
}
