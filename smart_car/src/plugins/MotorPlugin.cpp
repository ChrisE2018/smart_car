/*
 * Motor.cpp
 *
 *  Created on: Dec 23, 2022
 *      Author: cre
 */

#include "MotorPlugin.hpp"

#include "../robot/speed_counter.hpp"
#include "smart_car.hpp"

std::ostream& operator<< (std::ostream &lhs, const MotorDirection direction)
{
    switch (direction)
    {
        case MotorDirection::STOP:
            lhs << "STOP";
            break;
        case MotorDirection::FORWARD:
            lhs << "FORWARD";
            break;
        case MotorDirection::REVERSE:
            lhs << "REVERSE";
            break;
    }
    return lhs;
}

std::ostream& operator<< (std::ostream &lhs, const MotorLocation location)
{
    switch (location)
    {
        case MotorLocation::RIGHT:
            lhs << "RIGHT";
            break;
        case MotorLocation::LEFT:
            lhs << "LEFT";
            break;
    }
    return lhs;
}

std::ostream& operator<< (std::ostream &lhs, const MotorPlugin &motor)
{
    return lhs << "#[motor " << motor.location << " " << motor.measured_velocity << " m/s err="
            << motor.velocity_error << "]";
}

bool MotorPlugin::setup ()
{
    pinMode(enable_pin, OUTPUT);
    pinMode(forward_pin, OUTPUT);
    pinMode(reverse_pin, OUTPUT);
    pinMode(forward_led, OUTPUT);
    pinMode(reverse_led, OUTPUT);
    drive_stop();
    return true;
}

void MotorPlugin::led_demo (const int duration) const
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

void MotorPlugin::drive_forward (const int _speed)
{
    if (direction != MotorDirection::FORWARD || speed != _speed)
    {
        speed = _speed;
        direction = MotorDirection::FORWARD;
        digitalWrite(forward_led, HIGH);
        digitalWrite(reverse_led, LOW);
        digitalWrite(reverse_pin, LOW);
        digitalWrite(forward_pin, HIGH);
        analogWrite(enable_pin, _speed);
        cout << "drive " << location << " " << direction << " at " << speed << " mps "
                << measured_velocity << std::endl;
    }
}

void MotorPlugin::drive_reverse (const int _speed)
{
    if (direction != MotorDirection::REVERSE || speed != _speed)
    {
        speed = _speed;
        direction = MotorDirection::REVERSE;
        digitalWrite(forward_led, LOW);
        digitalWrite(reverse_led, HIGH);
        digitalWrite(forward_pin, LOW);
        digitalWrite(reverse_pin, HIGH);
        analogWrite(enable_pin, _speed);
        cout << "drive " << location << " " << direction << " at " << speed << " mps "
                << measured_velocity << std::endl;
    }
}

void MotorPlugin::drive_stop ()
{
    if (direction != MotorDirection::STOP || speed != 0)
    {
        direction = MotorDirection::STOP;
        speed = 0;
        desired_velocity = 0;
        cumulative_velocity_error = 0;
        cumulative_error_seconds = 0;
        digitalWrite(forward_led, LOW);
        digitalWrite(reverse_led, LOW);
        digitalWrite(forward_pin, LOW);
        digitalWrite(reverse_pin, LOW);
        analogWrite(enable_pin, LOW);
        cout << "drive " << location << " " << direction << " at " << speed << " mps "
                << measured_velocity << std::endl;
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
        drive_reverse(std::min(SPEED_FULL, -speed));
    }
    else
    {
        drive_stop();
    }
}

float MotorPlugin::get_velocity () const
{
    if (direction == MotorDirection::REVERSE)
    {
        return -(speed / 256.0);
    }
    else
    {
        return (speed / 256.0);
    }
}

unsigned long MotorPlugin::get_speed_counter () const
{
    return speed_counter;
}

void MotorPlugin::set_desired_velocity (const float _desired_velocity)
{
    desired_velocity = _desired_velocity;
    cumulative_velocity_error = 0;
    cumulative_error_seconds = 0;
}

float MotorPlugin::get_measured_velocity () const
{
    return measured_velocity;
}

float MotorPlugin::get_velocity_error () const
{
    return velocity_error;
}

int MotorPlugin::get_preferred_interval () const
{
    return 1;
}

int MotorPlugin::get_expected_ms () const
{
    return 1;
}

void MotorPlugin::cycle ()
{
    const unsigned long now = millis();
    if (now - last_cycle_ms > 100)
    {
        const float delta_seconds = (now - last_cycle_ms) * 0.001;
        const unsigned long previous_speed_counter = speed_counter;
        const float previous_velocity_error = velocity_error;
        speed_counter =
                (location == MotorLocation::RIGHT) ? get_right_speed_counter() :
                                                     get_left_speed_counter();
        measured_velocity = (speed_counter - previous_speed_counter) * count_to_meters_per_second
                / delta_seconds;
        velocity_error = desired_velocity - measured_velocity;
        const float velocity_error_rate = (velocity_error - previous_velocity_error)
                / delta_seconds;
        cumulative_velocity_error += velocity_error * delta_seconds;
        cumulative_error_seconds += delta_seconds;
        last_cycle_ms = now;
        const float control = k0
                * (velocity_error + k1 * cumulative_velocity_error + k2 * velocity_error_rate);

        set_speed(control);
//        if (velocity_error != 0)
//        {
//            cout << "ds: " << delta_seconds << " ve: " << velocity_error << " cve: "
//                    << cumulative_velocity_error << std::endl;
//        }
    }
}
