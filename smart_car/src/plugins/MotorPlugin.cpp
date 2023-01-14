/*
 * Motor.cpp
 *
 *  Created on: Dec 23, 2022
 *      Author: cre
 */

#include "MotorPlugin.hpp"

#include "../robot/speed_counter.hpp"
#include "../logging/RobotAppender.hpp"
#include "smart_car.hpp"

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
        case MotorLocation::RIGHT:
            return "RIGHT";
        case MotorLocation::LEFT:
            return "LEFT";
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
    return lhs << "#[motor " << motor.location << " " << motor.measured_velocity << " m/s err=" << motor.velocity_error
            << "]";
}

bool MotorPlugin::setup ()
{
    pinMode(enable_pin, OUTPUT);
    pinMode(forward_pin, OUTPUT);
    pinMode(reverse_pin, OUTPUT);
    pinMode(forward_led, OUTPUT);
    pinMode(reverse_led, OUTPUT);
    drive_stop();
    logger.data() << F("k0: ") << k0 << F(" k1: ") << k1 << F(" k2: ") << k2 << F(" k3: ") << k3 << F(" k4: ") << k4
            << std::endl;
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
        cout << location << "_" << direction << ": " << speed << " mps: " << measured_velocity << std::endl;
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
        cout << location << "_" << direction << ": " << speed << " mps: " << measured_velocity << std::endl;
    }
}

void MotorPlugin::drive_stop ()
{
    drive_zero_speed();
    auto_velocity = false;
}

void MotorPlugin::drive_zero_speed ()
{
    if (direction != MotorDirection::STOP || speed != 0 || desired_velocity != 0 || cumulative_velocity_error != 0)
    {
        direction = MotorDirection::STOP;
        speed = 0;
        desired_velocity = 0;
        cumulative_velocity_error = 0;
        digitalWrite(forward_led, LOW);
        digitalWrite(reverse_led, LOW);
        digitalWrite(forward_pin, LOW);
        digitalWrite(reverse_pin, LOW);
        analogWrite(enable_pin, LOW);
        cout << "drive " << location << " " << direction << " at " << speed << " power " << measured_velocity << " mps"
                << std::endl;
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
        drive_zero_speed();
    }
}

unsigned long MotorPlugin::get_speed_counter () const
{
    return speed_counter;
}

float MotorPlugin::get_measured_velocity () const
{
    return measured_velocity;
}

float MotorPlugin::get_desired_velocity () const
{
    return desired_velocity;
}

void MotorPlugin::set_desired_velocity (const float _desired_velocity)
{
    auto_velocity = true;
    if (desired_velocity != _desired_velocity)
    {
        desired_velocity = _desired_velocity;
        cumulative_velocity_error = 0;
    }
}

void MotorPlugin::cancel_auto_velocity ()
{
    auto_velocity = false;
}

float MotorPlugin::get_velocity_error () const
{
    return velocity_error;
}

int MotorPlugin::get_preferred_interval () const
{
    return 100;
}

int MotorPlugin::get_expected_us () const
{
    return 25000;
}

void MotorPlugin::cycle ()
{
    const unsigned long now = millis();
    if (now - last_cycle_ms > 100)
    {
        const float delta_seconds = (now - last_cycle_ms) * 0.001;
        const unsigned long previous_speed_counter = speed_counter;
        speed_counter = (location == MotorLocation::RIGHT) ? get_right_speed_counter() : get_left_speed_counter();
        const double measured_distance = (speed_counter - previous_speed_counter) * count_to_meters_per_second;
        measured_velocity = measured_distance / delta_seconds;
        if (measured_velocity > 0)
        {
            logger.data() << F("/motor/") << stringify(location).c_str() << F("/delta_seconds,") << delta_seconds
                    << F(",speed_counter,") << speed_counter << F(",measured_velocity,") << measured_velocity
                    << F(",measured_distance,") << measured_distance << std::endl;
        }
        if (auto_velocity)
        {
            const float previous_velocity_error = velocity_error;
            velocity_error = desired_velocity - measured_velocity;
            const float velocity_error_rate = (velocity_error - previous_velocity_error) / delta_seconds;
            cumulative_velocity_error += velocity_error * delta_seconds;
            const float smallness = std::max(0.0f, k4 - abs(measured_velocity));
            const float control = k0
                    * (velocity_error + k1 * cumulative_velocity_error + k2 * velocity_error_rate
                            + smallness * velocity_error * k3);
            set_speed(control);
            logger.data() << F("/motor/") << stringify(location).c_str() << F("/velocity_error,") << velocity_error
                    << F(",cumulative_velocity_error,") << cumulative_velocity_error << F(",velocity_error_rate,")
                    << velocity_error_rate << F(",smallness,") << smallness << F(",control,") << control << std::endl;
        }
        last_cycle_ms = now;
    }
}
