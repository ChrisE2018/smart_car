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
    return lhs << "#[motor " << motor.location << " " << motor.measured_velocity << " m/s err=" << motor.velocity_error
            << "]";
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
    drive_stop();
    logger.data() << F("k0: ") << k0 << F(" k1: ") << k1 << F(" k2: ") << k2 << F(" k3: ") << k3 << F(" k4: ") << k4
            << std::endl;
    return true;
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
        cout << location << "_" << direction << ": " << speed << " " << location << "_mps: " << measured_velocity
                << " cve: " << cumulative_velocity_error << std::endl;
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
        cout << location << "_" << direction << ": " << speed << " " << location << "_mps: " << measured_velocity
                << " cve: " << cumulative_velocity_error << std::endl;
    }
}

void MotorPlugin::drive_stop ()
{
    drive_zero_speed();
    desired_velocity = 0;
    cumulative_velocity_error = 0;
    auto_velocity = false;
    LOG_INFO(logger, "drive_stop %s", stringify(location).c_str());
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
        cout << "drive " << location << " " << direction << " at " << speed << " power " << measured_velocity << " mps"
                << " cve: " << cumulative_velocity_error << std::endl;
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
    desired_velocity = 0;
    velocity_error = 0;
    cumulative_velocity_error = 0;
    LOG_INFO(logger, "cancel_auto_velocity %s", stringify(location).c_str());
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

int MotorPlugin::sign (const float direction) const
{
    if (direction < 0)
    {
        return -1;
    }
    return 1;
}

int MotorPlugin::get_raw_ticks () const
{
    switch (location)
    {
        case MotorLocation::RIGHT_FRONT:
            return get_right_front_speed_counter();

        case MotorLocation::LEFT_FRONT:
            return get_left_front_speed_counter();

        case MotorLocation::RIGHT_REAR:
            return get_right_rear_speed_counter();

        case MotorLocation::LEFT_REAR:
            return get_left_rear_speed_counter();
    }
    return 0;
}

void MotorPlugin::cycle ()
{
    const unsigned long now = millis();
    const unsigned long delta_ms = now - last_cycle_ms;
    const long raw_speed_ticks = get_raw_ticks();
    const long measured_speed_counter = sign(desired_velocity) * raw_speed_ticks;
    const long speed_ticks = measured_speed_counter - speed_counter;
    if (delta_ms > minimum_cycle_ms || abs(speed_ticks) > minimum_speed_ticks)
    {
        const float delta_seconds = delta_ms * 0.001;
        const long previous_speed_counter = speed_counter;
        speed_counter = measured_speed_counter;
        const double measured_distance = speed_ticks * count_to_meters_per_second;
        measured_velocity = measured_distance / delta_seconds;
        if (abs(measured_velocity) > 0)
        {
            logger.data() << F("/motor/") << stringify(location).c_str() << F("/delta_seconds,") << delta_seconds
                    << F(",speed_counter,") << speed_counter << F(",measured_velocity,") << measured_velocity
                    << F(",measured_distance,") << measured_distance << F(",raw_speed_ticks,") << raw_speed_ticks
                    << std::endl;
        }
        if (auto_velocity)
        {
            const float previous_velocity_error = velocity_error;
            velocity_error = desired_velocity - measured_velocity;
            const float velocity_error_rate = (velocity_error - previous_velocity_error) / delta_seconds;
            cumulative_velocity_error += velocity_error * delta_seconds;
            const float smallness = std::max(0.0f, k4 - abs(measured_velocity));
            const float control_raw = k0
                    * (velocity_error + k1 * cumulative_velocity_error + k2 * velocity_error_rate
                            + smallness * velocity_error * k3);
            // No reversing
            const float control = desired_velocity >= 0.0f ? std::max(0.0f, control_raw) : std::min(0.0f, control_raw);
            logger.data() << F("/motor/") << stringify(location).c_str() << F("/velocity_error,") << velocity_error
                    << F(",cumulative_velocity_error,") << cumulative_velocity_error << F(",velocity_error_rate,")
                    << velocity_error_rate << F(",smallness,") << smallness << F(",control,") << control << std::endl;
            set_speed(control);
        }
        last_cycle_ms = now;
    }
}
