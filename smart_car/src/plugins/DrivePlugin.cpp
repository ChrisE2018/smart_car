/*
 * DriveMode.cpp
 *
 *  Created on: Dec 26, 2022
 *      Author: cre
 */

#include "smart_car.hpp"
#include "DrivePlugin.hpp"

#include "../logging/Logger.hpp"
#include "../robot/Car.hpp"

static Logger logger(__FILE__, Level::info);

DrivePlugin::DrivePlugin (const PluginId id, Car &car, const int duration, const MotorDirection right_motor_direction,
        const MotorDirection left_motor_direction) :
        Plugin(id), car(car), duration(duration), right_motor_direction(right_motor_direction), left_motor_direction(
                left_motor_direction)
{
}

void DrivePlugin::set_enabled (const bool enable)
{
    Plugin::set_enabled(enable);
    if (enable)
    {
        unsigned long now = millis();
        deadline = now + duration;
        logger.info() << "Starting DriveMode: " << get_id() << " speed " << motor_speed << " for " << duration
                << " from " << now << " until " << deadline << std::endl;
        switch (right_motor_direction)
        {
            case MotorDirection::STOP:
                car.set_right_speed(0);
                break;
            case MotorDirection::FORWARD:
                car.set_right_speed(motor_speed);
                break;
            case MotorDirection::REVERSE:
                car.set_right_speed(-motor_speed);
                break;
        }
        switch (left_motor_direction)
        {
            case MotorDirection::STOP:
                car.set_left_speed(0);
                break;
            case MotorDirection::FORWARD:
                car.set_left_speed(motor_speed);
                break;
            case MotorDirection::REVERSE:
                car.set_left_speed(-motor_speed);
                break;
        }
    }
}

void DrivePlugin::cycle ()
{
    if (is_enabled())
    {
        if (deadline < millis())
        {
            logger.info() << "Stopping DriveMode: " << get_id() << " at " << millis() << std::endl;
            set_enabled(false);
            car.all_stop();
        }
    }
}

int DrivePlugin::get_expected_us () const
{
    return 25;
}

void DrivePlugin::set_duration (const int _duration)
{
    duration = _duration;
}

void DrivePlugin::set_motor_speed (const int speed)
{
    motor_speed = speed;
}

