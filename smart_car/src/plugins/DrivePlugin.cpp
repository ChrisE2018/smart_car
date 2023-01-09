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

DrivePlugin::DrivePlugin (const PluginId id, Car &car, const int duration,
        const MotorDirection right_motor_direction, const MotorDirection left_motor_direction) :
                Plugin(id), car(car), duration(duration),
                right_motor_direction(right_motor_direction),
                left_motor_direction(left_motor_direction)
{
}

void DrivePlugin::set_enabled (const bool enable)
{
    Plugin::set_enabled(enable);
    if (enable)
    {
        unsigned long now = millis();
        deadline = now + duration;
        logger.info() << "Starting DriveMode: " << get_id() << " speed " << right_motor_speed
                << ", " << left_motor_speed << " for " << duration << " from " << now << " until "
                << deadline << std::endl;
        switch (right_motor_direction)
        {
            case MotorDirection::STOP:
                car.drive_stop(MotorLocation::RIGHT);
                break;
            case MotorDirection::FORWARD:
                car.drive_forward(MotorLocation::RIGHT, right_motor_speed);
                break;
            case MotorDirection::REVERSE:
                car.drive_reverse(MotorLocation::RIGHT, right_motor_speed);
                break;
        }
        switch (left_motor_direction)
        {
            case MotorDirection::STOP:
                car.drive_stop(MotorLocation::LEFT);
                break;
            case MotorDirection::FORWARD:
                car.drive_forward(MotorLocation::LEFT, left_motor_speed);
                break;
            case MotorDirection::REVERSE:
                car.drive_reverse(MotorLocation::LEFT, left_motor_speed);
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
