/*
 * DriveMode.hpp
 *
 *  Created on: Dec 26, 2022
 *      Author: cre
 */

#pragma once

#include "Motor.hpp"
#include "Plugin.hpp"
class Car;

class DrivePlugin: public Plugin
{
    public:
        DrivePlugin (const PluginId id, Car &car, const int duration,
                const MotorDirection right_motor_direction,
                const MotorDirection left_motor_direction);

        void set_enabled (const bool enable);

        void cycle () override;

        void set_duration (const int _duration)
        {
            duration = _duration;
        }

        void set_right_speed (const int speed)
        {
            right_motor_speed = speed;
        }

        void set_left_speed (const int speed)
        {
            left_motor_speed = speed;
        }

    private:
        Car &car;
        int duration;
        long deadline = 0;
        MotorDirection right_motor_direction;
        MotorDirection left_motor_direction;
        int right_motor_speed = SPEED_FULL;
        int left_motor_speed = SPEED_FULL;
};
