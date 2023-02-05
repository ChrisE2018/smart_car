/*
 * Copyright (c) 2023 by Christopher Eliot.
 *
 * DriveMode.hpp
 *
 *  Created on: Dec 26, 2022
 *      Author: cre
 */

#pragma once

#include "Plugin.hpp"
#include "MotorPlugin.hpp"
class Car;

class DrivePlugin: public Plugin
{
    public:
        DrivePlugin (const PluginId id, Car &car, const int duration, const MotorDirection right_motor_direction,
                const MotorDirection left_motor_direction);

        virtual void enter_state (const int state);

        virtual int get_expected_us () const;

        void cycle () override;

        void set_duration (const int _duration);

        void set_motor_speed (const int speed);

    private:
        Car &car;
        int duration;
        unsigned long deadline = 0;
        MotorDirection right_motor_direction;
        MotorDirection left_motor_direction;
        int motor_speed = SPEED_FULL;
};

