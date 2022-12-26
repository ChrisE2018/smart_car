/*
 * DriveMode.hpp
 *
 *  Created on: Dec 26, 2022
 *      Author: cre
 */

#pragma once

#include "Cyclic.hpp"
#include "Car.hpp"
#include "Mode.hpp"

class DriveMode: public Cyclic
{
    public:
        DriveMode (Car &car, Mode mode, const int duration, const MotorDirection right_motor_direction,
                const MotorDirection left_motor_direction);

        void set_mode (Mode mode) override;
        void cycle () override;

    private:
        Car &car;
        Mode mode;
        const int duration;
        int deadline = 0;
        MotorDirection right_motor_direction;
        MotorDirection left_motor_direction;
        int right_motor_speed = SPEED_FULL;
        int left_motor_speed = SPEED_FULL;
};

