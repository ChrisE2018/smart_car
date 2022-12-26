/*
 * WallMode.cpp
 *
 *  Created on: Dec 26, 2022
 *      Author: cre
 */

#include "WallMode.hpp"
#include "Car.hpp"

WallMode::WallMode (Car &car) : Plugin(WALL_PLUGIN), car(car)
{
}

void WallMode::set_mode (Mode mode)
{
    set_enabled(mode == WALL_MODE);
}

void WallMode::set_enabled (const bool enable)
{
    Plugin::set_enabled(enable);
    if (enable)
    {
        Serial.println("Starting WallMode");
        speed = SPEED_FULL;
        active = true;
    }
    else
    {
        active = false;
    }
}

void WallMode::cycle ()
{
    if (active)
    {
        long d = car.get_distance();
        Serial.print("Distance ");
        Serial.print(d);
        Serial.println(" cm");
        if (d < 10)
        {
            car.set_mode(COMMAND_MODE);
        }
        else if (d < 50)
        {
            speed = SPEED_Q3;
        }
        else if (d < 25)
        {
            speed = SPEED_HALF;
        }
        for (int i = 0; i < MOTOR_COUNT; i++)
        {
            car.drive_forward(i, speed);
        }
    }
}
