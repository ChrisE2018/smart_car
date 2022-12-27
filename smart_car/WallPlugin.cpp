/*
 * WallMode.cpp
 *
 *  Created on: Dec 26, 2022
 *      Author: cre
 */

#include "Car.hpp"
#include "WallPlugin.hpp"
#include "UltrasoundPlugin.hpp"

WallPlugin::WallPlugin (Car &car) : Plugin(WALL_PLUGIN), car(car)
{
}

void WallPlugin::set_mode (Mode mode)
{
    set_enabled(mode == WALL_MODE);
}

void WallPlugin::set_enabled (const bool enable)
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

void WallPlugin::cycle ()
{
    if (active)
    {
        UltrasoundPlugin* ultrasound_plugin = car.get_ultrasound_plugin();
        long d = ultrasound_plugin->get_distance();
        Serial.print("Distance ");
        Serial.print(d);
        Serial.println(" cm");
        if (d < 10)
        {
            car.all_stop();
            car.set_mode(COMMAND_MODE);
        }
        else
        {
            if (d < 50)
            {
                speed = SPEED_Q3;
            }
            else if (d < 25)
            {
                speed = SPEED_HALF;
            }
            car.drive_forward(RIGHT, speed);
            car.drive_forward(LEFT, speed);
        }
    }
}
