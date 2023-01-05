/*
 * WallMode.cpp
 *
 *  Created on: Dec 26, 2022
 *      Author: cre
 */

#include "../robot/Car.hpp"
#include "WallPlugin.hpp"
#include "UltrasoundPlugin.hpp"

WallPlugin::WallPlugin (Car &car) : Plugin(WALL_PLUGIN), car(car)
{
}

void WallPlugin::set_enabled (const bool enable)
{
    Plugin::set_enabled(enable);
    if (enable)
    {
        Serial.println("Starting WallMode");
        speed = SPEED_FULL;
    }
}

void WallPlugin::cycle ()
{
    if (is_enabled())
    {
        UltrasoundPlugin* ultrasound_plugin = car.get_ultrasound_plugin();
        const long d = ultrasound_plugin->get_distance();
        Serial.print("Distance ");
        Serial.print(d);
        Serial.println(" cm");
        if (d < 10)
        {
            set_enabled(false);
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
