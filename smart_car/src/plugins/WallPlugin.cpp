/*
 * WallMode.cpp
 *
 *  Created on: Dec 26, 2022
 *      Author: cre
 */

#include "../robot/Car.hpp"
#include "WallPlugin.hpp"
#include "UltrasoundPlugin.hpp"

extern Car car;
extern UltrasoundPlugin ultrasound_plugin;

WallPlugin::WallPlugin () :
        Plugin(PluginId::WALL_PLUGIN)
{
}

void WallPlugin::enter_state (const int state)
{
    if (state == Plugin::ENABLE)
    {
        Serial.println(F("Starting WallMode"));
        speed = SPEED_FULL;
    }
}

void WallPlugin::cycle ()
{
    if (get_state() == Plugin::ENABLE)
    {
        const long d = ultrasound_plugin.get_distance();
        Serial.print(F("Distance "));
        Serial.print(d);
        Serial.println(F(" cm"));
        if (d < 10)
        {
            set_state(Plugin::DISABLE);
            car.all_stop();
            car.set_mode(Mode::COMMAND_MODE);
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
            car.set_right_speed(speed);
            car.set_left_speed(speed);
        }
    }
}
