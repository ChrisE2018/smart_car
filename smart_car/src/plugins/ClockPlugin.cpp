/*
 * ClockPlugin.cpp
 *
 *  Created on: Dec 28, 2022
 *      Author: cre
 */

#include <Arduino.h>
#include "ClockPlugin.hpp"
#include <Wire.h>

ClockPlugin::ClockPlugin () :
        Plugin(PluginId::CLOCK_PLUGIN)
{
}

void ClockPlugin::begin ()
{
    if (enable_clock)
    {
        if (clock.begin())
        {
            Serial.println(F("Initialized clock"));
            // Send sketch compiling time to Arduino
            //        clock.setDateTime(__DATE__, __TIME__);
            is_setup = true;
        }
        else
        {
            Serial.println(F("Clock setup failed"));
        }
    }
}

bool ClockPlugin::setup ()
{
    if (is_setup)
    {
        return true;
    }
    else
    {
        return false;
    }
}

bool ClockPlugin::is_cyclic () const
{
    return false;
}

time_t ClockPlugin::unixtime ()
{
    if (is_setup)
    {
        dt = clock.getDateTime();
        // @see https://forum.arduino.cc/t/result-of-strftime-is-30-years-off/658302/9
        return dt.unixtime - UNIX_OFFSET + 3600;
    }
    else
    {
        return 0;
    }
}
