/*
 * Copyright (c) 2023 by Christopher Eliot.
 *
 * ClockPlugin.cpp
 *
 *  Created on: Dec 28, 2022
 *      Author: cre
 */

#include "ClockPlugin.hpp"
#include "smart_car.hpp"
#include <Wire.h>

ClockPlugin::ClockPlugin () :
        Plugin(PluginId::CLOCK_PLUGIN)
{
    if (enable_clock)
    {
        if (clock.begin())
        {
            // Send sketch compiling time to Arduino
//        clock.setDateTime(__DATE__, __TIME__);
            is_setup = true;
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
        cout << "Disabled RTC module" << std::endl;
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
