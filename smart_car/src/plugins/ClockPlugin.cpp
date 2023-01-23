/*
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
}

bool ClockPlugin::setup ()
{
    if (enable_clock)
    {
        cout << "Initialize RTC module" << std::endl;
        clock.begin();
        // Send sketch compiling time to Arduino
//        clock.setDateTime(__DATE__, __TIME__);
        is_setup = true;
    }
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

time_t ClockPlugin::get_unixtime ()
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
