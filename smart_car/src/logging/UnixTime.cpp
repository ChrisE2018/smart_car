/*
 * UnixTime.cpp
 *
 *  Created on: Jan 12, 2023
 *      Author: cre
 */

#include "UnixTime.hpp"
#include "../robot/Car.hpp"
#include "../plugins/ClockPlugin.hpp"

time_t get_unixtime (Car &car)
{
    time_t t = 0;
    ClockPlugin *const clock_plugin = car.get_clock_plugin();
    if (clock_plugin != nullptr)
    {
        t = clock_plugin->get_unixtime();
    }
    return t;
}

