/*
 * ClockPlugin.cpp
 *
 *  Created on: Dec 28, 2022
 *      Author: cre
 */

#include "ClockPlugin.hpp"
#include "smart_car.hpp"
#include <Wire.h>

ClockPlugin::ClockPlugin () : Plugin(CLOCK_PLUGIN)
{
    cout << "Created RTC Plugin " << get_id() << std::endl;
}

bool ClockPlugin::setup ()
{
    cout << "Initialize RTC module" << std::endl;
    clock.begin();
    return true;
}

void ClockPlugin::cycle ()
{
    if (is_enabled())
    {
        dt = clock.getDateTime();
        cout << "Raw data: " << static_cast<int>(dt.year) << "-";
        const int month = static_cast<int>(dt.month);
        if (month < 10)
            cout << "0";
        cout << month << "-";
        const int day = static_cast<int>(dt.day);
        if (day < 10)
            cout << "0";
        cout << day << " " << static_cast<int>(dt.hour) << ":";
        const int minute = static_cast<int>(dt.minute);
        if (minute < 10)
            cout << "0";
        cout << minute << ":";
        const int second = static_cast<int>(dt.second);
        if (second < 10)
            cout << "0";
        cout << second << std::endl;
    }
}
