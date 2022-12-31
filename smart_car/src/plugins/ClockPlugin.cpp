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
    Serial.println("Initialize RTC module");
    clock.begin();
    // Send sketch compiling time to Arduino
//    clock.setDateTime(__DATE__, __TIME__);
    return true;
}

void ClockPlugin::cycle ()
{
    if (is_enabled())
    {
        dt = clock.getDateTime();
        Serial.print("Raw data: ");
        Serial.print(dt.year);
        Serial.print("-");
        if (dt.month < 10)
            Serial.print("0");
        Serial.print(dt.month);
        Serial.print("-");
        if (dt.day < 10)
            Serial.print("0");
        Serial.print(dt.day);
        Serial.print(" ");
        Serial.print(dt.hour);
        Serial.print(":");
        if (dt.minute < 10)
            Serial.print("0");
        Serial.print(dt.minute);
        Serial.print(":");
        if (dt.second < 10)
            Serial.print("0");
        Serial.print(dt.second);
        Serial.println("");
    }
}
