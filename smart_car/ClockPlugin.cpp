/*
 * ClockPlugin.cpp
 *
 *  Created on: Dec 28, 2022
 *      Author: cre
 */

#include "ClockPlugin.hpp"
#include <Wire.h>

ClockPlugin::ClockPlugin () : Plugin(CLOCK_PLUGIN)
{
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
        Serial.print(dt.month);
        Serial.print("-");
        Serial.print(dt.day);
        Serial.print(" ");
        Serial.print(dt.hour);
        Serial.print(":");
        Serial.print(dt.minute);
        Serial.print(":");
        Serial.print(dt.second);
        Serial.println("");
    }
}
