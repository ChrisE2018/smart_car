/*
 * UltrasoundPlugin.cpp
 *
 *  Created on: Dec 26, 2022
 *      Author: cre
 */

#include <Arduino.h>
#include "UltrasoundPlugin.hpp"

UltrasoundPlugin::UltrasoundPlugin () : Plugin(ULTRASOUND_PLUGIN), sr04(ULTRASOUND_ECHO,
        ULTRASOUND_TRIGGER)
{

}

long UltrasoundPlugin::get_distance ()
{
    return sr04.Distance();
}

void UltrasoundPlugin::print_distance ()
{
    const long d = sr04.Distance();
    Serial.print("Distance ");
    Serial.print(d);
    Serial.println(" cm");
}

void UltrasoundPlugin::cycle ()
{
    if (is_enabled())
    {
        print_distance();
    }
}
