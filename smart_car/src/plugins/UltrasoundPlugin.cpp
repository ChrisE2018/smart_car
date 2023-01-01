/*
 * UltrasoundPlugin.cpp
 *
 *  Created on: Dec 26, 2022
 *      Author: cre
 */

#include "UltrasoundPlugin.hpp"
#include "smart_car.hpp"

UltrasoundPlugin::UltrasoundPlugin () : Plugin(ULTRASOUND_PLUGIN), sr04(ULTRASOUND_ECHO,
        ULTRASOUND_TRIGGER)
{

}

long UltrasoundPlugin::get_distance ()
{
    return sr04.Distance();
}

void UltrasoundPlugin::cycle ()
{
    if (is_enabled())
    {
        const long d = sr04.Distance();
        cout << " Distance " << d << " cm" << std::endl;
    }
}
