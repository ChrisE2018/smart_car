/*
 * UltrasoundPlugin.cpp
 *
 *  Created on: Dec 26, 2022
 *      Author: cre
 */

#include "UltrasoundPlugin.hpp"
#include "smart_car.hpp"
#include "../robot/Car.hpp"
#include "MotorPlugin.hpp"
#include "../logging/Logger.hpp"

static Logger logger(__FILE__, Level::info);

UltrasoundPlugin::UltrasoundPlugin (Car &car) :
                car(car), Plugin(PluginId::ULTRASOUND_PLUGIN),
                sr04(ULTRASOUND_ECHO, ULTRASOUND_TRIGGER)
{

}

int UltrasoundPlugin::get_preferred_interval () const
{
    return 100;
}

int UltrasoundPlugin::get_expected_ms () const
{
    return 8;
}

long UltrasoundPlugin::get_distance ()
{
    // This method is not const and includes a 25 ms delay.
    // That line has been commented out in the library routine.
    return sr04.Distance();
}

void UltrasoundPlugin::cycle ()
{
    const long d = sr04.Distance();
    if (is_enabled())
    {
        logger.info() << "Distance " << d << " cm" << std::endl;
    }
    if (d < 10)
    {
        bool did_stop = false;
        MotorPlugin &right_motor = car.get_motor(MotorLocation::RIGHT);
        MotorPlugin &left_motor = car.get_motor(MotorLocation::LEFT);
        // Only stop if moving forward toward the obstacle.
        if (right_motor.get_velocity() > 0)
        {
            did_stop = true;
            right_motor.drive_stop();
        }
        if (left_motor.get_velocity() > 0)
        {
            did_stop = true;
            left_motor.drive_stop();
        }
        if (did_stop)
        {
            LOG_INFO(logger, "Stopped due to object at %d cm", d);
        }
    }
}
