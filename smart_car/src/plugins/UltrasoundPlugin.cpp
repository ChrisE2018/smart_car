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
        car(car), Plugin(PluginId::ULTRASOUND_PLUGIN), sr04(ULTRASOUND_ECHO, ULTRASOUND_TRIGGER)
{
    set_state(static_cast<int>(UltrasoundState::INACTIVE));
}

int UltrasoundPlugin::get_preferred_interval () const
{
    return 100;
}

int UltrasoundPlugin::get_expected_us () const
{
    return 2800;
}

long UltrasoundPlugin::get_distance () const
{
    return distance;
}

void UltrasoundPlugin::cycle ()
{
    UltrasoundState state = get_state();
    if (state != UltrasoundState::INACTIVE)
    {
        // This method is not const and includes a 25 ms delay.
        // That line has been commented out in the library routine.
        distance = sr04.Distance();
        if (state == UltrasoundState::BLOCKING)
        {
            if (distance < blocking_distance)
            {
                bool did_stop = false;
                for (int m = 0; m < MOTOR_COUNT; m++)
                {
                    MotorPlugin &motor = car.get_motor(static_cast<MotorLocation>(m));
                    // Only stop if moving forward toward the obstacle.
                    if (motor.get_measured_velocity() > 0)
                    {
                        did_stop = true;
                        motor.drive_stop();
                    }
                }
                if (did_stop)
                {
                    LOG_INFO(logger, "Stopped due to object at %d cm", distance);
                }
            }
        }
    }
}

void UltrasoundPlugin::trace ()
{
    logger.info() << "Distance " << distance << " cm" << std::endl;
}
