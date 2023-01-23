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
#include "Logger.hpp"

extern Car car;

static logging::Logger logger(__FILE__, logging::Level::info);

UltrasoundPlugin::UltrasoundPlugin () :
        Plugin(PluginId::ULTRASOUND_PLUGIN), sr04(ULTRASOUND_ECHO, ULTRASOUND_TRIGGER)
{
    set_state(Plugin::DISABLE);
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
    const int state = get_state();
    if (state != Plugin::DISABLE)
    {
        // This method is not const and includes a 25 ms delay.
        // That line has been commented out in the library routine.
        // https://www.physicsclassroom.com/class/sound/Lesson-2/The-Speed-of-Sound
        distance = sr04.Distance();
        if (state == BLOCKING)
        {
            if (distance < blocking_distance)
            {
                bool did_stop = false;
                for (int m = 0; m < MOTOR_COUNT; m++)
                {
                    PidPlugin &motor = car.get_pid_plugin(static_cast<MotorLocation>(m));
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
