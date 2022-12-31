/*
 * DemoMode.cpp
 *
 *  Created on: Dec 26, 2022
 *      Author: cre
 */

#include "DemoPlugin.hpp"

#include "Car.hpp"
#include "DrivePlugin.hpp"

DemoPlugin::DemoPlugin (Car &car) : Plugin(DEMO_PLUGIN), car(car)
{
}

void DemoPlugin::set_mode (Mode mode)
{
    set_enabled(mode == DEMO_MODE);
}

void DemoPlugin::set_enabled (const bool enable)
{
    Plugin::set_enabled(enable);
    if (enable)
    {
        Serial.println("Starting DemoMode");
        set_phase(init_phase);
    }
    else
    {
        phase = inactive_phase;
    }
}

void DemoPlugin::set_phase (DemoPhase phase)
{
    this->phase = phase;
    phase_start = millis();
    switch (phase)
    {
        case init_phase:
        {
            const int duration = 10;
            phase_change = phase_start + duration;
            next_phase = forward_phase;
            car.get_forward_plugin()->set_enabled(false);
            car.get_reverse_plugin()->set_enabled(false);
            car.get_clockwise_plugin()->set_enabled(false);
            car.get_counterclockwise_plugin()->set_enabled(false);
            break;
        }

        case forward_phase:
        {
            const int duration = 1500;
            phase_change = phase_start + duration;
            next_phase = idle_phase1;
            DrivePlugin* drive_mode = car.get_forward_plugin();
            if (drive_mode != nullptr)
            {
                drive_mode->set_right_speed(speed);
                drive_mode->set_left_speed(speed);
                drive_mode->set_duration(duration);
                drive_mode->set_enabled(true);
            }
            break;
        }

        case idle_phase1:
        {
            phase_change = phase_start + 5000;
            next_phase = reverse_phase;
            car.drive_stop(RIGHT);
            car.drive_stop(LEFT);
            break;
        }

        case reverse_phase:
        {
            const int duration = 1500;
            phase_change = phase_start + duration;
            next_phase = idle_phase2;
            DrivePlugin *drive_mode = car.get_reverse_plugin();
            if (drive_mode != nullptr)
            {
                drive_mode->set_right_speed(speed);
                drive_mode->set_left_speed(speed);
                drive_mode->set_duration(duration);
                drive_mode->set_enabled(true);
            }
            break;
        }

        case idle_phase2:
        {
            phase_change = phase_start + 5000;
            next_phase = clockwise_phase;
            car.drive_stop(RIGHT);
            car.drive_stop(LEFT);
            break;
        }

        case clockwise_phase:
        {
            const int duration = 1500;
            phase_change = phase_start + duration;
            next_phase = idle_phase3;
            DrivePlugin *drive_mode = car.get_clockwise_plugin();
            if (drive_mode != nullptr)
            {
                drive_mode->set_right_speed(speed);
                drive_mode->set_left_speed(speed);
                drive_mode->set_duration(duration);
                drive_mode->set_enabled(true);
            }
            break;
        }

        case idle_phase3:
        {
            phase_change = phase_start + 5000;
            next_phase = counterclockwise_phase;
            car.drive_stop(RIGHT);
            car.drive_stop(LEFT);
            break;
        }

        case counterclockwise_phase:
        {
            const int duration = 1500;
            phase_change = phase_start + duration;
            next_phase = idle_phase4;
            DrivePlugin *drive_mode = car.get_counterclockwise_plugin();
            if (drive_mode != nullptr)
            {
                drive_mode->set_right_speed(speed);
                drive_mode->set_left_speed(speed);
                drive_mode->set_duration(duration);
                drive_mode->set_enabled(true);
            }
            break;
        }

        case idle_phase4:
        {
            phase_change = phase_start + 5000;
            next_phase = forward_phase;
            speed = SPEED_STOP;
            car.drive_stop(RIGHT);
            car.drive_stop(LEFT);
            break;
        }
    }
}

void DemoPlugin::cycle ()
{
    if (phase != inactive_phase)
    {
        if (phase_change < millis())
        {
            set_phase(next_phase);
        }
    }
}
