/*
 * DemoMode.cpp
 *
 *  Created on: Dec 26, 2022
 *      Author: cre
 */

#include "DemoPlugin.hpp"

#include "../robot/Car.hpp"
#include "DrivePlugin.hpp"

DemoPlugin::DemoPlugin (Car &car) :
                Plugin(PluginId::DEMO_PLUGIN), car(car)
{
}

void DemoPlugin::set_enabled (const bool enable)
{
    Plugin::set_enabled(enable);
    if (enable)
    {
        Serial.println(F("Starting DemoMode"));
        set_phase(DemoPhase::init_phase);
    }
    else
    {
        phase = DemoPhase::inactive_phase;
    }
}

int DemoPlugin::get_expected_us () const
{
    return 100;
}

void DemoPlugin::set_phase (const DemoPhase phase)
{
    this->phase = phase;
    phase_start = millis();
    switch (phase)
    {
        case DemoPhase::init_phase:
        {
            const int duration = 10;
            phase_change = phase_start + duration;
            next_phase = DemoPhase::forward_phase;
            car.get_forward_plugin()->set_enabled(false);
            car.get_reverse_plugin()->set_enabled(false);
            car.get_clockwise_plugin()->set_enabled(false);
            car.get_counterclockwise_plugin()->set_enabled(false);
            break;
        }

        case DemoPhase::forward_phase:
        {
            const int duration = 1500;
            phase_change = phase_start + duration;
            next_phase = DemoPhase::idle_phase1;
            DrivePlugin *drive_mode = car.get_forward_plugin();
            if (drive_mode != nullptr)
            {
                drive_mode->set_right_speed(speed);
                drive_mode->set_left_speed(speed);
                drive_mode->set_duration(duration);
                drive_mode->set_enabled(true);
            }
            break;
        }

        case DemoPhase::idle_phase1:
        {
            phase_change = phase_start + 5000;
            next_phase = DemoPhase::reverse_phase;
            car.drive_stop(MotorLocation::RIGHT_FRONT);
            car.drive_stop(MotorLocation::LEFT_FRONT);
            break;
        }

        case DemoPhase::reverse_phase:
        {
            const int duration = 1500;
            phase_change = phase_start + duration;
            next_phase = DemoPhase::idle_phase2;
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

        case DemoPhase::idle_phase2:
        {
            phase_change = phase_start + 5000;
            next_phase = DemoPhase::clockwise_phase;
            car.drive_stop(MotorLocation::RIGHT_FRONT);
            car.drive_stop(MotorLocation::LEFT_FRONT);
            break;
        }

        case DemoPhase::clockwise_phase:
        {
            const int duration = 1500;
            phase_change = phase_start + duration;
            next_phase = DemoPhase::idle_phase3;
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

        case DemoPhase::idle_phase3:
        {
            phase_change = phase_start + 5000;
            next_phase = DemoPhase::counterclockwise_phase;
            car.drive_stop(MotorLocation::RIGHT_FRONT);
            car.drive_stop(MotorLocation::LEFT_FRONT);
            break;
        }

        case DemoPhase::counterclockwise_phase:
        {
            const int duration = 1500;
            phase_change = phase_start + duration;
            next_phase = DemoPhase::idle_phase4;
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

        case DemoPhase::idle_phase4:
        {
            phase_change = phase_start + 5000;
            next_phase = DemoPhase::forward_phase;
            speed = SPEED_STOP;
            car.drive_stop(MotorLocation::RIGHT_FRONT);
            car.drive_stop(MotorLocation::LEFT_FRONT);
            break;
        }
    }
}

void DemoPlugin::cycle ()
{
    if (phase != DemoPhase::inactive_phase)
    {
        if (phase_change < millis())
        {
            set_phase(next_phase);
        }
    }
}
