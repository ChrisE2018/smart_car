/*
 * DemoMode.cpp
 *
 *  Created on: Dec 26, 2022
 *      Author: cre
 */

#include "DemoMode.hpp"
#include "Car.hpp"

DemoMode::DemoMode (Car &car) : Plugin(DEMO_PLUGIN), car(car)
{
}

void DemoMode::set_mode (Mode mode)
{
    set_enabled(mode == DEMO_MODE);
}

void DemoMode::set_enabled (const bool enable)
{
    Plugin::set_enabled(enable);
    if (enable)
    {
        Serial.println("Starting DemoMode");
        set_phase(forward_phase);
    }
    else
    {
        phase = inactive_phase;
    }
}

void DemoMode::set_phase (DemoPhase phase)
{
    this->phase = phase;
    phase_start = millis();
    switch (phase)
    {
        case forward_phase:
        {
            const int duration = 1500;
            phase_change = phase_start + duration;
            next_phase = idle_phase1;
            DriveMode *drive_mode = car.get_forward_mode();
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
            car.drive_stop(0);
            car.drive_stop(1);
            break;
        }

        case reverse_phase:
        {
            const int duration = 1500;
            phase_change = phase_start + duration;
            next_phase = idle_phase2;
            DriveMode *drive_mode = car.get_reverse_mode();
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
            car.drive_stop(0);
            car.drive_stop(1);
            break;
        }

        case clockwise_phase:
        {
            const int duration = 1500;
            phase_change = phase_start + duration;
            next_phase = idle_phase3;
            DriveMode *drive_mode = car.get_clockwise_mode();
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
            car.drive_stop(0);
            car.drive_stop(1);
            break;
        }

        case counterclockwise_phase:
        {
            const int duration = 1500;
            phase_change = phase_start + duration;
            next_phase = idle_phase4;
            DriveMode *drive_mode = car.get_counterclockwise_mode();
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
            car.drive_stop(0);
            car.drive_stop(1);
            break;
        }
    }
}

void DemoMode::cycle ()
{
    if (phase != inactive_phase)
    {
        if (phase_change < millis())
        {
            set_phase(next_phase);
        }
    }
}
