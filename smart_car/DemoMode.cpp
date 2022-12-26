/*
 * DemoMode.cpp
 *
 *  Created on: Dec 26, 2022
 *      Author: cre
 */

#include "DemoMode.hpp"
#include "Car.hpp"

DemoMode::DemoMode (Car &car) : car(car)
{
}

void DemoMode::set_mode (Mode mode)
{
    Cyclic::set_mode(mode);
    if (mode == DEMO_MODE)
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
            phase_change = phase_start + 1500;
            next_phase = idle_phase1;
            speed = SPEED_FULL;
            car.drive_forward(0, speed);
            car.drive_forward(1, speed);
            break;

        case idle_phase1:
            phase_change = phase_start + 5000;
            next_phase = reverse_phase;
            speed = SPEED_STOP;
            car.drive_stop(0);
            car.drive_stop(1);
            break;

        case reverse_phase:
            phase_change = phase_start + 1500;
            next_phase = idle_phase2;
            speed = SPEED_FULL;
            car.drive_reverse(0, speed);
            car.drive_reverse(1, speed);
            break;

        case idle_phase2:
            phase_change = phase_start + 5000;
            next_phase = clockwise_phase;
            speed = SPEED_STOP;
            car.drive_stop(0);
            car.drive_stop(1);
            break;

        case clockwise_phase:
            phase_change = phase_start + 1500;
            next_phase = idle_phase3;
            speed = SPEED_FULL;
            car.drive_forward(0, SPEED_FULL);
            car.drive_reverse(1, SPEED_FULL);
            break;

        case idle_phase3:
            phase_change = phase_start + 5000;
            next_phase = counterclockwise_phase;
            speed = SPEED_STOP;
            car.drive_stop(0);
            car.drive_stop(1);
            break;

        case counterclockwise_phase:
            phase_change = phase_start + 1500;
            next_phase = idle_phase4;
            speed = SPEED_FULL;
            car.drive_forward(0, SPEED_FULL);
            car.drive_reverse(1, SPEED_FULL);
            break;

        case idle_phase4:
            phase_change = phase_start + 5000;
            next_phase = forward_phase;
            speed = SPEED_STOP;
            car.drive_stop(0);
            car.drive_stop(1);
            break;
    }
}

void DemoMode::cycle ()
{
    if (get_mode() == DEMO_MODE)
    {
        if (phase_change < millis())
        {
            set_phase(next_phase);
        }
    }
}
