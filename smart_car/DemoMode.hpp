/*
 * DemoMode.hpp
 *
 *  Created on: Dec 26, 2022
 *      Author: cre
 */

#pragma once

#include "Car.hpp"
#include "Cyclic.hpp"

enum DemoPhase
{
    inactive_phase,
    forward_phase,
    idle_phase1,
    reverse_phase,
    idle_phase2,
    clockwise_phase,
    idle_phase3,
    counterclockwise_phase,
    idle_phase4,
};

class DemoMode: public Cyclic
{
    public:
        DemoMode (Car &car);

        void set_mode (Mode mode) override;
        void cycle () override;
        void set_phase (DemoPhase phase);

    private:
        Car &car;
        int speed = 0;
        DemoPhase phase = inactive_phase;
        DemoPhase next_phase = inactive_phase;
        long phase_start = 0;
        long phase_change = 0;
};

