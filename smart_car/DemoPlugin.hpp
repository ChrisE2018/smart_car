/*
 * DemoMode.hpp
 *
 *  Created on: Dec 26, 2022
 *      Author: cre
 */

#pragma once

#include "Plugin.hpp"
#include "Motor.hpp"
class Car;

enum DemoPhase
{
    inactive_phase,
    init_phase,
    forward_phase,
    idle_phase1,
    reverse_phase,
    idle_phase2,
    clockwise_phase,
    idle_phase3,
    counterclockwise_phase,
    idle_phase4,
};

class DemoPlugin: public Plugin
{
    public:
        DemoPlugin (Car &car);
        void set_mode (Mode mode);
        void set_enabled(const bool enable);
        void cycle () override;
        void set_phase (DemoPhase phase);

    private:
        Car &car;
        int speed = SPEED_FULL;
        DemoPhase phase = inactive_phase;
        DemoPhase next_phase = inactive_phase;
        long phase_start = 0;
        long phase_change = 0;
};

