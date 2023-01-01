/*
 * GoalPlugin.hpp
 *
 *  Created on: Jan 1, 2023
 *      Author: cre
 */

#pragma once

#include "Plugin.hpp"

class Car;

class GoalPlugin : public Plugin
{
    public:
        GoalPlugin (Car &Car);

        void set_goal (const float angle);
        void set_goal (const float x, const float y);
        bool setup ();
        void reset ();
        void cycle ();

    private:

        Car &car;
        const float angle_tolerance = 0.01;
        const float medium_speed_angle = 0.25;
        const float slow_speed_angle = 0.15;
        bool adjust_angle = false;
        float goal_angle = 0;
        const float position_tolerance = 0.05;
        bool adjust_position = false;
        float goal_x = 0;
        float goal_y = 0;
};

