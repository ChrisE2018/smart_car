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
        virtual int get_preferred_interval () const;
        virtual int get_expected_us () const;
        void cycle ();

    private:
        Car &car;
        const float angle_tolerance = 0.01;
        const float angle_desired_velocity = 1.0;
        const float medium_speed_angle = 0.3;
        const float slow_speed_angle = 0.2;
        bool adjust_angle = false;
        float goal_angle = 0;
        const float position_tolerance = 0.05;
        const float slow_speed_distance = 0.25;
        const float medium_speed_distance = 0.5;
        bool adjust_position = false;
        float goal_x = 0;
        float goal_y = 0;
        void angle_cycle (const float measured_angle, const float desired_angle);
        void angle_step (const float measured_angle, const float desired_angle, const float delta);
        void position_cycle (const float measured_angle, const float measured_x,
                const float measured_y, const float desired_x, const float desired_y);
        void position_step (const float measured_angle, const float measured_x,
                const float measured_y, const float desired_x, const float desired_y);

        int get_angle_speed (const float desired_angle);
        int get_position_speed (const float distance);
};

