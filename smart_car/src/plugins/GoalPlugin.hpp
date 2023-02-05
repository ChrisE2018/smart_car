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
        static constexpr int ADJUST_ANGLE = 3;
        static constexpr int ADJUST_POSITION = 4;
        GoalPlugin (Car &Car);

        void set_goal (const float angle);
        void set_goal (const float x, const float y);
        bool setup ();
        virtual int get_preferred_interval () const;
        virtual int get_expected_us () const;
        virtual void cycle () override;

        /* Rotation distance calculation.
         * a = 11 cm wheel distance front to back center to center
         * b = 13 cm = wheel distance side to side center to center
         * D = diagonal wheel distance
         * a^2 + b^2 == D^2
         * 290 = D^2
         * D = 17 cm == 0.17 meters
         * Assume it is a circle
         * P = 53.4 cm = perimeter = pi * D = distance wheels move in one full circle in place
         */
    private:
        static constexpr float rotation_perimeter = M_PI * 0.17; // meters

        Car &car;
        const float angle_tolerance = 0.05;
        const float angle_desired_velocity = 1.0;
        const float medium_speed_angle = 0.3;
        const float slow_speed_angle = 0.2;

        float goal_angle = 0;
        const float position_tolerance = 0.05;
        const float slow_speed_distance = 0.25;
        const float medium_speed_distance = 0.5;

        float goal_x = 0;
        float goal_y = 0;
        void angle_cycle (const float measured_angle, const float desired_angle);
        void position_cycle (const float measured_angle, const float measured_x,
                const float measured_y, const float desired_x, const float desired_y);
        void position_step (const float measured_angle, const float measured_x,
                const float measured_y, const float desired_x, const float desired_y);

        int get_angle_speed (const float desired_angle);
        int get_position_speed (const float distance);
};

