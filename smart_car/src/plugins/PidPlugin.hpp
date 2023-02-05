/*
 * Copyright (c) 2023 by Christopher Eliot.
 *
 * PidPlugin.hpp
 *
 *  Created on: Jan 17, 2023
 *      Author: cre
 */

#pragma once

#include "Plugin.hpp"
#include "MotorPlugin.hpp"

class Car;

class PidPlugin: public Plugin
{
    public:
        PidPlugin (const PluginId id, MotorPlugin &motor_plugin);
        friend std::ostream& operator<< (std::ostream &lhs, const PidPlugin &motor);
        MotorLocation get_location () const;
        unsigned long get_speed_counter () const;
        float get_measured_distance () const;
        float get_desired_velocity () const;
        void set_desired_velocity (const float desired_velocity);
        float get_measured_velocity () const;
        float get_cumulative_velocity_error () const;
        virtual void enter_state (const int state) override;
        void drive_stop ();
        float get_velocity_error () const;
        virtual int get_preferred_interval () const override;
        virtual int get_expected_us () const override;
        virtual bool setup () override;
        virtual void cycle () override;

    private:
        MotorPlugin &motor_plugin;
        const MotorLocation location;

        // Support for speed encoders
        static constexpr int minimum_cycle_ms = 100;    // When stopped
        static constexpr int minimum_speed_ticks = 4;   // When moving

        static constexpr float k0 = 0.25 * SPEED_FULL;
        static constexpr float k1 = 1.0;
        static constexpr float k2 = 0;//-0.01;
        static constexpr float k3 = 0; //0.15;
        static constexpr float k4 = 0; //0.2;

        float desired_velocity = 0;
        float measured_velocity = 0;
        unsigned long last_cycle_ms = 0;
        long speed_counter = 0;
        float velocity_error = 0;
        float cumulative_velocity_error = 0;
        // Only returns +1 or -1 unlike signum
        int sign (const float direction) const;
};

