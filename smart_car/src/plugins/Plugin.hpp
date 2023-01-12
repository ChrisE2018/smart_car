/*
 * Plugin.hpp
 *
 *  Created on: Dec 23, 2022
 *      Author: cre
 */

#pragma once

#include <string>
#include "../robot/Mode.hpp"

enum PluginId
{
    CLOCK_PLUGIN,
    CLOCKWISE_PLUGIN,
    COUNTERCLOCKWISE_PLUGIN,
    DEMO_PLUGIN,
    FORWARD_PLUGIN,
    GOAL_PLUGIN,
    KALMAN_PLUGIN,
    ODOM_PLUGIN,
    MOTOR_LEFT_PLUGIN,
    MOTOR_RIGHT_PLUGIN,
    MPU_PLUGIN,
    REVERSE_PLUGIN,
    ULTRASOUND_PLUGIN,
    WALL_PLUGIN,
    COMMAND_CYCLE,
    IDLE_CYCLE
};

const std::string stringify (const PluginId id);

std::ostream& operator<< (std::ostream &lhs, const PluginId id);

class Plugin
{
    public:
        Plugin (const PluginId id);
        virtual ~Plugin ();

        const PluginId get_id () const;
        virtual bool setup ();
        virtual bool reset ();
        virtual const bool is_cyclic () const;
        virtual int get_preferred_interval () const;
        virtual int get_expected_us () const;
        const bool is_enabled () const;
        virtual void set_enabled (const bool enable);
        virtual void cycle ();
        void start_cycle ();
        void end_cycle ();
        unsigned long get_cycle_count () const;
        unsigned long get_total_micros () const;
        unsigned long get_overrun_count () const;

    private:
        const PluginId id;
        bool enable = false;
        unsigned long cycle_count = 0;
        unsigned long overrun_count = 0;
        unsigned long cycle_start_micros = 0;
        unsigned long total_micros = 0;
};

