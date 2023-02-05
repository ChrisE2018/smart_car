/*
 * Copyright (c) 2023 by Christopher Eliot.
 *
 * Plugin.hpp
 *
 *  Created on: Dec 23, 2022
 *      Author: cre
 */

#pragma once

#include <string>
#include "../robot/Mode.hpp"

enum PluginId : unsigned char
{
    CLOCK_PLUGIN,
    FORWARD_PLUGIN,
    GOAL_PLUGIN,
    KALMAN_PLUGIN,
    ODOM_PLUGIN,
    MOTOR_LEFT_FRONT_PLUGIN,
    MOTOR_RIGHT_FRONT_PLUGIN,
    MOTOR_LEFT_REAR_PLUGIN,
    MOTOR_RIGHT_REAR_PLUGIN,
    MPU_PLUGIN,
    PID_LEFT_FRONT_PLUGIN,
    PID_RIGHT_FRONT_PLUGIN,
    PID_LEFT_REAR_PLUGIN,
    PID_RIGHT_REAR_PLUGIN,
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
        static constexpr int DISABLE = 0;
        static constexpr int ENABLE = 1;

        Plugin (const PluginId id);
        virtual ~Plugin ();
        friend std::ostream& operator<< (std::ostream &lhs, const Plugin &plugin);
        friend std::ostream& operator<< (std::ostream &lhs, const Plugin *plugin);

        PluginId get_id () const;
        int get_instance () const;
        virtual bool setup ();
        virtual bool reset ();
        int get_state () const;
        void set_state (const int state);
        void _set_state (const int state);
        virtual void enter_state (const int state);
        virtual void exit_state (const int state);
        bool is_trace () const;
        void set_trace (const bool enable);
        virtual bool is_cyclic () const;
        virtual int get_preferred_interval () const;
        virtual int get_expected_us () const;
        void major_cycle ();
        void start_cycle ();
        virtual void cycle ();
        void end_cycle ();
        virtual void trace ();
        unsigned long get_cycle_count () const;
        unsigned long get_total_micros () const;
        unsigned long get_overrun_count () const;

    private:
        static int INSTANCE;
        const PluginId id;
        bool enable_trace = false;
        const int instance;
        int state = DISABLE;
        unsigned long cycle_count = 0;
        unsigned long overrun_count = 0;
        unsigned long cycle_start_micros = 0;
        unsigned long total_micros = 0;
};

