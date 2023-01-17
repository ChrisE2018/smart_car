/*
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

        const PluginId get_id () const;
        virtual bool setup ();
        virtual bool reset ();
        virtual bool is_cyclic () const;
        virtual int get_preferred_interval () const;
        virtual int get_expected_us () const;
        bool is_enabled () const;
        virtual void set_enabled (const bool enable);
        bool is_trace () const;
        void set_trace (const bool enable);
        void major_cycle ();
        virtual void cycle ();
        void start_cycle ();
        void end_cycle ();
        virtual void trace ();
        unsigned long get_cycle_count () const;
        unsigned long get_total_micros () const;
        unsigned long get_overrun_count () const;
        int get_state () const;
        void set_state (const int state);
        virtual void enter_state (const int state);
        virtual void exit_state (const int state);

    private:
        const PluginId id;
        bool enable = false;
        bool enable_trace = false;
        int state = DISABLE;
        unsigned long cycle_count = 0;
        unsigned long overrun_count = 0;
        unsigned long cycle_start_micros = 0;
        unsigned long total_micros = 0;
};

