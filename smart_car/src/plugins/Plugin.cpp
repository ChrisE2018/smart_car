/*
 * Cyclic.cpp
 *
 *  Created on: Dec 23, 2022
 *      Author: cre
 */

#include "Plugin.hpp"

#include "Arduino.h"

const std::string stringify (const PluginId id)
{
    switch (id)
    {
        case CLOCK_PLUGIN:
            return "CLOCK_PLUGIN";
        case CLOCKWISE_PLUGIN:
            return "CLOCKWISE_PLUGIN";
        case COUNTERCLOCKWISE_PLUGIN:
            return "COUNTERCLOCKWISE_PLUGIN";
        case DEMO_PLUGIN:
            return "DEMO_PLUGIN";
        case FORWARD_PLUGIN:
            return "FORWARD_PLUGIN";
        case GOAL_PLUGIN:
            return "GOAL_PLUGIN";
        case MPU_PLUGIN:
            return "MPU_PLUGIN";
        case KALMAN_PLUGIN:
            return "KALMAN_PLUGIN";
        case ODOM_PLUGIN:
            return "ODOM_PLUGIN";
        case REVERSE_PLUGIN:
            return "REVERSE_PLUGIN";
        case ULTRASOUND_PLUGIN:
            return "ULTRASOUND_PLUGIN";
        case WALL_PLUGIN:
            return "WALL_PLUGIN";
        case COMMAND_CYCLE:
            return "COMMAND_CYCLE";
        case IDLE_CYCLE:
            return "IDLE_CYCLE";
        default:
            return "???_PLUGIN";
    }
}

std::ostream& operator<< (std::ostream &lhs, PluginId id)
{
    lhs << stringify(id).c_str();
    return lhs;
}

Plugin::Plugin (const PluginId id) : id(id)
{
}

Plugin::~Plugin ()
{
}

const PluginId Plugin::get_id () const
{
    return id;
}

bool Plugin::setup ()
{
    return true;
}

bool Plugin::reset ()
{
    return setup();
}

const bool Plugin::is_enabled () const
{
    return enable;
}

void Plugin::set_enabled (const bool _enable)
{
    enable = _enable;
}

int Plugin::get_preferred_interval () const
{
    return 100;
}

int Plugin::get_expected_ms () const
{
    return 0;
}

void Plugin::cycle ()
{
}

void Plugin::start_cycle ()
{
    cycle_start_micros = micros();
}

void Plugin::end_cycle ()
{
    const unsigned long cycle_micros = micros() - cycle_start_micros;
    total_micros += cycle_micros;
    cycle_count++;
}

unsigned long Plugin::get_cycle_count () const
{
    return cycle_count;
}

unsigned long Plugin::get_total_micros () const
{
    return total_micros;
}
