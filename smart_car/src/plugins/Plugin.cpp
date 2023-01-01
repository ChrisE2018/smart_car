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
        case COMMAND_PLUGIN:
            return "COMMAND_PLUGIN";
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
            return "KALMAN_PLUGIN244";
        case ODOM_PLUGIN:
            return "ODOM_PLUGIN";
        case REVERSE_PLUGIN:
            return "REVERSE_PLUGIN";
        case ULTRASOUND_PLUGIN:
            return "ULTRASOUND_PLUGIN";
        case WALL_PLUGIN:
            return "WALL_PLUGIN";
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

void Plugin::reset ()
{
}

const bool Plugin::is_enabled () const
{
    return enable;
}

void Plugin::set_enabled (const bool _enable)
{
    enable = _enable;
}

void Plugin::cycle ()
{
}
