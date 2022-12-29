/*
 * Cyclic.cpp
 *
 *  Created on: Dec 23, 2022
 *      Author: cre
 */

#include "Plugin.hpp"

#include "Arduino.h"

std::ostream& operator<< (std::ostream &lhs, PluginId id)
{
    switch (id)
    {
        case COMMAND_PLUGIN:
            lhs << "COMMAND_PLUGIN";
            break;
        case DEMO_PLUGIN:
            lhs << "DEMO_PLUGIN";
            break;
        case WALL_PLUGIN:
            lhs << "WALL_PLUGIN";
            break;
        case FORWARD_PLUGIN:
            lhs << "FORWARD_PLUGIN";
            break;
        case REVERSE_PLUGIN:
            lhs << "REVERSE_PLUGIN";
            break;
        case CLOCKWISE_PLUGIN:
            lhs << "CLOCKWISE_PLUGIN";
            break;
        case COUNTERCLOCKWISE_PLUGIN:
            lhs << "COUNTERCLOCKWISE_PLUGIN";
            break;
        default:
            lhs << "???_PLUGIN";
            break;

    }
    return lhs;
}

Plugin::Plugin (const PluginId id) : id(id)
{
}


Plugin::~Plugin ()
{
}

PluginId Plugin::get_id ()
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

void Plugin::set_mode (Mode mode)
{
}

bool Plugin::is_enabled ()
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
