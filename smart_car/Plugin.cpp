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

PluginId translate_mode (const Mode mode)
{
    switch (mode)
    {
        case COMMAND_MODE:
            return COMMAND_PLUGIN;

        case DEMO_MODE:
            return DEMO_PLUGIN;

        case WALL_MODE:
            return WALL_PLUGIN;

        default:
            return COMMAND_PLUGIN;
    }
}

Mode translate_mode (const PluginId id)
{
    switch (id)
    {
        case COMMAND_PLUGIN:
            return COMMAND_MODE;

        case DEMO_PLUGIN:
            return DEMO_MODE;

        case WALL_PLUGIN:
            return WALL_MODE;

        default:
            return COMMAND_MODE;
    }
}

Plugin::Plugin (const PluginId id) : id(id)
{
}

Plugin::Plugin (const Mode mode_id) : id(translate_mode(mode_id))
{

}

Plugin::~Plugin ()
{
}

PluginId Plugin::get_id ()
{
    return id;
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
