/*
 * Cyclic.cpp
 *
 *  Created on: Dec 23, 2022
 *      Author: cre
 */

#include "Plugin.hpp"

#include "Arduino.h"

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

        case FORWARD_MODE:
            return FORWARD_PLUGIN;

        case REVERSE_MODE:
            return REVERSE_PLUGIN;

        case CLOCKWISE_MODE:
            return CLOCKWISE_PLUGIN;

        case COUNTERCLOCKWISE_MODE:
            return COUNTERCLOCKWISE_PLUGIN;

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

        case FORWARD_PLUGIN:
            return FORWARD_MODE;

        case REVERSE_PLUGIN:
            return REVERSE_MODE;

        case CLOCKWISE_PLUGIN:
            return CLOCKWISE_MODE;

        case COUNTERCLOCKWISE_PLUGIN:
            return COUNTERCLOCKWISE_MODE;

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

Mode Plugin::get_mode ()
{
    return mode;
}

void Plugin::set_mode (Mode _mode)
{
    mode = _mode;
    Serial.print("Cyclic mode: ");
    Serial.println(_mode);
}

void Plugin::cycle ()
{
}
