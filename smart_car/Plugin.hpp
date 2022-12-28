/*
 * Cyclic.hpp
 *
 *  Created on: Dec 23, 2022
 *      Author: cre
 */

#pragma once

#include "Mode.hpp"

enum PluginId
{
    COMMAND_PLUGIN,
    DEMO_PLUGIN,
    WALL_PLUGIN,
    FORWARD_PLUGIN,
    REVERSE_PLUGIN,
    CLOCKWISE_PLUGIN,
    COUNTERCLOCKWISE_PLUGIN,
    IMU_PLUGIN,
    ULTRASOUND_PLUGIN,
    CLOCK_PLUGIN
};

std::ostream& operator<< (std::ostream &lhs, PluginId id);

class Plugin
{
    public:
        Plugin (const PluginId id);
        virtual ~Plugin ();

        PluginId get_id ();
        virtual void set_mode (Mode mode);
        virtual bool setup ();
        virtual void cycle ();
        bool is_enabled ();
        virtual void set_enabled (const bool enable);

    private:
        const PluginId id;
        bool enable = false;
};

