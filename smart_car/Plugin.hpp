/*
 * Cyclic.hpp
 *
 *  Created on: Dec 23, 2022
 *      Author: cre
 */

#pragma once

#include <string>
#include "Mode.hpp"

enum PluginId
{
    CLOCK_PLUGIN,
    CLOCKWISE_PLUGIN,
    COMMAND_PLUGIN,
    COUNTERCLOCKWISE_PLUGIN,
    DEMO_PLUGIN,
    FORWARD_PLUGIN,
    IMU_PLUGIN,
    NAVIGATION_PLUGIN,
    REVERSE_PLUGIN,
    ULTRASOUND_PLUGIN,
    WALL_PLUGIN
};

const std::string stringify(const PluginId id);

std::ostream& operator<< (std::ostream &lhs, PluginId id);

class Plugin
{
    public:
        Plugin (const PluginId id);
        virtual ~Plugin ();

        const PluginId get_id () const;
        virtual void set_mode (Mode mode);
        virtual bool setup ();
        virtual void reset ();
        virtual void cycle ();
        const bool is_enabled () const;
        virtual void set_enabled (const bool enable);

    private:
        const PluginId id;
        bool enable = false;
};

