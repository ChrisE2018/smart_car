/*
 * Plug.hpp
 *
 *  Created on: Jan 22, 2023
 *      Author: cre
 */

#pragma once

#include <vector>
#include "Plugin.hpp"

class Plug
{
    public:
        Plug ();
        friend std::ostream& operator<< (std::ostream &os, const Plug &car);
        void add_plugin (Plugin *plugin);
        std::vector<Plugin*> &get_available_plugins ();
        std::vector<Plugin*> &get_plugins ();
        std::vector<Plugin*> &get_cyclic_plugins ();
        void cycle ();

    private:
        unsigned long cycle_count = 0;
        unsigned long total_cycle_us = 0;
        std::vector<Plugin*> available_plugins;
        std::vector<Plugin*> plugins;
        std::vector<Plugin*> cyclic_plugins;
        Plugin* get_plugin (const PluginId id) const;
};

