/*
 * Plug.cpp
 *
 *  Created on: Jan 22, 2023
 *      Author: cre
 */

#include <WString.h>
#include "Logger.hpp"

#include "Plug.hpp"

static logging::Logger logger(__FILE__, logging::Level::info);

Plug::Plug ()
{
}

std::ostream& operator<< (std::ostream &lhs, const Plug &plug)
{
    return lhs << F("#[plug ") << plug.plugins.size() << F("]");
}

void Plug::add_plugin (Plugin *plugin)
{
    Serial.println(F("_add_plugin_"));

    logger.info(__LINE__) << F("add_plugin ") << plugin << std::endl;
    available_plugins.push_back(plugin);
    if (plugin->setup())
    {
        logger.info(__LINE__) << F("Setup ") << plugin << std::endl;
        plugins.push_back(plugin);
        plugin->enter_state(Plugin::DISABLE);
        if (plugin->is_cyclic())
        {
            cyclic_plugins.push_back(plugin);
        }
    }
    else
    {
        logger.info(__LINE__) << F("Disabled ") << plugin << std::endl;
    }
}

std::vector<Plugin*>& Plug::get_available_plugins ()
{
    return available_plugins;
}

std::vector<Plugin*>& Plug::get_plugins ()
{
    return plugins;
}

std::vector<Plugin*>& Plug::get_cyclic_plugins ()
{
    return cyclic_plugins;
}

Plugin* Plug::get_plugin (const PluginId id) const
{
    for (Plugin *const plugin : cyclic_plugins)
    {
        if (plugin->get_id() == id)
        {
            return plugin;
        }
    }
    return nullptr;
}

void Plug::cycle ()
{
    const unsigned long cycle_start_us = micros();

    for (Plugin *const plugin : cyclic_plugins)
    {
        plugin->major_cycle();
    }

    total_cycle_us += (micros() - cycle_start_us);
    cycle_count++;
}
