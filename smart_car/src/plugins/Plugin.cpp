/*
 * Plugin.cpp
 *
 *  Created on: Dec 23, 2022
 *      Author: cre
 */

#include <Arduino.h>
#include "Plugin.hpp"

const std::string stringify (const PluginId id)
{
    switch (id)
    {
        case PluginId::CLOCK_PLUGIN:
            return "CLOCK_PLUGIN";
        case PluginId::CLOCKWISE_PLUGIN:
            return "CLOCKWISE_PLUGIN";
        case PluginId::COUNTERCLOCKWISE_PLUGIN:
            return "COUNTERCLOCKWISE_PLUGIN";
        case PluginId::DEMO_PLUGIN:
            return "DEMO_PLUGIN";
        case PluginId::FORWARD_PLUGIN:
            return "FORWARD_PLUGIN";
        case PluginId::GOAL_PLUGIN:
            return "GOAL_PLUGIN";
        case PluginId::MPU_PLUGIN:
            return "MPU_PLUGIN";
        case PluginId::KALMAN_PLUGIN:
            return "KALMAN_PLUGIN";
        case PluginId::ODOM_PLUGIN:
            return "ODOM_PLUGIN";
        case PluginId::MOTOR_LEFT_FRONT_PLUGIN:
            return "MOTOR_LEFT_FRONT_PLUGIN";
        case PluginId::MOTOR_RIGHT_FRONT_PLUGIN:
            return "MOTOR_RIGHT_FRONT_PLUGIN";
        case PluginId::MOTOR_LEFT_REAR_PLUGIN:
            return "MOTOR_LEFT_REAR_PLUGIN";
        case PluginId::MOTOR_RIGHT_REAR_PLUGIN:
            return "MOTOR_RIGHT_REAR_PLUGIN";
        case PluginId::REVERSE_PLUGIN:
            return "REVERSE_PLUGIN";
        case PluginId::ULTRASOUND_PLUGIN:
            return "ULTRASOUND_PLUGIN";
        case PluginId::WALL_PLUGIN:
            return "WALL_PLUGIN";
        case PluginId::COMMAND_CYCLE:
            return "COMMAND_CYCLE";
        case PluginId::IDLE_CYCLE:
            return "IDLE_CYCLE";
        default:
            return "???_PLUGIN";
    }
}

std::ostream& operator<< (std::ostream &lhs, const PluginId id)
{
    lhs << stringify(id).c_str();
    return lhs;
}

Plugin::Plugin (const PluginId id) :
        id(id)
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

bool Plugin::is_cyclic () const
{
    return true;
}

bool Plugin::is_enabled () const
{
    return enable;
}

void Plugin::set_enabled (const bool _enable)
{
    enable = _enable;
}

bool Plugin::is_trace () const
{
    return enable_trace;
}

void Plugin::set_trace (const bool _trace)
{
    enable_trace = _trace;
}

int Plugin::get_preferred_interval () const
{
    return 100;
}

int Plugin::get_expected_us () const
{
    return 25;
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
    const unsigned long expected_us = get_expected_us();
    if (cycle_micros > expected_us)
    {
        overrun_count++;
        if (overrun_count % 100 == 0)
        {
            Serial.print(stringify(id).c_str());
            Serial.print(F(" overrun "));
            Serial.print(cycle_micros);
            Serial.print(F(" > "));
            Serial.print(expected_us);
            Serial.println(F(" micros"));
        }
    }
}

void Plugin::trace ()
{
}

unsigned long Plugin::get_cycle_count () const
{
    return cycle_count;
}

unsigned long Plugin::get_total_micros () const
{
    return total_micros;
}

unsigned long Plugin::get_overrun_count () const
{
    return overrun_count;
}

int Plugin::get_state () const
{
    return state;
}

void Plugin::set_state (const int _state)
{
    exit_state(state);
    enter_state(_state);
}

void Plugin::enter_state (const int state)
{

}

void Plugin::exit_state (const int state)
{

}
