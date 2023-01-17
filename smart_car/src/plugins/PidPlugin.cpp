/*
 * PidPlugin.cpp
 *
 *  Created on: Jan 17, 2023
 *      Author: cre
 */

#include <Arduino.h>
#include "PidPlugin.hpp"
#include "../robot/Car.hpp"
#include "../logging/Logger.hpp"
#include "../robot/speed_counter.hpp"

static Logger logger(__FILE__, Level::info);

PidPlugin::PidPlugin (const PluginId id, MotorPlugin &motor_plugin) :
        Plugin(id), motor_plugin(motor_plugin), location(motor_plugin.get_location())
{
}

std::ostream& operator<< (std::ostream &lhs, const PidPlugin &motor)
{
    return lhs << "#[pid " << motor.location << " " << motor.measured_velocity << " m/s err=" << motor.velocity_error
            << "]";
}

bool PidPlugin::setup ()
{
    logger.data() << F("k0: ") << k0 << F(" k1: ") << k1 << F(" k2: ") << k2 << F(" k3: ") << k3 << F(" k4: ") << k4
            << std::endl;
    return true;
}

MotorLocation PidPlugin::get_location () const
{
    return location;
}

unsigned long PidPlugin::get_speed_counter () const
{
    return speed_counter;
}

float PidPlugin::get_measured_velocity () const
{
    return measured_velocity;
}

float PidPlugin::get_desired_velocity () const
{
    return desired_velocity;
}

void PidPlugin::set_desired_velocity (const float _desired_velocity)
{
    auto_velocity = true;
    if (desired_velocity != _desired_velocity)
    {
        desired_velocity = _desired_velocity;
        cumulative_velocity_error = 0;
    }
}

void PidPlugin::cancel_auto_velocity ()
{
    auto_velocity = false;
    desired_velocity = 0;
    velocity_error = 0;
    cumulative_velocity_error = 0;
    LOG_INFO(logger, "cancel_auto_velocity %s", stringify(location).c_str());
}

void PidPlugin::drive_stop ()
{
    motor_plugin.set_speed(0);
    cancel_auto_velocity();
    LOG_INFO(logger, "drive_stop %s", stringify(location).c_str());
}

float PidPlugin::get_velocity_error () const
{
    return velocity_error;
}

int PidPlugin::get_preferred_interval () const
{
    return 100;
}

int PidPlugin::get_expected_us () const
{
    return 25000;
}

int PidPlugin::sign (const float direction) const
{
    if (direction < 0)
    {
        return -1;
    }
    return 1;
}

void PidPlugin::cycle ()
{
    const unsigned long now = millis();
    const unsigned long delta_ms = now - last_cycle_ms;
    const long raw_speed_ticks = get_speed_counter_value(location);
    const long measured_speed_counter = sign(desired_velocity) * raw_speed_ticks;
    const long speed_ticks = measured_speed_counter - speed_counter;
    if (delta_ms > minimum_cycle_ms || abs(speed_ticks) > minimum_speed_ticks)
    {
        const float delta_seconds = delta_ms * 0.001;
        const long previous_speed_counter = speed_counter;
        speed_counter = measured_speed_counter;
        const double measured_distance = speed_ticks * count_to_meters;
        measured_velocity = measured_distance / delta_seconds;
        if (abs(measured_velocity) > 0)
        {
            logger.data() << F("/motor/") << stringify(location).c_str() << F("/delta_seconds,") << delta_seconds
                    << F(",speed_counter,") << speed_counter << F(",measured_velocity,") << measured_velocity
                    << F(",measured_distance,") << measured_distance << F(",raw_speed_ticks,") << raw_speed_ticks
                    << std::endl;
        }
        if (auto_velocity)
        {
            const float previous_velocity_error = velocity_error;
            velocity_error = desired_velocity - measured_velocity;
            const float velocity_error_rate = (velocity_error - previous_velocity_error) / delta_seconds;
            cumulative_velocity_error += velocity_error * delta_seconds;
            const float smallness = std::max(0.0f, k4 - abs(measured_velocity));
            const float control_raw = k0
                    * (velocity_error + k1 * cumulative_velocity_error + k2 * velocity_error_rate
                            + smallness * velocity_error * k3);
            // No reversing
            const float control = desired_velocity >= 0.0f ? std::max(0.0f, control_raw) : std::min(0.0f, control_raw);
            logger.data() << F("/motor/") << stringify(location).c_str() << F("/velocity_error,") << velocity_error
                    << F(",cumulative_velocity_error,") << cumulative_velocity_error << F(",velocity_error_rate,")
                    << velocity_error_rate << F(",smallness,") << smallness << F(",control,") << control << std::endl;
            motor_plugin.set_speed(control);
        }
        last_cycle_ms = now;
    }
}
