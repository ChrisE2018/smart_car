/*
 * Car.cpp
 *
 *  Created on: Dec 23, 2022
 *      Author: cre
 */

#include "Car.hpp"
#include "SerialAppender.hpp"
#include "smart_car.hpp"

#include "../plugins/Robot.hpp"

// For some reason this does not always resolve.
extern HardwareSerial Serial;

extern logging::SerialAppender *usb_appender;
extern logging::SerialAppender *bluetooth_appender;

Car::Car () :
        logger(__FILE__, logging::Level::debug), serial_parser(Serial), bluetooth_parser(Serial3)
{
    available_plugins.push_back(&motors[static_cast<int>(MotorLocation::RIGHT_FRONT)]);
    available_plugins.push_back(&motors[static_cast<int>(MotorLocation::LEFT_FRONT)]);
    available_plugins.push_back(&motors[static_cast<int>(MotorLocation::RIGHT_REAR)]);
    available_plugins.push_back(&motors[static_cast<int>(MotorLocation::LEFT_REAR)]);
    available_plugins.push_back(&pid_controls[static_cast<int>(MotorLocation::RIGHT_FRONT)]);
    available_plugins.push_back(&pid_controls[static_cast<int>(MotorLocation::LEFT_FRONT)]);
    available_plugins.push_back(&pid_controls[static_cast<int>(MotorLocation::RIGHT_REAR)]);
    available_plugins.push_back(&pid_controls[static_cast<int>(MotorLocation::LEFT_REAR)]);
}

Car::~Car ()
{
    for (Plugin *plugin : plugins)
    {
        delete plugin;
    }
    plugins.clear();
}

std::ostream& operator<< (std::ostream &lhs, const Car &car)
{
    return lhs << F("#[car ") << car.mode << F(" ") << kalman_plugin.get_x() << F(", ") << kalman_plugin.get_y()
            << F(" ang ") << kalman_plugin.get_angle() << F("]");
}

void Car::setup ()
{
    LOG_INFO(logger, "Attempting setup of %d available plugins", available_plugins.size());
    for (Plugin *const plugin : available_plugins)
    {
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

    logger.info(__LINE__) << F("Testing info logging") << std::endl;
    logger.debug(__LINE__) << F("Testing debug logging") << std::endl;
}

void Car::set_mode (const Mode _mode)
{
    mode = _mode;
    logger.info(__LINE__) << F("Set ") << *this << F(" to ") << _mode << std::endl;
    switch (mode)
    {
        case Mode::COMMAND_MODE:
            all_stop();
            wall_plugin.set_state(Plugin::DISABLE);
            break;
        case Mode::DEMO_MODE:
            all_stop();
            wall_plugin.set_state(Plugin::DISABLE);
            break;
        case Mode::GOAL_MODE:
            all_stop();
            wall_plugin.set_state(Plugin::DISABLE);
            break;
        case Mode::WALL_MODE:
            all_stop();
            wall_plugin.set_state(Plugin::ENABLE);
            break;
    }
}

void Car::cycle ()
{
    const unsigned long cycle_start_us = micros();

    for (Plugin *const plugin : cyclic_plugins)
    {
        plugin->major_cycle();
    }
    command_cycle();

    total_cycle_us += (micros() - cycle_start_us);
    cycle_count++;
}

void Car::command_cycle ()
{
    if (serial_parser.has_input())
    {
        usb_appender->set_level(logging::Level::info);
        bluetooth_appender->set_level(logging::Level::none);
        serial_parser.handle_command(*this);
    }
    if (bluetooth_parser.has_input())
    {
        usb_appender->set_level(logging::Level::none);
        usb_appender->set_level(logging::Level::none);
        bluetooth_appender->set_level(logging::Level::info);
        bluetooth_parser.handle_command(*this);
    }
}

void Car::demo_drive_leds ()
{
    int duration = 150;
    for (int i = 0; i < 3; i++)
    {
        for (int motor = 0; motor < MOTOR_COUNT; motor++)
        {
            motors[motor].led_demo(duration);
        }
        duration /= 2;
    }
}

void Car::all_stop ()
{
    for (int motor = 0; motor < MOTOR_COUNT; motor++)
    {
        pid_controls[motor].drive_stop();
    }
    forward_plugin.set_state(Plugin::DISABLE);
    reverse_plugin.set_state(Plugin::DISABLE);
    goal_plugin.set_state(Plugin::DISABLE);
}

const MotorPlugin& Car::get_motor (const MotorLocation motor) const
{
    return motors[static_cast<int>(motor)];
}

MotorPlugin& Car::get_motor (const MotorLocation motor)
{
    return motors[static_cast<int>(motor)];
}

void Car::drive_stop (const MotorLocation motor)
{
    pid_controls[static_cast<int>(motor)].drive_stop();
}

void Car::set_speed (const MotorLocation motor, const int speed)
{
    motors[static_cast<int>(motor)].set_speed(speed);
}

void Car::set_right_speed (const int speed)
{
    motors[MotorLocation::RIGHT_FRONT].set_speed(speed);
    motors[MotorLocation::RIGHT_REAR].set_speed(speed);
}

void Car::set_left_speed (const int speed)
{
    motors[MotorLocation::LEFT_FRONT].set_speed(speed);
    motors[MotorLocation::LEFT_REAR].set_speed(speed);
}

int Car::get_drive_speed (const MotorLocation motor) const
{
    return motors[static_cast<int>(motor)].get_speed();
}

float Car::get_desired_velocity (const MotorLocation motor) const
{
    return pid_controls[static_cast<int>(motor)].get_desired_velocity();
}

void Car::set_desired_velocity (const MotorLocation motor, const float velocity)
{
    pid_controls[static_cast<int>(motor)].set_desired_velocity(velocity);
}

float Car::get_measured_velocity (const MotorLocation motor) const
{
    return pid_controls[static_cast<int>(motor)].get_measured_velocity();
}

float Car::get_cumulative_velocity_error (const MotorLocation motor) const
{
    return pid_controls[static_cast<int>(motor)].get_cumulative_velocity_error();
}

const PidPlugin& Car::get_pid_plugin (const MotorLocation location) const
{
    return pid_controls[static_cast<int>(location)];
}

PidPlugin& Car::get_pid_plugin (const MotorLocation location)
{
    return pid_controls[static_cast<int>(location)];
}

Plugin* Car::get_plugin (const PluginId id) const
{
    switch (id)
    {
        case PluginId::MOTOR_RIGHT_FRONT_PLUGIN:
            return &motors[static_cast<int>(MotorLocation::RIGHT_FRONT)];
        case PluginId::MOTOR_LEFT_FRONT_PLUGIN:
            return &motors[static_cast<int>(MotorLocation::LEFT_FRONT)];
        case PluginId::MOTOR_RIGHT_REAR_PLUGIN:
            return &motors[static_cast<int>(MotorLocation::RIGHT_REAR)];
        case PluginId::MOTOR_LEFT_REAR_PLUGIN:
            return &motors[static_cast<int>(MotorLocation::LEFT_REAR)];
        default:
            return nullptr;
    }
}
