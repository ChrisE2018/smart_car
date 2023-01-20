/*
 * Car.cpp
 *
 *  Created on: Dec 23, 2022
 *      Author: cre
 */

#include "Car.hpp"
#include "../logging/RobotAppender.hpp"
#include "../logging/SerialAppender.hpp"
#include "smart_car.hpp"

#include "../plugins/ClockPlugin.hpp"
#include "../plugins/DrivePlugin.hpp"
#include "../plugins/GoalPlugin.hpp"
#include "../plugins/KalmanPlugin.hpp"
#include "../plugins/MpuPlugin.hpp"
#include "../plugins/OdomPlugin.hpp"
#include "../plugins/PidPlugin.hpp"
#include "../plugins/UltrasoundPlugin.hpp"
#include "../plugins/WallPlugin.hpp"

// For some reason this does not always resolve.
extern HardwareSerial Serial;

extern RobotAppender *robot_appender;
extern SerialAppender *usb_appender;

Car::Car () :
        logger(__FILE__, Level::debug), serial_parser(Serial), bluetooth_parser(Serial3), clock_plugin(
                new ClockPlugin()), forward_plugin(
                new DrivePlugin(PluginId::FORWARD_PLUGIN, *this, 500, MotorDirection::FORWARD,
                        MotorDirection::FORWARD)), goal_plugin(new GoalPlugin(*this)), mpu_plugin(new MpuPlugin()), kalman_plugin(
                new KalmanPlugin(*this)), odom_plugin(new OdomPlugin(*this)), reverse_plugin(
                new DrivePlugin(PluginId::REVERSE_PLUGIN, *this, 500, MotorDirection::REVERSE,
                        MotorDirection::REVERSE)), ultrasound_plugin(new UltrasoundPlugin(*this)), wall_plugin(
                new WallPlugin(*this))
{
    available_plugins.push_back(clock_plugin);
    available_plugins.push_back(forward_plugin);
    available_plugins.push_back(goal_plugin);
    available_plugins.push_back(reverse_plugin);
    available_plugins.push_back(&motors[static_cast<int>(MotorLocation::RIGHT_FRONT)]);
    available_plugins.push_back(&motors[static_cast<int>(MotorLocation::LEFT_FRONT)]);
    available_plugins.push_back(&motors[static_cast<int>(MotorLocation::RIGHT_REAR)]);
    available_plugins.push_back(&motors[static_cast<int>(MotorLocation::LEFT_REAR)]);
    available_plugins.push_back(&pid_controls[static_cast<int>(MotorLocation::RIGHT_FRONT)]);
    available_plugins.push_back(&pid_controls[static_cast<int>(MotorLocation::LEFT_FRONT)]);
    available_plugins.push_back(&pid_controls[static_cast<int>(MotorLocation::RIGHT_REAR)]);
    available_plugins.push_back(&pid_controls[static_cast<int>(MotorLocation::LEFT_REAR)]);
    available_plugins.push_back(mpu_plugin);
    available_plugins.push_back(kalman_plugin);
    available_plugins.push_back(odom_plugin);
    available_plugins.push_back(ultrasound_plugin);
    available_plugins.push_back(wall_plugin);
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
    return lhs << F("#[car ") << car.mode << F(" ") << car.kalman_plugin->get_x() << F(", ") << car.kalman_plugin->get_y()
            << F(" ang ") << car.kalman_plugin->get_angle() << F("]");
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
            wall_plugin->set_state(Plugin::DISABLE);
            break;
        case Mode::DEMO_MODE:
            all_stop();
            wall_plugin->set_state(Plugin::DISABLE);
            break;
        case Mode::GOAL_MODE:
            all_stop();
            wall_plugin->set_state(Plugin::DISABLE);
            break;
        case Mode::WALL_MODE:
            all_stop();
            wall_plugin->set_state(Plugin::ENABLE);
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
    if (robot_appender == nullptr)
    {
        serial_parser.handle_command(*this);
        bluetooth_parser.handle_command(*this);
    }
    else
    {
        if (serial_parser.has_input())
        {
            robot_appender->enable_usb_logger(true);
            robot_appender->enable_bluetooth_logger(false);
            serial_parser.handle_command(*this);
        }
        if (bluetooth_parser.has_input())
        {
            usb_appender->set_level(Level::none);
            robot_appender->enable_usb_logger(false);
            robot_appender->enable_bluetooth_logger(true);
            bluetooth_parser.handle_command(*this);
        }
        robot_appender->flush();
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
    forward_plugin->set_state(Plugin::DISABLE);
    reverse_plugin->set_state(Plugin::DISABLE);
    goal_plugin->set_state(Plugin::DISABLE);
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

ClockPlugin* Car::get_clock_plugin () const
{
    return clock_plugin;
}

DrivePlugin* Car::get_forward_plugin () const
{
    return forward_plugin;
}

GoalPlugin* Car::get_goal_plugin () const
{
    return goal_plugin;
}

DrivePlugin* Car::get_reverse_plugin () const
{
    return reverse_plugin;
}

MpuPlugin* Car::get_mpu_plugin () const
{
    return mpu_plugin;
}

KalmanPlugin* Car::get_kalman_plugin () const
{
    return kalman_plugin;
}

OdomPlugin* Car::get_odom_plugin () const
{
    return odom_plugin;
}

const PidPlugin& Car::get_pid_plugin (const MotorLocation location) const
{
    return pid_controls[static_cast<int>(location)];
}

PidPlugin& Car::get_pid_plugin (const MotorLocation location)
{
    return pid_controls[static_cast<int>(location)];
}

UltrasoundPlugin* Car::get_ultrasound_plugin () const
{
    return ultrasound_plugin;
}

WallPlugin* Car::get_wall_plugin () const
{
    return wall_plugin;
}

Plugin* Car::get_plugin (const PluginId id) const
{
    switch (id)
    {
        case PluginId::CLOCK_PLUGIN:
            return clock_plugin;
        case PluginId::FORWARD_PLUGIN:
            return forward_plugin;
        case PluginId::GOAL_PLUGIN:
            return goal_plugin;
        case PluginId::KALMAN_PLUGIN:
            return kalman_plugin;
        case PluginId::MOTOR_RIGHT_FRONT_PLUGIN:
            return &motors[static_cast<int>(MotorLocation::RIGHT_FRONT)];
        case PluginId::MOTOR_LEFT_FRONT_PLUGIN:
            return &motors[static_cast<int>(MotorLocation::LEFT_FRONT)];
        case PluginId::MOTOR_RIGHT_REAR_PLUGIN:
            return &motors[static_cast<int>(MotorLocation::RIGHT_REAR)];
        case PluginId::MOTOR_LEFT_REAR_PLUGIN:
            return &motors[static_cast<int>(MotorLocation::LEFT_REAR)];
        case PluginId::MPU_PLUGIN:
            return mpu_plugin;
        case PluginId::ODOM_PLUGIN:
            return odom_plugin;
        case PluginId::REVERSE_PLUGIN:
            return reverse_plugin;
        case PluginId::ULTRASOUND_PLUGIN:
            return ultrasound_plugin;
        case PluginId::WALL_PLUGIN:
            return wall_plugin;
        default:
            return nullptr;
    }
}
