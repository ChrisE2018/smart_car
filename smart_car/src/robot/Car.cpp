/*
 * Car.cpp
 *
 *  Created on: Dec 23, 2022
 *      Author: cre
 */

#include "Car.hpp"
#include "../logging/RobotAppender.hpp"
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
    return lhs << "#[car " << car.mode << " " << car.kalman_plugin->get_x() << ", " << car.kalman_plugin->get_y()
            << " ang " << car.kalman_plugin->get_angle() << "]";
}

void Car::setup ()
{
    LOG_INFO(logger, "Attempting setup of %d available plugins", available_plugins.size());
    for (Plugin *const plugin : available_plugins)
    {
        if (plugin->setup())
        {
            logger.info(__LINE__) << F("Setup ") << plugin->get_id() << std::endl;
            plugins.push_back(plugin);
            plugin->enter_state(0);
        }
        else
        {
            logger.info(__LINE__) << F("Disabled ") << plugin->get_id() << std::endl;
        }
    }
    for (int i = 0; i < schedule_size; i++)
    {
        schedule[i] = PluginId::IDLE_CYCLE;
    }
    int cycle = 0;
    int scheduled_count = 0;
    //while (cycle < schedule_size)
    {
        for (Plugin *plugin : available_plugins)
        {
            if (plugin->is_cyclic())
            {
                const int max_ms = 999 + plugin->get_expected_us();
                const int expected_ms = max_ms / 1000;
                if (cycle < schedule_size - expected_ms)
                {
                    const PluginId id = plugin->get_id();
                    const int actual_interval = get_actual_interval(cycle, id);
                    const int preferred_interval = plugin->get_preferred_interval();
                    if (actual_interval >= preferred_interval)
                    {
                        scheduled_count++;
                        schedule[cycle++] = id;
                        for (int i = 1; i < expected_ms; i++)
                        {
                            schedule[cycle++] = PluginId::IDLE_CYCLE;
                        }
                    }
                }
            }
            else
            {
                scheduled_count++;
            }
        }
        if (cycle < schedule_size - 3)
        {
            schedule[cycle++] = PluginId::IDLE_CYCLE;
            schedule[cycle++] = PluginId::COMMAND_CYCLE;
            schedule[cycle++] = PluginId::IDLE_CYCLE;
            schedule[cycle++] = PluginId::IDLE_CYCLE;
        }
        else
        {
            LOG_ERROR(logger, "Could not schedule command cycle in %d slots", schedule_size);
        }
    }
    if (scheduled_count < plugins.size())
    {
        LOG_WARNING(logger, "Scheduled only %d of %d enabled plugins", scheduled_count, plugins.size());
    }
    else
    {
        LOG_INFO(logger, "Scheduled all %d enabled plugins", plugins.size());
    }

    logger.info(__LINE__) << F("Testing info logging") << std::endl;
    logger.debug(__LINE__) << F("Testing debug logging") << std::endl;
}

int Car::get_actual_interval (const int cycle, const PluginId id) const
{
    for (int i = cycle; i >= 0; i--)
    {
        if (schedule[i] == id)
        {
            return cycle - i;
        }
    }
    return 100;
}

void Car::set_mode (const Mode _mode)
{
    mode = _mode;
    logger.info(__LINE__) << F("Set ") << *this << F(" to ") << _mode << std::endl;
    switch (mode)
    {
        case Mode::COMMAND_MODE:
            all_stop();
            wall_plugin->set_enabled(false);
            break;
        case Mode::DEMO_MODE:
            all_stop();
            wall_plugin->set_enabled(false);
            break;
        case Mode::GOAL_MODE:
            all_stop();
            wall_plugin->set_enabled(false);
            break;
        case Mode::WALL_MODE:
            all_stop();
            wall_plugin->set_enabled(true);
            break;
    }
}

void Car::cycle ()
{
    if (use_simple_cycle)
    {
        simple_cycle();
    }
    else
    {
        schedule_cycle();
    }
}

void Car::schedule_cycle ()
{
    const unsigned long cycle_start_us = micros();
    const int ms = millis() % 1000;
    const int cycle = ms % schedule_size;

    const PluginId scheduled_plugin = schedule[cycle];
    if (scheduled_plugin == PluginId::COMMAND_CYCLE)
    {
        command_cycle();
    }
    else
    {
        Plugin *const plugin = get_plugin(scheduled_plugin);
        if (plugin != nullptr)
        {
            plugin->start_cycle();
            plugin->cycle();
            plugin->end_cycle();
            if (plugin->is_trace())
            {
                plugin->trace();
            }
        }
    }

    total_cycle_us += (micros() - cycle_start_us);
    cycle_count++;
}

void Car::simple_cycle ()
{
    const unsigned long cycle_start_us = micros();

    for (Plugin *const plugin : plugins)
    {
        plugin->start_cycle();
        plugin->cycle();
        plugin->end_cycle();
        if (plugin->is_trace())
        {
            plugin->trace();
        }
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
    forward_plugin->set_enabled(false);
    reverse_plugin->set_enabled(false);
    goal_plugin->set_enabled(false);
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

float Car::get_measured_velocity (const MotorLocation motor) const
{
    return pid_controls[static_cast<int>(motor)].get_measured_velocity();
}

float Car::get_desired_velocity (const MotorLocation motor) const
{
    return pid_controls[static_cast<int>(motor)].get_desired_velocity();
}

void Car::set_desired_velocity (const MotorLocation motor, const float velocity)
{
    pid_controls[static_cast<int>(motor)].set_desired_velocity(velocity);
}

MotorDirection Car::get_drive_direction (const MotorLocation motor) const
{
    return motors[static_cast<int>(motor)].get_direction();
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
