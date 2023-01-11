/*
 * Car.cpp
 *
 *  Created on: Dec 23, 2022
 *      Author: cre
 */

#include "Car.hpp"

#include "smart_car.hpp"
#include "../logging/Logger.hpp"
#include "../robot/heap.hpp"

#include "../plugins/KalmanPlugin.hpp"
#include "../plugins/ClockPlugin.hpp"
#include "../plugins/DemoPlugin.hpp"
#include "../plugins/DrivePlugin.hpp"
#include "../plugins/GoalPlugin.hpp"
#include "../plugins/MpuPlugin.hpp"
#include "../plugins/OdomPlugin.hpp"
#include "../plugins/UltrasoundPlugin.hpp"
#include "../plugins/WallPlugin.hpp"

// For some reason this does not always resolve.
extern HardwareSerial Serial;

static Logger logger(__FILE__, Level::debug);

Car::Car () :
                serial_parser(Serial), bluetooth_parser(Serial3), clock_plugin(new ClockPlugin()),
                clockwise_plugin(
                        new DrivePlugin(PluginId::CLOCKWISE_PLUGIN, *this, 500,
                                MotorDirection::FORWARD, MotorDirection::REVERSE)),
                counterclockwise_plugin(
                        new DrivePlugin(PluginId::COUNTERCLOCKWISE_PLUGIN, *this, 500,
                                MotorDirection::REVERSE, MotorDirection::FORWARD)),
                demo_plugin(new DemoPlugin(*this)),
                forward_plugin(
                        new DrivePlugin(PluginId::FORWARD_PLUGIN, *this, 500,
                                MotorDirection::FORWARD, MotorDirection::FORWARD)),
                goal_plugin(new GoalPlugin(*this)), mpu_plugin(new MpuPlugin()),
                kalman_plugin(new KalmanPlugin(*this)), odom_plugin(new OdomPlugin(*this)),
                reverse_plugin(
                        new DrivePlugin(PluginId::REVERSE_PLUGIN, *this, 500,
                                MotorDirection::REVERSE, MotorDirection::REVERSE)),
                ultrasound_plugin(new UltrasoundPlugin(*this)), wall_plugin(new WallPlugin(*this))
{
    available_plugins.push_back(clock_plugin);
    available_plugins.push_back(demo_plugin);
    available_plugins.push_back(forward_plugin);
    available_plugins.push_back(goal_plugin);
    available_plugins.push_back(reverse_plugin);
    available_plugins.push_back(clockwise_plugin);
    available_plugins.push_back(counterclockwise_plugin);
    available_plugins.push_back(&motors[static_cast<int>(MotorLocation::RIGHT)]);
    available_plugins.push_back(&motors[static_cast<int>(MotorLocation::LEFT)]);
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
    return lhs << "#[car " << car.mode << " " << car.kalman_plugin->get_x() << ", "
            << car.kalman_plugin->get_y() << " ang " << car.kalman_plugin->get_angle() << "]";
}

void Car::setup ()
{
    LOG_INFO(logger, "Attempting setup of %d available plugins", available_plugins.size());
    for (Plugin *const plugin : available_plugins)
    {
        if (plugin->setup())
        {
            logger.info() << F("Setup ") << plugin->get_id() << std::endl;
            plugins.push_back(plugin);
        }
        else
        {
            logger.info() << F("Disabled ") << plugin->get_id() << std::endl;
        }
    }
    int cycle = 0;
    while (cycle < 100)
    {
        for (Plugin *plugin : available_plugins)
        {
            const int expected_cycles = plugin->get_expected_ms() / ms_per_cycle;
            if (cycle < 100 - expected_cycles)
            {
                const PluginId id = plugin->get_id();
                const int actual_interval = get_actual_interval(cycle, id);
                const int preferred_interval = plugin->get_preferred_interval() / ms_per_cycle;
                if (actual_interval >= preferred_interval)
                {
                    schedule[cycle++] = id;
                    for (int i = 0; i < expected_cycles; i++)
                    {
                        schedule[cycle++] = PluginId::IDLE_CYCLE;
                    }
                }
            }
        }
        if (cycle < 99)
        {
            schedule[cycle++] = PluginId::COMMAND_CYCLE;
            schedule[cycle++] = PluginId::IDLE_CYCLE;
        }
    }
    LOG_INFO(logger, "Completed setup of %d enabled plugins", plugins.size());

    logger.info() << F("Testing info logging") << std::endl;
    logger.debug() << F("Testing debug logging") << std::endl;
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
    logger.info() << F("Set ") << *this << F(" to ") << _mode << std::endl;
    switch (mode)
    {
        case Mode::COMMAND_MODE:
            all_stop();
            demo_plugin->set_enabled(false);
            wall_plugin->set_enabled(false);
            break;
        case Mode::DEMO_MODE:
            all_stop();
            demo_plugin->set_enabled(true);
            wall_plugin->set_enabled(false);
            break;
        case Mode::GOAL_MODE:
            all_stop();
            demo_plugin->set_enabled(false);
            wall_plugin->set_enabled(false);
            break;
        case Mode::WALL_MODE:
            all_stop();
            demo_plugin->set_enabled(false);
            wall_plugin->set_enabled(true);
            break;
    }
}

void Car::cycle ()
{
    const unsigned long cycle_start_us = micros();
    const int ms = (millis()) % 1000;
    const int cycle = ms / ms_per_cycle;

    const PluginId scheduled_plugin = schedule[cycle];
    if (scheduled_plugin == PluginId::COMMAND_CYCLE)
    {
        serial_parser.handle_command(*this);
        bluetooth_parser.handle_command(*this);
    }
    else
    {
        Plugin *const plugin = get_plugin(scheduled_plugin);
        if (plugin != nullptr)
        {
            plugin->start_cycle();
            plugin->cycle();
            plugin->end_cycle();
        }
    }

    total_cycle_us += (micros() - cycle_start_us);
    cycle_count++;
}

/** Execute a command from the user.
 @param words Command string broken into words.
 */
void Car::execute_command (const std::vector<String> &words)
{
    const int n = words.size();
    const String command = words[0];
    logger.info() << F("Command: ") << command.c_str() << std::endl;
    if (command == F("angle"))
    {
        if (n > 1)
        {
            set_mode(Mode::GOAL_MODE);
            goal_plugin->set_goal(words[1].toFloat());
        }
    }
    else if (command == F("b"))
    {
        int speed = SPEED_FULL;
        int duration = 500;
        if (n > 1)
        {
            speed = words[1].toInt();
        }
        if (n > 2)
        {
            duration = words[2].toInt();
        }
        set_mode(Mode::COMMAND_MODE);
        reverse_plugin->set_duration(duration);
        reverse_plugin->set_right_speed(speed);
        reverse_plugin->set_left_speed(speed);
        reverse_plugin->set_enabled(true);
    }
    else if (command == F("bb"))
    {
        float velocity = -1.0;
        if (n > 1)
        {
            velocity = -words[1].toFloat();
        }
        set_mode(Mode::COMMAND_MODE);
        motors[static_cast<int>(MotorLocation::RIGHT)].set_desired_velocity(velocity);
        motors[static_cast<int>(MotorLocation::LEFT)].set_desired_velocity(velocity);
    }
    else if (command == F("calibrate"))
    {
        mpu_plugin->calibrate();
    }
    else if (command == F("c"))
    {
        set_mode(Mode::COMMAND_MODE);
        logger.info() << F("Current mode is ") << mode << std::endl;
    }
    else if (command == F("clock"))
    {
        clock_plugin->set_enabled(!clock_plugin->is_enabled());
    }
    else if (command == F("demo"))
    {
        set_mode(Mode::DEMO_MODE);
        logger.info() << F("Current mode is ") << mode << std::endl;
    }
    else if (command == F("distance"))
    {
        ultrasound_plugin->set_enabled(!ultrasound_plugin->is_enabled());
    }
    else if (command == F("f"))
    {
        int speed = SPEED_FULL;
        int duration = 500;
        if (n > 1)
        {
            speed = words[1].toInt();
        }
        if (n > 2)
        {
            duration = words[2].toInt();
        }
        set_mode(Mode::COMMAND_MODE);
        forward_plugin->set_duration(duration);
        forward_plugin->set_right_speed(speed);
        forward_plugin->set_left_speed(speed);
        forward_plugin->set_enabled(true);
    }
    else if (command == F("ff"))
    {
        float velocity = 1.0;
        if (n > 1)
        {
            velocity = words[1].toFloat();
        }
        set_mode(Mode::COMMAND_MODE);
        motors[static_cast<int>(MotorLocation::RIGHT)].set_desired_velocity(velocity);
        motors[static_cast<int>(MotorLocation::LEFT)].set_desired_velocity(velocity);
    }
    else if (command == F("goal"))
    {
        if (n > 2)
        {
            set_mode(Mode::GOAL_MODE);
            goal_plugin->set_goal(words[1].toFloat(), words[2].toFloat());
        }
    }
    else if (command == F("heap"))
    {
        print_heap_state();
    }
    else if (command == F("kalman"))
    {
        kalman_plugin->set_enabled(!kalman_plugin->is_enabled());
    }
    else if (command == F("led"))
    {
        demo_drive_leds();
    }
    else if (command == F("l"))
    {
        float desired_velocity = 0.5;
        if (n > 1)
        {
            desired_velocity = words[1].toFloat();
        }
//        set_mode(Mode::COMMAND_MODE);
        motors[static_cast<int>(MotorLocation::LEFT)].set_desired_velocity(desired_velocity);
    }
    else if (command == F("left"))
    {
        int speed = SPEED_FULL;
        int duration = 500;
        if (n > 1)
        {
            speed = words[1].toInt();
        }
        if (n > 2)
        {
            duration = words[2].toInt();
        }
        set_mode(Mode::COMMAND_MODE);
        clockwise_plugin->set_duration(duration);
        clockwise_plugin->set_right_speed(speed);
        clockwise_plugin->set_left_speed(speed);
        clockwise_plugin->set_enabled(true);
    }
    else if (command == F("mpu"))
    {
        mpu_plugin->set_enabled(!mpu_plugin->is_enabled());
    }
    else if (command == "odom")
    {
        odom_plugin->set_enabled(!odom_plugin->is_enabled());
    }
    else if (command == "plugins")
    {
        LOG_INFO(logger, "Plugins %d", plugins.size());
        for (Plugin *const plugin : plugins)
        {
            const long cycle_count = plugin->get_cycle_count();
            const float total_micros = plugin->get_total_micros();
            if (cycle_count > 0)
            {
                logger.info() << F("Plugin ") << plugin->get_id() << F(" average ")
                        << total_micros / cycle_count << F(" us over ") << cycle_count << F(" cycles")
                        << std::endl;
            }
        }
        const float f_total_micros = total_cycle_us;
        logger.info() << F("Cycle count ") << cycle_count << F(" total cycle micros ") << total_cycle_us
                << F(" average micros per cycle ") << f_total_micros / cycle_count << std::endl;
    }
    else if (command == F("r"))
    {
        float desired_velocity = 0.5;
        if (n > 1)
        {
            desired_velocity = words[1].toFloat();
        }
//        set_mode(Mode::COMMAND_MODE);
        motors[static_cast<int>(MotorLocation::RIGHT)].set_desired_velocity(desired_velocity);
    }
    else if (command == F("right"))
    {
        int speed = SPEED_FULL;
        int duration = 500;
        if (n > 1)
        {
            speed = words[1].toInt();
        }
        if (n > 2)
        {
            duration = words[2].toInt();
        }
        set_mode(Mode::COMMAND_MODE);
        counterclockwise_plugin->set_duration(duration);
        counterclockwise_plugin->set_right_speed(speed);
        counterclockwise_plugin->set_left_speed(speed);
        counterclockwise_plugin->set_enabled(true);
    }
    else if (command == F("reset"))
    {
        all_stop();
        for (Plugin *plugin : plugins)
        {
            plugin->reset();
        }
        set_mode(Mode::COMMAND_MODE);
    }
    else if (command == F("s"))
    {
        all_stop();
        set_mode(Mode::COMMAND_MODE);
    }
    else if (command == F("schedule"))
    {
        for (int i = 0; i < 100; i++)
        {
            const PluginId id = schedule[i];
            const Plugin *plugin = get_plugin(id);
            logger.info() << F("[") << i << F("] ") << id << F(": ")
                    << ((plugin == nullptr) ? F("null") : F("found")) << std::endl;
        }
    }
    else if (command == F("wall"))
    {
        set_mode(Mode::WALL_MODE);
        logger.info() << F("Current mode is ") << mode << std::endl;
    }
    else if (command == F("zero"))
    {
        set_mode(Mode::COMMAND_MODE);
        kalman_plugin->reset();
        odom_plugin->reset();
    }
    else if (command == F("?"))
    {
        help_command();
    }
    else
    {
        logger.info() << F("Invalid command: ") << command << std::endl;
    }
}

void Car::help_command () const
{
    logger.info() << F("Robot ") << *this << std::endl;
    long d = ultrasound_plugin->get_distance();
    logger.info() << F("Distance ") << d << F(" cm") << std::endl;

    const BLA::Matrix<Nstate> &state = kalman_plugin->get_state();
    logger.info() << F("Position ") << state(0) << F(", ") << state(1) << F(" angle ") << state(2)
            << F(" Velocity ") << state(3) << F(", ") << state(4) << F(" angle ") << state(5)
            << F(" Acceleration ") << state(6) << F(", ") << state(7) << F(" angle ") << state(8)
            << std::endl;
    logger.info() << F("Angle: ") << kalman_plugin->get_angle() << std::endl;

    LOG_INFO(logger, "Plugins %d", plugins.size());
    if (cycle_count > 0)
    {
        const float f_total_micros = total_cycle_us;
        logger.info() << F("Cycle count ") << cycle_count << F(" total cycle micros ") << total_cycle_us
                << F(" average micros per cycle ") << f_total_micros / cycle_count << std::endl;
    }

    for (int motor = 0; motor < MOTOR_COUNT; motor++)
    {
        const MotorPlugin &m = motors[motor];
        logger.info() << m.get_location() << F(" speed counter ") << m.get_speed_counter()
                << std::endl;
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
        motors[motor].drive_stop();
    }
    forward_plugin->set_enabled(false);
    reverse_plugin->set_enabled(false);
    clockwise_plugin->set_enabled(false);
    counterclockwise_plugin->set_enabled(false);
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
    if (motor == MotorLocation::RIGHT || motor == MotorLocation::LEFT)
    {
        motors[static_cast<int>(motor)].drive_stop();
    }
}

void Car::drive (const MotorLocation motor, const int speed)
{
    if (speed > 0)
    {
        motors[static_cast<int>(motor)].drive_forward(speed);
    }
    else if (speed < 0)
    {
        motors[static_cast<int>(motor)].drive_reverse(-speed);
    }
    else
    {
        motors[static_cast<int>(motor)].drive_stop();
    }
}

int Car::get_drive_speed (const MotorLocation motor) const
{
    const int speed = motors[static_cast<int>(motor)].get_speed();
    if (motors[static_cast<int>(motor)].get_direction() == MotorDirection::REVERSE)
    {
        return -speed;
    }
    return speed;
}

float Car::get_measured_velocity (const MotorLocation motor) const
{
    return motors[static_cast<int>(motor)].get_measured_velocity();
}

float Car::get_desired_velocity (const MotorLocation motor) const
{
    return motors[static_cast<int>(motor)].get_desired_velocity();
}

void Car::set_desired_velocity (const MotorLocation motor, const float velocity)
{
    motors[static_cast<int>(motor)].set_desired_velocity(velocity);
}

MotorDirection Car::get_drive_direction (const MotorLocation motor) const
{
    return motors[static_cast<int>(motor)].get_direction();
}

void Car::drive_forward (const MotorLocation motor, const int speed)
{
    if (motor == MotorLocation::RIGHT || motor == MotorLocation::LEFT)
    {
        motors[static_cast<int>(motor)].drive_forward(speed);
    }
}

void Car::drive_reverse (const MotorLocation motor, const int speed)
{
    if (motor == MotorLocation::RIGHT || motor == MotorLocation::LEFT)
    {
        motors[static_cast<int>(motor)].drive_reverse(speed);
    }
}

ClockPlugin* Car::get_clock_plugin () const
{
    return clock_plugin;
}

DemoPlugin* Car::get_demo_plugin () const
{
    return demo_plugin;
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

DrivePlugin* Car::get_clockwise_plugin () const
{
    return clockwise_plugin;
}

DrivePlugin* Car::get_counterclockwise_plugin () const
{
    return counterclockwise_plugin;
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
        case PluginId::CLOCKWISE_PLUGIN:
            return clockwise_plugin;
        case PluginId::COUNTERCLOCKWISE_PLUGIN:
            return counterclockwise_plugin;
        case PluginId::DEMO_PLUGIN:
            return demo_plugin;
        case PluginId::FORWARD_PLUGIN:
            return forward_plugin;
        case PluginId::GOAL_PLUGIN:
            return goal_plugin;
        case PluginId::KALMAN_PLUGIN:
            return kalman_plugin;
        case PluginId::MOTOR_RIGHT_PLUGIN:
            return &motors[static_cast<int>(MotorLocation::RIGHT)];
        case PluginId::MOTOR_LEFT_PLUGIN:
            return &motors[static_cast<int>(MotorLocation::LEFT)];
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
