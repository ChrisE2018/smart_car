/*
 * Car.cpp
 *
 *  Created on: Dec 23, 2022
 *      Author: cre
 */

#include "Car.hpp"

#include "smart_car.hpp"
#include "../logging/Logger.hpp"

// For some reason this does not always resolve.
extern HardwareSerial Serial;

static Logger logger(__FILE__, Level::debug);

Car::Car () : serial_parser(Serial), bluetooth_parser(Serial3)
{
    clock_plugin = new ClockPlugin();
    clockwise_plugin = new DrivePlugin(CLOCKWISE_PLUGIN, *this, 500, FORWARD, REVERSE);
    counterclockwise_plugin = new DrivePlugin(COUNTERCLOCKWISE_PLUGIN, *this, 500, REVERSE,
            FORWARD);
    demo_plugin = new DemoPlugin(*this);
    forward_plugin = new DrivePlugin(FORWARD_PLUGIN, *this, 500, FORWARD, FORWARD);
    goal_plugin = new GoalPlugin(*this);
    mpu_plugin = new MpuPlugin();
    kalman_plugin = new KalmanPlugin(*this);
    odom_plugin = new OdomPlugin(*this);
    reverse_plugin = new DrivePlugin(REVERSE_PLUGIN, *this, 500, REVERSE, REVERSE);
    ultrasound_plugin = new UltrasoundPlugin(*this);
    wall_plugin = new WallPlugin(*this);

    available_plugins.push_back(clock_plugin);
    available_plugins.push_back(demo_plugin);
    available_plugins.push_back(forward_plugin);
    available_plugins.push_back(goal_plugin);
    available_plugins.push_back(reverse_plugin);
    available_plugins.push_back(clockwise_plugin);
    available_plugins.push_back(counterclockwise_plugin);
    available_plugins.push_back(&motors[0]);
    available_plugins.push_back(&motors[1]);
    available_plugins.push_back(mpu_plugin);
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
    for (int motor = 0; motor < MOTOR_COUNT; motor++)
    {
        motors[motor].setup();
    }
    LOG_INFO(logger, "Attempting setup of %d available plugins", available_plugins.size());
    for (Plugin *plugin : available_plugins)
    {
        if (plugin->setup())
        {
            logger.info() << "Setup " << plugin->get_id() << std::endl;
            plugins.push_back(plugin);
        }
        else
        {
            logger.info() << "Disabled " << plugin->get_id() << std::endl;
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
                        schedule[cycle++] = IDLE_CYCLE;
                    }
                }
            }
        }
        if (cycle < 99)
        {
            schedule[cycle++] = COMMAND_CYCLE;
            schedule[cycle++] = IDLE_CYCLE;
        }
    }
    LOG_INFO(logger, "Completed setup of %d enabled plugins", plugins.size());

    logger.info() << "Testing info logging" << std::endl;
    logger.debug() << "Testing debug logging" << std::endl;
}

int Car::get_actual_interval (int cycle, PluginId id) const
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
    logger.info() << "Set " << *this << " to " << _mode << std::endl;
    switch (mode)
    {
        case COMMAND_MODE:
            all_stop();
            demo_plugin->set_enabled(false);
            wall_plugin->set_enabled(false);
            break;
        case DEMO_MODE:
            all_stop();
            demo_plugin->set_enabled(true);
            wall_plugin->set_enabled(false);
            break;
        case GOAL_MODE:
            all_stop();
            demo_plugin->set_enabled(false);
            wall_plugin->set_enabled(false);
            break;
        case WALL_MODE:
            all_stop();
            demo_plugin->set_enabled(false);
            wall_plugin->set_enabled(true);
            break;
    }
}

void Car::cycle ()
{
    const unsigned long cycle_start_us = micros();
    const int ms = millis() % 1000;
    const int cycle = ms / ms_per_cycle;
    if (cycle != last_cycle)
    {
        last_cycle = cycle;
        const PluginId scheduled_plugin = schedule[cycle];
        Plugin *plugin = get_plugin(scheduled_plugin);
        if (plugin == nullptr)
        {
            handle_command();
        }
        else
        {
            plugin->start_cycle();
            plugin->cycle();
            plugin->end_cycle();
        }
    }

    const unsigned long duration = micros() - cycle_start_us;
    total_cycle_us += duration;
    cycle_count++;
}

void Car::handle_command ()
{
    serial_parser.handle_command(*this);
    bluetooth_parser.handle_command(*this);
}

/** Execute a command from the user.
 @param words Command string broken into words.
 */
void Car::execute_command (const std::vector<String> words)
{
    const int n = words.size();
    String command = words[0];
    logger.info() << "Command: " << command << std::endl;
    if (command == "angle")
    {
        if (n > 1)
        {
            set_mode(GOAL_MODE);
            goal_plugin->set_goal(words[1].toFloat());
        }
    }
    else if (command == "b")
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
        set_mode(COMMAND_MODE);
        reverse_plugin->set_duration(duration);
        reverse_plugin->set_right_speed(speed);
        reverse_plugin->set_left_speed(speed);
        reverse_plugin->set_enabled(true);
    }
    else if (command == "calibrate")
    {
        mpu_plugin->calibrate();
    }
    else if (command == "c")
    {
        set_mode(COMMAND_MODE);
        logger.info() << "Current mode is " << mode << std::endl;
    }
    else if (command == "clock")
    {
        clock_plugin->set_enabled(!clock_plugin->is_enabled());
    }
    else if (command == "demo")
    {
        set_mode(DEMO_MODE);
        logger.info() << "Current mode is " << mode << std::endl;
    }
    else if (command == "distance")
    {
        ultrasound_plugin->set_enabled(!ultrasound_plugin->is_enabled());
    }
    else if (command == "f")
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
        set_mode(COMMAND_MODE);
        forward_plugin->set_duration(duration);
        forward_plugin->set_right_speed(speed);
        forward_plugin->set_left_speed(speed);
        forward_plugin->set_enabled(true);
    }
    else if (command == "goal")
    {
        if (n > 2)
        {
            set_mode(GOAL_MODE);
            goal_plugin->set_goal(words[1].toFloat(), words[2].toFloat());
        }
    }
    else if (command == "kalman")
    {
        kalman_plugin->set_enabled(!kalman_plugin->is_enabled());
    }
    else if (command == "led")
    {
        demo_drive_leds();
    }
    else if (command == "l")
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
        set_mode(COMMAND_MODE);
        clockwise_plugin->set_duration(duration);
        clockwise_plugin->set_right_speed(speed);
        clockwise_plugin->set_left_speed(speed);
        clockwise_plugin->set_enabled(true);
    }
    else if (command == "mpu")
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
        for (Plugin *plugin : plugins)
        {
            const long cycle_count = plugin->get_cycle_count();
            const long total_micros = plugin->get_total_micros();
            if (cycle_count > 0)
            {
                logger.info() << "Plugin " << plugin->get_id() << " average "
                        << total_micros / cycle_count << " us over " << cycle_count << " cycles"
                        << std::endl;
            }
        }
        const float f_total_micros = total_cycle_us;
        logger.info() << "Cycle count " << cycle_count << " total cycle micros " << total_cycle_us
                << " average micros per cycle " << f_total_micros / cycle_count << std::endl;
    }
    else if (command == "r")
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
        set_mode(COMMAND_MODE);
        counterclockwise_plugin->set_duration(duration);
        counterclockwise_plugin->set_right_speed(speed);
        counterclockwise_plugin->set_left_speed(speed);
        counterclockwise_plugin->set_enabled(true);
    }
    else if (command == "reset")
    {
        all_stop();
        for (Plugin *plugin : plugins)
        {
            plugin->reset();
        }
        set_mode(COMMAND_MODE);
    }
    else if (command == "s")
    {
        all_stop();
        set_mode(COMMAND_MODE);
    }
    else if (command == "schedule")
    {
        for (int i = 0; i < 100; i++)
        {
            const PluginId id = schedule[i];
            const Plugin *plugin = get_plugin(id);
            logger.info() << "[" << i << "] " << id << ": "
                    << ((plugin == nullptr) ? "null" : "found") << std::endl;
        }
    }
    else if (command == "wall")
    {
        set_mode(WALL_MODE);
        logger.info() << "Current mode is " << mode << std::endl;
    }
    else if (command == "zero")
    {
        set_mode(COMMAND_MODE);
        kalman_plugin->reset();
        odom_plugin->reset();
    }
    else if (command == "?")
    {
        help_command();
    }
    else
    {
        logger.info() << "Invalid command: " << command << std::endl;
    }
}

void Car::help_command () const
{
    logger.info() << "Robot " << *this << std::endl;
    long d = ultrasound_plugin->get_distance();
    logger.info() << "Distance " << d << " cm" << std::endl;

    const BLA::Matrix<Nstate> &state = kalman_plugin->get_state();
    logger.info() << "Position " << state(0) << ", " << state(1) << " angle " << state(2)
            << " Velocity " << state(3) << ", " << state(4) << " angle " << state(5)
            << " Acceleration " << state(6) << ", " << state(7) << " angle " << state(8)
            << std::endl;
    logger.info() << "Angle: " << kalman_plugin->get_angle() << std::endl;

    LOG_INFO(logger, "Plugins %d", plugins.size());
    const float f_total_micros = total_cycle_us;
    logger.info() << "Cycle count " << cycle_count << " total cycle micros " << total_cycle_us
            << " average micros per cycle " << f_total_micros / cycle_count << std::endl;

    for (int motor = 0; motor < MOTOR_COUNT; motor++)
    {
        const MotorPlugin &m = motors[motor];
        logger.info() << m.get_location() << " speed counter " << m.get_speed_counter()
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
    if (motor == RIGHT || motor == LEFT)
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
    if (motors[static_cast<int>(motor)].get_direction() == REVERSE)
    {
        return -speed;
    }
    return speed;
}

float Car::get_drive_velocity (const MotorLocation motor) const
{
    return motors[static_cast<int>(motor)].get_velocity();
}

MotorDirection Car::get_drive_direction (const MotorLocation motor) const
{
    return motors[static_cast<int>(motor)].get_direction();
}

void Car::drive_forward (const MotorLocation motor, const int speed)
{
    if (motor == RIGHT || motor == LEFT)
    {
        motors[static_cast<int>(motor)].drive_forward(speed);
    }
}

void Car::drive_reverse (const MotorLocation motor, const int speed)
{
    if (motor == RIGHT || motor == LEFT)
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

Plugin* Car::get_plugin (PluginId id) const
{
    switch (id)
    {
        case CLOCK_PLUGIN:
            return clock_plugin;
        case CLOCKWISE_PLUGIN:
            return clockwise_plugin;
        case COUNTERCLOCKWISE_PLUGIN:
            return counterclockwise_plugin;
        case DEMO_PLUGIN:
            return demo_plugin;
        case FORWARD_PLUGIN:
            return forward_plugin;
        case GOAL_PLUGIN:
            return goal_plugin;
        case MPU_PLUGIN:
            return mpu_plugin;
        case KALMAN_PLUGIN:
            return kalman_plugin;
        case ODOM_PLUGIN:
            return odom_plugin;
        case REVERSE_PLUGIN:
            return reverse_plugin;
        case ULTRASOUND_PLUGIN:
            return ultrasound_plugin;
        case WALL_PLUGIN:
            return wall_plugin;
        default:
            return nullptr;
    }
}
