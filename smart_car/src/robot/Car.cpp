/*
 * Car.cpp
 *
 *  Created on: Dec 23, 2022
 *      Author: cre
 */

#include "Car.hpp"
#include "smart_car.hpp"

// For some reason this does not resolve.
extern HardwareSerial Serial;

Car::Car () : serial_parser(Serial), bluetooth_parser(Serial1)
{
    clock_plugin = new ClockPlugin();
    clockwise_plugin = new DrivePlugin(CLOCKWISE_PLUGIN, *this, 500, FORWARD, REVERSE);
    counterclockwise_plugin = new DrivePlugin(COUNTERCLOCKWISE_PLUGIN, *this, 500, REVERSE,
            FORWARD);
    demo_plugin = new DemoPlugin(*this);
    forward_plugin = new DrivePlugin(FORWARD_PLUGIN, *this, 500, FORWARD, FORWARD);
    mpu_plugin = new MpuPlugin();
    kalman_plugin = new KalmanPlugin(*this);
    odom_plugin = new OdomPlugin(*this);
    reverse_plugin = new DrivePlugin(REVERSE_PLUGIN, *this, 500, REVERSE, REVERSE);
    ultrasound_plugin = new UltrasoundPlugin();
    wall_plugin = new WallPlugin(*this);

    available_plugins.push_back(clock_plugin);
    available_plugins.push_back(demo_plugin);
    available_plugins.push_back(forward_plugin);
    available_plugins.push_back(reverse_plugin);
    available_plugins.push_back(clockwise_plugin);
    available_plugins.push_back(counterclockwise_plugin);
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
    return lhs << "#[car " << car.kalman_plugin->get_x() << ", " << car.kalman_plugin->get_y() << "]";
}

void Car::setup ()
{
    for (int motor = 0; motor < MOTOR_COUNT; motor++)
    {
        motors[motor].setup();
    }

    cout << "Attempting setup of " << available_plugins.size() << " available plugins" << std::endl;
    for (Plugin *plugin : available_plugins)
    {
        if (plugin->setup())
        {
            cout << "Setup " << plugin->get_id() << std::endl;
            plugins.push_back(plugin);
        }
        else
        {
            cout << "Disabled " << plugin->get_id() << std::endl;
        }
    }
    cout << "Completed Setup of " << plugins.size() << " enabled plugins" << std::endl;
}

void Car::set_mode (const Mode _mode)
{
    cout << "Setting " << *this << " to " << _mode << std::endl;
    mode = _mode;
    switch (mode)
    {
        case COMMAND_MODE:
            all_stop ();
            demo_plugin->set_enabled(false);
            wall_plugin->set_enabled(false);
            break;
        case DEMO_MODE:
            all_stop ();
            demo_plugin->set_enabled(true);
            wall_plugin->set_enabled(false);
            break;
        case GOAL_MODE:
            all_stop ();
            demo_plugin->set_enabled(false);
            wall_plugin->set_enabled(false);
            break;
        case WALL_MODE:
            all_stop ();
            demo_plugin->set_enabled(false);
            wall_plugin->set_enabled(true);
            break;
    }
}

void Car::cycle ()
{
    const long cycle_start_us = micros();

    handle_command();
    for (Plugin *plugin : plugins)
    {
        plugin->cycle();
    }

    total_cycle_us += micros() - cycle_start_us;
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
    Serial.print("Command: ");
    Serial.println(command);
    if (command == "b")
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
//        imu_plugin->calibrate();
        mpu_plugin->calibrate();
    }
    else if (command == "c")
    {
        set_mode(COMMAND_MODE);
        cout << "Current mode is " << mode << std::endl;
    }
    else if (command == "clock")
    {
        clock_plugin->set_enabled(!clock_plugin->is_enabled());
    }
    else if (command == "demo")
    {
        set_mode(DEMO_MODE);
        cout << "Current mode is " << mode << std::endl;
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
        for (Plugin *plugin : plugins)
        {
            cout << "Plugin " << plugin->get_id() << std::endl;
        }
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
    else if (command == "s")
    {
        all_stop();
        set_mode(COMMAND_MODE);
    }
    else if (command == "wall")
    {
        set_mode(WALL_MODE);
        cout << "Current mode is " << mode << std::endl;
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
        Serial.print("Invalid command: ");
        Serial.println(command);
    }
}

void Car::help_command ()
{
    Serial.println("b - backward");
    Serial.println("c - command mode");
    Serial.println("d - demo mode");
    Serial.println("f - forward");
    Serial.println("l - left turn");
    Serial.println("r - right turn");
    Serial.println("s - stop moving");
    Serial.println("? - help");

    cout << "Current mode is " << mode << std::endl;
//    imu_plugin->read_imu();
    ultrasound_plugin->print_distance();

    Serial.print("Plugins: ");
    Serial.println(plugins.size());
    Serial.print("Cycle Count: ");
    Serial.print(cycle_count);
    Serial.print(" total cycle micros ");
    Serial.print(total_cycle_us);
    Serial.print(" average micros per cycle ");
    Serial.println(total_cycle_us / (double) cycle_count);
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

Motor& Car::get_motor (const MotorLocation motor)
{
    return motors[motor];
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
}

void Car::drive_stop (const MotorLocation motor)
{
    motors[motor].drive_stop();
}

void Car::drive_forward (const MotorLocation motor, const int speed)
{
    motors[motor].drive_forward(speed);
}

void Car::drive_reverse (const MotorLocation motor, const int speed)
{
    motors[motor].drive_reverse(speed);
}

WallPlugin* Car::get_wall_plugin ()
{
    return wall_plugin;
}

DemoPlugin* Car::get_demo_plugin ()
{
    return demo_plugin;
}

DrivePlugin* Car::get_forward_plugin ()
{
    return forward_plugin;
}

DrivePlugin* Car::get_reverse_plugin ()
{
    return reverse_plugin;
}

DrivePlugin* Car::get_clockwise_plugin ()
{
    return clockwise_plugin;
}

DrivePlugin* Car::get_counterclockwise_plugin ()
{
    return counterclockwise_plugin;
}

//ImuPlugin* Car::get_imu_plugin ()
//{
//    return imu_plugin;
//}

MpuPlugin* Car::get_mpu_plugin ()
{
    return mpu_plugin;
}

KalmanPlugin* Car::get_navigation_plugin ()
{
    return kalman_plugin;
}

OdomPlugin* Car::get_odom_plugin ()
{
    return odom_plugin;
}

UltrasoundPlugin* Car::get_ultrasound_plugin ()
{
    return ultrasound_plugin;
}
