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
}

std::ostream& operator<< (std::ostream &lhs, const Car &car)
{
    return lhs << F("#[car ") << car.mode << F(" ") << kalman_plugin.get_x() << F(", ") << kalman_plugin.get_y()
            << F(" ang ") << kalman_plugin.get_angle() << F("]");
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

