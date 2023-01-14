/*
 * car_handler.cpp
 *
 *  Created on: Jan 13, 2023
 *      Author: cre
 */

#include "Car.hpp"
#include "Heap.hpp"
#include "../logging/Logger.hpp"
#include "../logging/RobotAppender.hpp"
#include "../plugins/DrivePlugin.hpp"
#include "../plugins/GoalPlugin.hpp"
#include "../plugins/KalmanPlugin.hpp"
#include "../plugins/MpuPlugin.hpp"
#include "../plugins/OdomPlugin.hpp"
#include "../plugins/UltrasoundPlugin.hpp"

/** Execute a command from the user.
 @param words Command string broken into words.
 */
void Car::execute_command (const std::vector<String> &words)
{
    const int n = words.size();
    const String command = words[0];
    logger.info(__LINE__) << F("Command: ") << command.c_str() << std::endl;
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
        logger.info(__LINE__) << F("Current mode is ") << mode << std::endl;
    }
    else if (command == F("demo"))
    {
        set_mode(Mode::DEMO_MODE);
        logger.info(__LINE__) << F("Current mode is ") << mode << std::endl;
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
//    else if (command == F("log"))
//    {
//        if (robot_appender != nullptr && n > 1)
//        {
//            if (n > 2)
//            {
//                robot_appender->set_logger_state(words[1], words[2]);
//            }
//            logger.info(__LINE__) << words[1].c_str() << F(": ")
//                    << robot_appender->get_logger_state(words[1]) << std::endl;
//        }
//    }
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
                logger.info(__LINE__) << F("Plugin ") << plugin->get_id() << F(" average ")
                        << total_micros / cycle_count << F(" / ") << plugin->get_expected_us()
                        << F(" us expected over ") << cycle_count << F(" cycles") << std::endl;
            }
        }
        const float f_total_micros = total_cycle_us;
        logger.info(__LINE__) << F("Cycle count ") << cycle_count << F(" total cycle micros ")
                << total_cycle_us << F(" average micros per cycle ") << f_total_micros / cycle_count
                << std::endl;
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
        for (int i = 0; i < schedule_size; i++)
        {
            const PluginId id = schedule[i];
            const Plugin *plugin = get_plugin(id);
            logger.info(__LINE__) << F("[") << i << F("] ") << id << F(": ")
                    << ((plugin == nullptr) ? F("null") : F("found")) << std::endl;
        }
    }
    else if (command == F("wall"))
    {
        set_mode(Mode::WALL_MODE);
        logger.info(__LINE__) << F("Current mode is ") << mode << std::endl;
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
        logger.info(__LINE__) << F("Invalid command: ") << command.c_str() << std::endl;
    }
}

void Car::help_command () const
{
    logger.info(__LINE__) << F("Robot ") << *this << std::endl;
    long d = ultrasound_plugin->get_distance();
    logger.info(__LINE__) << F("Distance ") << d << F(" cm") << std::endl;

    const BLA::Matrix<Nstate> &state = kalman_plugin->get_state();
    logger.info(__LINE__) << F("Position ") << state(0) << F(", ") << state(1) << F(" angle ")
            << state(2) << F(" Velocity ") << state(3) << F(", ") << state(4) << F(" angle ")
            << state(5) << F(" Acceleration ") << state(6) << F(", ") << state(7) << F(" angle ")
            << state(8) << std::endl;
    logger.info(__LINE__) << F("Angle: ") << kalman_plugin->get_angle() << std::endl;

    LOG_INFO(logger, "Plugins %d", plugins.size());
    if (cycle_count > 0)
    {
        const float f_total_micros = total_cycle_us;
        logger.info(__LINE__) << F("Cycle count ") << cycle_count << F(" total cycle micros ")
                << total_cycle_us << F(" average micros per cycle ") << f_total_micros / cycle_count
                << std::endl;
    }

    for (int motor = 0; motor < MOTOR_COUNT; motor++)
    {
        const MotorPlugin &m = motors[motor];
        logger.info(__LINE__) << m.get_location() << F(" speed counter ") << m.get_speed_counter()
                << std::endl;
    }
}
