/*
 * GoalPlugin.cpp
 *
 *  Created on: Jan 1, 2023
 *      Author: cre
 */

#include "../robot/Car.hpp"
#include "../robot/robot_math.hpp"
#include "../robot/speed_counter.hpp"
#include "GoalPlugin.hpp"
#include "KalmanPlugin.hpp"
#include "MpuPlugin.hpp"
#include "smart_car.hpp"
#include "../logging/Logger.hpp"

static logging::Logger logger(__FILE__, logging::Level::info);

GoalPlugin::GoalPlugin (Car &car) :
        Plugin(PluginId::GOAL_PLUGIN), car(car)
{
}

bool GoalPlugin::setup ()
{
    return true;
}

void GoalPlugin::set_goal (const float angle)
{
    goal_angle = angle;
    set_state(ADJUST_ANGLE);
    logger.info(__LINE__) << "Set " << car << " goal angle " << goal_angle << std::endl;
}

void GoalPlugin::set_goal (const float x, const float y)
{
    goal_x = x;
    goal_y = y;
    set_state(ADJUST_POSITION);
    logger.info(__LINE__) << "Set " << car << " goal position " << x << ", " << y << std::endl;
}

int GoalPlugin::get_preferred_interval () const
{
    return 10;
}

int GoalPlugin::get_expected_us () const
{
    return 50;
}

void GoalPlugin::cycle ()
{
    const int state = get_state();
    if (state != DISABLE)
    {
//        const float measured_angle = car.get_kalman_plugin()->get_angle();
        const float measured_angle = car.get_mpu_plugin()->get_yaw();

        if (!is_overflow(measured_angle))
        {
            if (state == ADJUST_ANGLE)
            {
                angle_cycle(normalize_angle(measured_angle), goal_angle);
            }
            else if (state == ADJUST_POSITION)
            {
                const float measured_x = car.get_kalman_plugin()->get_x();
                const float measured_y = car.get_kalman_plugin()->get_y();
                position_cycle(normalize_angle(measured_angle), measured_x, measured_y, goal_x, goal_y);
            }
        }
        else
        {
            logger.info(__LINE__) << "Invalid imu reading " << measured_angle << std::endl;
        }
    }
}

void GoalPlugin::angle_cycle (const float measured_angle, const float desired_angle)
{
    const float delta = angle_delta(desired_angle, measured_angle);
    if (abs(delta) < angle_tolerance)
    {
        logger.info(__LINE__) << "Car " << car << " at " << measured_angle << " is goal angle " << desired_angle
                << std::endl;
        car.drive_stop(MotorLocation::RIGHT_FRONT);
        car.drive_stop(MotorLocation::LEFT_FRONT);
        car.drive_stop(MotorLocation::RIGHT_REAR);
        car.drive_stop(MotorLocation::LEFT_REAR);
        set_state(DISABLE);
    }
    else
    {
        if (delta > 0)
        {
            logger.info(__LINE__) << measured_angle << "=yaw Clockwise from " << measured_angle << " by " << delta
                    << " to goal angle " << desired_angle << std::endl;
            car.set_desired_velocity(MotorLocation::RIGHT_FRONT, -angle_desired_velocity);
            car.set_desired_velocity(MotorLocation::RIGHT_REAR, -angle_desired_velocity);
            car.set_desired_velocity(MotorLocation::LEFT_FRONT, angle_desired_velocity);
            car.set_desired_velocity(MotorLocation::LEFT_REAR, angle_desired_velocity);
        }
        else if (delta < 0)
        {
            logger.info(__LINE__) << measured_angle << "=yaw Counterclockwise from " << measured_angle << " by "
                    << delta << " to goal angle " << desired_angle << std::endl;
            car.set_desired_velocity(MotorLocation::RIGHT_FRONT, angle_desired_velocity);
            car.set_desired_velocity(MotorLocation::RIGHT_REAR, angle_desired_velocity);
            car.set_desired_velocity(MotorLocation::LEFT_FRONT, -angle_desired_velocity);
            car.set_desired_velocity(MotorLocation::LEFT_REAR, -angle_desired_velocity);
        }
        const float yaw_perimeter_meters = abs(measured_angle) * rotation_perimeter / 2.0 * M_PI; // meters
        const unsigned long delta_ticks = yaw_perimeter_meters / (2.0 * count_to_meters);
        logger.info(__LINE__) << "perimeter " << yaw_perimeter_meters << " speed ticks " << delta_ticks << std::endl;
        car.get_motor(RIGHT_FRONT).set_delta_limit(delta_ticks, 0);
        car.get_motor(RIGHT_REAR).set_delta_limit(delta_ticks, 0);
        car.get_motor(LEFT_FRONT).set_delta_limit(delta_ticks, 0);
        car.get_motor(LEFT_REAR).set_delta_limit(delta_ticks, 0);
    }
}

void GoalPlugin::position_cycle (const float measured_angle, const float measured_x, const float measured_y,
        const float desired_x, const float desired_y)
{
    const float dx = desired_x - measured_x;
    const float dy = desired_y - measured_y;
    if (abs(dx) < position_tolerance && abs(dy) < position_tolerance)
    {
        logger.info(__LINE__) << "Robot " << car << " is at goal " << desired_x << ", " << desired_y << std::endl;
        car.drive_stop(MotorLocation::RIGHT_FRONT);
        car.drive_stop(MotorLocation::LEFT_FRONT);
        car.drive_stop(MotorLocation::RIGHT_REAR);
        car.drive_stop(MotorLocation::LEFT_REAR);
        set_state(DISABLE);
    }
    else
    {
        position_step(measured_angle, measured_x, measured_y, desired_x, desired_y);
    }
}

void GoalPlugin::position_step (const float measured_angle, const float measured_x, const float measured_y,
        const float desired_x, const float desired_y)
{
    const float dx = desired_x - measured_x;
    const float dy = desired_y - measured_y;
    const float distance = sqrt(dx * dx + dy * dy);
    const float desired_angle = atan2(dy, dx);
    const float delta_angle = angle_delta(desired_angle, measured_angle);
    const float abs_delta_angle = abs(delta_angle);

    logger.info(__LINE__) << "Required " << car << " dx " << dx << " dy " << dy << " angle " << desired_angle << " for "
            << distance << " to " << desired_x << ", " << desired_y << std::endl;

// @see https://en.wikipedia.org/wiki/PID_controller

    if (abs_delta_angle >= 1.0 || abs_delta_angle > distance)
    {
        angle_cycle(measured_angle, desired_angle);
    }
    else if (abs_delta_angle < angle_tolerance)
    {
        const int speed = get_position_speed(distance);
        car.set_right_speed(speed);
        car.set_left_speed(speed);
    }
    else if (delta_angle > 0)
    {
        const int speed_high = std::min(get_position_speed(distance), get_angle_speed(abs_delta_angle));
        const int speed_low = speed_high * (1.0 - abs_delta_angle);
        logger.info(__LINE__) << "Drive Clockwise from " << measured_angle << " by " << delta_angle << " to goal angle "
                << desired_angle << " at R" << speed_low << " L" << speed_high << std::endl;
        car.set_left_speed(speed_high);
        car.set_right_speed(speed_low);
    }
    else
    {
        const int speed_high = std::min(get_position_speed(distance), get_angle_speed(abs_delta_angle));
        const int speed_low = speed_high * (1.0 - abs_delta_angle);
        logger.info(__LINE__) << "Drive Counterclockwise from " << measured_angle << " by " << delta_angle
                << " to goal angle " << desired_angle << " at R" << speed_high << " L" << speed_low << std::endl;
        car.set_right_speed(speed_high);
        car.set_left_speed(speed_low);
    }
}

int GoalPlugin::get_angle_speed (const float angle)
{
    if (angle < slow_speed_angle)
        return SPEED_100;
    else if (angle < medium_speed_angle)
        return SPEED_150;
    return SPEED_FULL;
}

int GoalPlugin::get_position_speed (const float distance)
{
    if (distance < slow_speed_distance)
    {
        return SPEED_100;
    }
    else if (distance < medium_speed_distance)
    {
        return SPEED_150;
    }
    return SPEED_FULL;
}
