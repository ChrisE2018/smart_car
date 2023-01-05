/*
 * GoalPlugin.cpp
 *
 *  Created on: Jan 1, 2023
 *      Author: cre
 */

#include "../robot/Car.hpp"
#include "../robot/robot_math.hpp"
#include "GoalPlugin.hpp"
#include "KalmanPlugin.hpp"
#include "smart_car.hpp"

GoalPlugin::GoalPlugin (Car &car) : Plugin(GOAL_PLUGIN), car(car)
{
}

bool GoalPlugin::setup ()
{
    adjust_angle = false;
    adjust_position = false;
    return true;
}

void GoalPlugin::set_goal (const float angle)
{
    goal_angle = angle;
    adjust_angle = true;
    adjust_position = false;
    set_enabled(true);
    cout << "Set " << car << " goal angle " << goal_angle << std::endl;
    cout1 << "Set " << car << " goal angle " << goal_angle << std::endl;
}

void GoalPlugin::set_goal (const float x, const float y)
{
    goal_x = x;
    goal_y = y;
    adjust_position = true;
    adjust_angle = false;
    set_enabled(true);
    cout << "Set " << car << " goal position " << x << ", " << y << std::endl;
    cout1 << "Set " << car << " goal position " << x << ", " << y << std::endl;
}

void GoalPlugin::cycle ()
{
    if (is_enabled())
    {
        const float measured_angle = car.get_kalman_plugin()->get_angle();

        if (!is_overflow(measured_angle))
        {
            if (adjust_angle)
            {
                angle_cycle(measured_angle, goal_angle);
            }
            if (adjust_position)
            {
                const float measured_x = car.get_kalman_plugin()->get_x();
                const float measured_y = car.get_kalman_plugin()->get_y();
                position_cycle(measured_angle, measured_x, measured_y, goal_x, goal_y);
            }
        }
        else
        {
            cout << "Invalid imu reading " << measured_angle << std::endl;
        }
    }
}

void GoalPlugin::angle_cycle (const float measured_angle, const float desired_angle)
{
    const float delta = angle_delta(desired_angle, measured_angle);
    if (abs(delta) > angle_tolerance)
    {
        angle_step(measured_angle, desired_angle, delta);
    }
    else
    {
        cout << "Car " << car << " at " << measured_angle << " is goal angle " << desired_angle
                << std::endl;
        cout1 << "Car " << car << " at " << measured_angle << " is goal angle " << desired_angle
                << std::endl;
        car.drive_stop(RIGHT);
        car.drive_stop(LEFT);
        adjust_angle = false;
    }

}

void GoalPlugin::angle_step (const float measured_angle, const float desired_angle,
        const float delta)
{
    const int speed = get_angle_speed(abs(delta));
    if (delta > 0)
    {
        cout << "Clockwise by " << delta << " from " << measured_angle << " to goal angle "
                << desired_angle << " at " << speed << std::endl;
        cout1 << "Clockwise by " << delta << " from " << measured_angle << " to goal angle "
                << desired_angle << " at " << speed << std::endl;
        car.drive_reverse(RIGHT, speed);
        car.drive_forward(LEFT, speed);
    }
    else
    {
        cout << "Counterclockwise by " << delta << " from " << measured_angle << " to goal angle "
                << desired_angle << " at " << speed << std::endl;
        cout1 << "Counterclockwise by " << delta << " from " << measured_angle << " to goal angle "
                << desired_angle << " at " << speed << std::endl;
        car.drive_forward(RIGHT, speed);
        car.drive_reverse(LEFT, speed);
    }
}

void GoalPlugin::position_cycle (const float measured_angle, const float measured_x,
        const float measured_y, const float desired_x, const float desired_y)
{
    const float dx = desired_x - measured_x;
    const float dy = desired_y - measured_y;
    if (abs(dx) < position_tolerance && abs(dy) < position_tolerance)
    {
        cout << "Robot " << car << " is at goal " << desired_x << ", " << desired_y << std::endl;
        cout1 << "Robot " << car << " is at goal " << desired_x << ", " << desired_y << std::endl;
        car.drive_stop(RIGHT);
        car.drive_stop(LEFT);
        adjust_position = false;
    }
    else
    {
        position_step(measured_angle, measured_x, measured_y, desired_x, desired_y);
    }
}

void GoalPlugin::position_step (const float measured_angle, const float measured_x,
        const float measured_y, const float desired_x, const float desired_y)
{
    const float dx = desired_x - measured_x;
    const float dy = desired_y - measured_y;
    const float distance = sqrt(dx * dx + dy * dy);
    const float desired_angle = atan2(dy, dx);
    const float delta_angle = angle_delta(desired_angle, measured_angle);
    const float abs_delta_angle = abs(delta_angle);

    cout << "Required " << car << " dx " << dx << " dy " << dy << " angle " << desired_angle
            << " for " << distance << " to " << desired_x << ", " << desired_y << std::endl;
    cout1 << "Required " << car << " dx " << dx << " dy " << dy << " angle " << desired_angle
            << " for " << distance << " to " << desired_x << ", " << desired_y << std::endl;

// @see https://en.wikipedia.org/wiki/PID_controller

    if (abs_delta_angle >= 1.0 || abs_delta_angle > distance)
    {
        angle_step(measured_angle, desired_angle, delta_angle);
    }
    else if (abs_delta_angle < angle_tolerance)
    {
        const int speed = get_position_speed(distance);
        car.drive_forward(RIGHT, speed);
        car.drive_forward(LEFT, speed);
    }
    else if (delta_angle > 0)
    {
        const int speed_high = std::min(get_position_speed(distance),
                get_angle_speed(abs_delta_angle));
        const int speed_low = speed_high * (1.0 - abs_delta_angle);
        cout << "Drive Clockwise by " << delta_angle << " from " << measured_angle
                << " to goal angle " << desired_angle << " at R" << speed_low << " L" << speed_high
                << std::endl;
        cout1 << "Drive Clockwise by " << delta_angle << " from " << measured_angle
                << " to goal angle " << desired_angle << " at R" << speed_low << " L" << speed_high
                << std::endl;
        car.drive_forward(LEFT, speed_high);
        car.drive_forward(RIGHT, speed_low);
    }
    else
    {
        const int speed_high = std::min(get_position_speed(distance),
                get_angle_speed(abs_delta_angle));
        const int speed_low = speed_high * (1.0 - abs_delta_angle);
        cout << "Drive Counterclockwise from " << measured_angle << " to goal angle "
                << desired_angle << " at R" << speed_high << " L" << speed_low << std::endl;
        cout1 << "Drive Counterclockwise from " << measured_angle << " to goal angle "
                << desired_angle << " at R" << speed_high << " L" << speed_low << std::endl;
        car.drive_forward(RIGHT, speed_high);
        car.drive_forward(LEFT, speed_low);
    }
}

int GoalPlugin::get_angle_speed (const float angle)
{
    if (angle < slow_speed_angle)
        return SPEED_150;
    else if (angle < medium_speed_angle)
        return SPEED_175;
    return SPEED_FULL;
}

int GoalPlugin::get_position_speed (const float distance)
{
    if (distance < slow_speed_distance)
    {
        return SPEED_150;
    }
    else if (distance < medium_speed_distance)
    {
        return SPEED_175;
    }
    return SPEED_FULL;
}
