/*
 * GoalPlugin.cpp
 *
 *  Created on: Jan 1, 2023
 *      Author: cre
 */

#include "Car.hpp"
#include "GoalPlugin.hpp"

#include "../robot/robot_math.hpp"
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

void GoalPlugin::reset ()
{
    setup();
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
        if (adjust_angle)
        {
            angle_cycle(goal_angle);
        }
        if (adjust_position)
        {
            position_cycle();
        }
    }
}

bool GoalPlugin::angle_cycle (const float desired_angle)
{
    const float current_angle = car.get_kalman_plugin()->get_angle();

    if (!is_overflow(current_angle))
    {
        const float delta = angle_delta(desired_angle, current_angle);
        const float abs_delta = abs(delta);
        if (abs_delta > angle_tolerance)
        {
            const int speed = get_angle_speed(abs_delta);
            if (delta > 0)
            {
                cout << "Clockwise by " << delta << " from " << current_angle << " to goal angle "
                        << desired_angle << " at " << speed << std::endl;
                cout1 << "Clockwise by " << delta << " from " << current_angle << " to goal angle "
                        << desired_angle << " at " << speed << std::endl;
                car.drive_reverse(MotorLocation::RIGHT, speed);
                car.drive_forward(MotorLocation::LEFT, speed);
            }
            else
            {
                cout << "Counterclockwise by " << delta << " from " << current_angle
                        << " to goal angle " << desired_angle << " at " << speed << std::endl;
                cout1 << "Counterclockwise by " << delta << " from " << current_angle
                        << " to goal angle " << desired_angle << " at " << speed << std::endl;
                car.drive_forward(MotorLocation::RIGHT, speed);
                car.drive_reverse(MotorLocation::LEFT, speed);
            }
        }
        else
        {
            cout << "Car " << car << " at " << current_angle << " is goal angle " << desired_angle
                    << std::endl;
            cout1 << "Car " << car << " at " << current_angle << " is goal angle " << desired_angle
                    << std::endl;
            car.drive_stop(MotorLocation::RIGHT);
            car.drive_stop(MotorLocation::LEFT);
            adjust_angle = false;
            return false;
        }
    }
    else
    {
        cout << "Invalid imu reading " << current_angle << std::endl;
    }
    return true;

}

bool GoalPlugin::position_cycle ()
{
    const float current_angle = car.get_kalman_plugin()->get_angle();

    if (!is_overflow(current_angle))
    {
        const float current_x = car.get_kalman_plugin()->get_x();
        const float current_y = car.get_kalman_plugin()->get_y();
        const float dx = goal_x - current_x;
        const float dy = goal_y - current_y;
        if (abs(dx) < position_tolerance && abs(dy) < position_tolerance)
        {
            cout << "Robot " << car << " is at goal " << goal_x << ", " << goal_y << std::endl;
            cout1 << "Robot " << car << " is at goal " << goal_x << ", " << goal_y << std::endl;
            car.drive_stop(MotorLocation::RIGHT);
            car.drive_stop(MotorLocation::LEFT);
            adjust_position = false;
            return false;
        }
        const float desired_angle = atan2(dy, dx);
        const float delta_angle = angle_delta(desired_angle, current_angle);
        const float abs_delta_angle = abs(delta_angle);
        const float distance = sqrt(dx * dx + dy * dy);

        cout << "Required " << car << " dx " << dx << " dy " << dy << " angle " << desired_angle
                << " for " << distance << " to " << goal_x << ", " << goal_y << std::endl;
        cout1 << "Required " << car << " dx " << dx << " dy " << dy << " angle " << desired_angle
                << " for " << distance << " to " << goal_x << ", " << goal_y << std::endl;

        const int speed = get_position_speed(distance);

        if (abs_delta_angle > 0.5)
        {
            angle_cycle(desired_angle);
        }
        else if (abs_delta_angle < angle_tolerance)
        {
            car.drive_forward(MotorLocation::RIGHT, speed);
            car.drive_forward(MotorLocation::LEFT, speed);
        }
        else if (delta_angle > 0)
        {
            const int speed = get_angle_speed(abs_delta_angle);
            cout << "Drive Clockwise by " << delta_angle << " from " << current_angle
                    << " to goal angle " << desired_angle << " at " << speed << std::endl;
            cout1 << "Drive Clockwise by " << delta_angle << " from " << current_angle
                    << " to goal angle " << desired_angle << " at " << speed << std::endl;
            car.drive_forward(MotorLocation::LEFT, speed);
            car.drive_forward(MotorLocation::RIGHT, (speed * 3) / 4);
        }
        else
        {
            const int speed = get_angle_speed(abs_delta_angle);
            cout << "Drive Counterclockwise from " << current_angle << " to goal angle "
                    << desired_angle << " at " << speed << std::endl;
            cout1 << "Drive Counterclockwise from " << current_angle << " to goal angle "
                    << desired_angle << " at " << speed << std::endl;
            car.drive_forward(MotorLocation::RIGHT, speed);
            car.drive_forward(MotorLocation::LEFT, (speed * 3) / 4);
        }
    }
    return true;
}

int GoalPlugin::get_angle_speed (const float angle)
{
    if (angle < slow_speed_angle)
        return SPEED_HALF;
    else if (angle < medium_speed_angle)
        return SPEED_160;
    return SPEED_FULL;
}

int GoalPlugin::get_position_speed (const float distance)
{
    if (distance < slow_speed_distance)
    {
        return SPEED_HALF;
    }
    else if (distance < medium_speed_distance)
    {
        return SPEED_160;
    }
    return SPEED_FULL;
}
