/*
 * GoalPlugin.cpp
 *
 *  Created on: Jan 1, 2023
 *      Author: cre
 */

#include "Car.hpp"
#include "GoalPlugin.hpp"
#include "KalmanPlugin.hpp"
#include "smart_car.hpp"
#include "math.hpp"

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
    set_enabled(true);
    cout << "Set " << car << " goal angle " << goal_angle << std::endl;
}

void GoalPlugin::set_goal (const float x, const float y)
{
    goal_x = x;
    goal_y = y;
    adjust_position = true;
    set_enabled(true);
    cout << "Set " << car << " goal position " << x << ", " << y << std::endl;
}

void GoalPlugin::cycle ()
{
    if (is_enabled())
    {
        if (adjust_angle)
        {
            const float current_angle = car.get_kalman_plugin()->get_angle();

            if (!is_overflow(current_angle))
            {
                const float delta = current_angle - goal_angle;
                const float abs_delta = abs(delta);
                if (abs_delta > angle_tolerance)
                {
                    int speed = SPEED_FULL;
                    if (abs_delta < slow_speed_angle) speed = SPEED_160;
                    else if (abs_delta < medium_speed_angle) speed = SPEED_Q3;
                    if (goal_angle > current_angle)
                    {
                        cout << "Clockwise from " << current_angle << " to goal angle "
                                << goal_angle << " at " << speed << std::endl;
                        car.drive_reverse(MotorLocation::RIGHT, speed);
                        car.drive_forward(MotorLocation::LEFT, speed);
                    }
                    else
                    {
                        cout << "Counterclockwise from " << current_angle << " to goal angle "
                                << goal_angle << " at " << speed << std::endl;
                        car.drive_forward(MotorLocation::RIGHT, speed);
                        car.drive_reverse(MotorLocation::LEFT, speed);
                    }
                }
                else
                {
                    cout << "Car " << car << " at " << current_angle << " is goal angle "
                            << goal_angle << std::endl;
                    car.drive_stop(MotorLocation::RIGHT);
                    car.drive_stop(MotorLocation::LEFT);
                    set_enabled(false);
                }
            }
            else
            {
                cout << "Invalid imu reading " << current_angle << std::endl;
            }
        }
    }
}
