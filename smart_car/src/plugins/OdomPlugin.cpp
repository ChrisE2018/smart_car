/*
 * OdomPlugin.cpp
 *
 *  Created on: Dec 31, 2022
 *      Author: cre
 */

#include "../robot/Car.hpp"
#include "../robot/speed_counter.hpp"
#include "OdomPlugin.hpp"

// This is needed to make matrices print
using namespace BLA;

OdomPlugin::OdomPlugin (Car &car) :
        Plugin(PluginId::ODOM_PLUGIN), car(car)
{

}

bool OdomPlugin::setup ()
{
    set_state(Plugin::DISABLE);

    // x, y, a, dx, dy, da
    state =
    { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };

    t = millis();
    return true;
}

int OdomPlugin::get_preferred_interval () const
{
    return 50;
}

int OdomPlugin::get_expected_us () const
{
    return 450;
}

void OdomPlugin::cycle ()
{
    const unsigned long now = millis();
    if (now > t)
    {
        dt = (now - t) * 0.001;
        t = now;
        const PidPlugin &rf_pid = car.get_pid_plugin(MotorLocation::RIGHT_FRONT);
        const PidPlugin &rr_pid = car.get_pid_plugin(MotorLocation::RIGHT_REAR);
        const PidPlugin &lf_pid = car.get_pid_plugin(MotorLocation::LEFT_FRONT);
        const PidPlugin &lr_pid = car.get_pid_plugin(MotorLocation::LEFT_REAR);
        const float measured_right_distance = (rf_pid.get_measured_distance() + rr_pid.get_measured_distance()) * 0.5;
        const float measured_left_distance = (lf_pid.get_measured_distance() + lr_pid.get_measured_distance()) * 0.5;
        right_velocity = (measured_right_distance - right_distance) / dt;
        left_velocity = (measured_left_distance - left_distance) / dt;
        right_distance = measured_right_distance;
        left_distance = measured_left_distance;

        // clockwise
        angular_velocity = (left_velocity - right_velocity) / wheel_spacing_sideways;
        body_velocity = (left_velocity + right_velocity) * 0.5;

        // Use midpoint angle
        const float angle = state(2) + angular_velocity * dt * 0.5;

        const float sin_angle = sin(angle);
        const float cos_angle = cos(angle);

        // See https://en.wikipedia.org/wiki/Differential_wheeled_robot for a better formula
        const float vx = body_velocity * cos_angle;
        const float vy = -body_velocity * sin_angle;

        // Integrate velocity and add to position
        state(0) = state(0) + vx * dt;
        state(1) = state(1) + vy * dt;
        state(2) = state(2) + angular_velocity * dt;

        // Set velocity to observed values
        state(3) = vx;
        state(4) = vy;
        state(5) = angular_velocity;
    }
}

void OdomPlugin::trace ()
{
    // PRINT RESULTS: measures and estimated state
    Serial << "Odom State: " << state << " dt: " << dt << " rm: " << right_velocity << " lm: " << left_velocity
            << " body velocity " << body_velocity << " angular velocity " << angular_velocity << "\n";
}
