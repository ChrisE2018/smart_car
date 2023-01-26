/*
 * OdomPlugin.cpp
 *
 *  Created on: Dec 31, 2022
 *      Author: cre
 */

#include "../robot/Car.hpp"
#include "OdomPlugin.hpp"

// This is needed to make matrices print
using namespace BLA;

OdomPlugin::OdomPlugin (Car &car) :
        Plugin(PluginId::ODOM_PLUGIN), car(car), t(0)
{

}

bool OdomPlugin::setup ()
{
    set_state(Plugin::DISABLE);

    // x, y, a, dx, dy, da
    state =
    { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };

//    // x, y, a, dx, dy, da
//    obs =
//    { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
//    time_update =
//    { 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, //
//            0.0, 0.0, 0.0, 0.0, 1.0, 0.0, //
//            0.0, 0.0, 0.0, 0.0, 0.0, 1.0, //
//            0.0, 0.0, 0.0, 0.0, 0.0, 0.0, //
//            0.0, 0.0, 0.0, 0.0, 0.0, 0.0, //
//            0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };

    t = millis();
    return true;
}

int OdomPlugin::get_preferred_interval () const
{
    return 50;
}

int OdomPlugin::get_expected_us () const
{
    return 400;
}

void OdomPlugin::cycle ()
{
    const unsigned long now = millis();
    dt = (now - t) * 0.001;
    t = now;

    right_velocity = car.get_pid_plugin(MotorLocation::RIGHT_FRONT).get_measured_velocity();
    left_velocity = car.get_pid_plugin(MotorLocation::LEFT_FRONT).get_measured_velocity();

    // clockwise
    // https://en.wikipedia.org/wiki/Differential_wheeled_robot
    const float angular_velocity = (left_velocity - right_velocity) / 0.13;

    const BLA::Matrix<2> body_velocity =
    { (right_velocity + left_velocity) * 0.5, 0.0 };
    update_transforms(state(2));
    const BLA::Matrix<2> world_velocity = body_2_world * body_velocity;
    const float dx = world_velocity(0);
    const float dy = world_velocity(1);
//    obs =
//    { state(0), state(1), state(2), (state(3) + dx) * 0.5, (state(4) + dy) * 0.5, (state(5) + angular_velocity) * 0.5 };
//
//    state = (state + obs) * 0.5;

    // Combine with observed velocity values
    state(3) = (state(3) + dx) * 0.5;
    state(4) = (state(4) + dy) * 0.5;
    state(5) = (state(5) + angular_velocity) * 0.5;

//    state = state + time_update * state * dt;
    // Integrate velocity and add to position
    state(0) = state(0) + state(3) * dt;
    state(1) = state(1) + state(4) * dt;
    state(2) = state(2) + state(5) * dt;
}

void OdomPlugin::trace ()
{
    // PRINT RESULTS: measures and estimated state
    Serial << "Odom State: " << state << " dt: " << dt << " rm: " << right_velocity << " lm: "
            << left_velocity << " b2w" << body_2_world << "\n";
}

void OdomPlugin::update_transforms (const float angle)
{
    const float sin_angle = sin(angle);
    const float cos_angle = cos(angle);
    // Counterclockwise
    body_2_world =
    { cos_angle, -sin_angle, sin_angle, cos_angle };
    // Clockwise
    world_2_body =
    { cos_angle, sin_angle, -sin_angle, cos_angle };
}
