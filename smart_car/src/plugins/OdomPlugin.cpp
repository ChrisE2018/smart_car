/*
 * OdomPlugin.cpp
 *
 *  Created on: Dec 31, 2022
 *      Author: cre
 */

#include "Car.hpp"
#include "OdomPlugin.hpp"

OdomPlugin::OdomPlugin (Car &car) : Plugin(ODOM_PLUGIN), car(car), t(0)
{

}

bool OdomPlugin::setup ()
{
    set_enabled(true);

    // x, y, a, dx, dy, da
    state =
    { 1.0, 0.0, 0.0, 0.0, 0.0, 0.0 };

    // x,y, a,dx, dy, da
    obs =
    { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
    time_update =
    { 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, //
            0.0, 0.0, 0.0, 0.0, 1.0, 0.0, //
            0.0, 0.0, 0.0, 0.0, 0.0, 1.0, //
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0, //
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0, //
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };

    t = millis();
    return true;
}

void OdomPlugin::reset ()
{
    setup();
}

void OdomPlugin::cycle ()
{
    const long now = millis();
    const float dt = (now - t) * 0.001;

    const Motor &right_motor = car.get_motor(RIGHT);
    const Motor &left_motor = car.get_motor(LEFT);
    const float right_velocity = right_motor.get_velocity();
    const float left_velocity = left_motor.get_velocity();

// Counterclockwise
    const float angular_velocity = right_velocity - left_velocity;

// TODO: Scale from motor velocity to meters per second.
// TODO: estimate rotational velocity from motor difference.
// TODO: verify that rotational directions are correct
    const BLA::Matrix<2> body_velocity =
    { right_velocity, left_velocity };
    update_transforms(state(2));
    const BLA::Matrix<2> world_velocity = body_2_world * body_velocity;
    const float dx = world_velocity(0);
    const float dy = world_velocity(1);
    obs =
    { state(0), state(1), state(2), dx, dy, angular_velocity };

    state = (state + obs) * 0.5;

    state = state + time_update * state * dt;

    t = now;

    if (is_enabled())
    {
        // PRINT RESULTS: measures and estimated state
        Serial << " State: " << state << "dt: " << dt << " Obs: " << obs << " rm: "
                << right_velocity << " lm: " << left_velocity << " b2w" << body_2_world << "\n";
    }
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
    { cos_angle, sin_angle, sin_angle, -cos_angle };
}