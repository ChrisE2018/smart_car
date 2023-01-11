/*
 * OdomPlugin.cpp
 *
 *  Created on: Dec 31, 2022
 *      Author: cre
 */

#include "../robot/Car.hpp"
#include "OdomPlugin.hpp"

// This is needed or the matrices won't print
using namespace BLA;

OdomPlugin::OdomPlugin (Car &car) :
                Plugin(PluginId::ODOM_PLUGIN), car(car), t(0)
{

}

bool OdomPlugin::setup ()
{
    set_enabled(false);

    // x, y, a, dx, dy, da
    state =
    {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

    // x, y, a, dx, dy, da
    obs =
    {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    time_update =
    {0.0, 0.0, 0.0, 1.0, 0.0, 0.0, //
            0.0, 0.0, 0.0, 0.0, 1.0, 0.0, //
            0.0, 0.0, 0.0, 0.0, 0.0, 1.0, //
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0, //
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0, //
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

    t = millis();
    return false;
}

int OdomPlugin::get_preferred_interval () const
{
    return 50;
}

void OdomPlugin::cycle ()
{
    const long now = millis();
    const float dt = (now - t) * 0.001;

    const MotorPlugin &right_motor = car.get_motor(MotorLocation::RIGHT);
    const MotorPlugin &left_motor = car.get_motor(MotorLocation::LEFT);
    const float right_velocity = right_motor.get_measured_velocity();
    const float left_velocity = left_motor.get_measured_velocity();

    // clockwise
    const float angular_velocity = left_velocity - right_velocity;

    const BLA::Matrix<2> body_velocity =
    {(right_velocity + left_velocity) * 0.5, 0.0};
    update_transforms(state(2));
    const BLA::Matrix<2> world_velocity = body_2_world * body_velocity;
    const float dx = world_velocity(0);
    const float dy = world_velocity(1);
    obs =
    {state(0), state(1), state(2), dx, dy, angular_velocity};

    state = (state + obs) * 0.5;

    state = state + time_update * state * dt;

    t = now;

    if (is_enabled())
    {
        // PRINT RESULTS: measures and estimated state
        Serial << "Odom State: " << state << " dt: " << dt << " Obs: " << obs << " rm: "
                << right_velocity << " lm: " << left_velocity << " b2w" << body_2_world
                << "\n";
    }
}

void OdomPlugin::update_transforms (const float angle)
{
    const float sin_angle = sin(angle);
    const float cos_angle = cos(angle);
    // Counterclockwise
    body_2_world =
    {cos_angle, -sin_angle, sin_angle, cos_angle};
    // Clockwise
    world_2_body =
    {cos_angle, sin_angle, -sin_angle, cos_angle};
}
