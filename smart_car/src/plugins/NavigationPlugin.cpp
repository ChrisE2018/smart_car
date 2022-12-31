/*
 * NavigationPlugin.cpp
 *
 *  Created on: Dec 28, 2022
 *      Author: cre
 */

#include "NavigationPlugin.hpp"
#include "Car.hpp"

//------------------------------------
/****       KALMAN PARAMETERS    ****/
//------------------------------------
// measurement std (to be characterized from your sensors)
#define n1 0.1 // noise on the 1st measurement component
#define n2 0.1 // noise on the 2nd measurement component

// model std (~1/inertia). Freedom you give to relieve your evolution equation
#define m1 0.01
#define m2 0.01
#define m3 0.01

NavigationPlugin::NavigationPlugin (Car &car) : Plugin(NAVIGATION_PLUGIN), car(car)
{
}

bool NavigationPlugin::setup ()
{
    set_enabled(false);

    // Initial state of the system
    // Px, Py, Pa, Vx, Vy, Va, Ax, Ay, Aa
    K.x =
    { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };

    // model matrix. Size is <Nstate, Nstate>
    K.F =
    { 1.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0,       // Px = Px + Vx
            0.0, 1.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, // Py = Py + Vy
            0.0, 0.0, 1.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, // Pa = Pa + Va
            0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 1.0, 0.0, 0.0, // Vx = Vx + Ax
            0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 1.0, 0.0, // Vy = Vy + Ay
            0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 1.0, // Va = Va + Aa
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, // Ax = Ax
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, // Ay = Ay
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0  // Aa = Aa
            };

    // measurement matrix. Size is <Nobs, Nstate>
    K.H =
    { 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,       //
            0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, //
            0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, //
            0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, //
            0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, //
            0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, //
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, //
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, //
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, //
            };

    // measurement covariance matrix. Size is <Nobs, Nobs>
    K.R =
    { n1 * n1, n1 * n1, n1 * n1, n1 * n1, n1 * n1, n1 * n1, n1 * n1, n1 * n1, n1 * n1,  //
            n1 * n1, n1 * n1, n1 * n1, n1 * n1, n1 * n1, n1 * n1, n1 * n1, n1 * n1, n1 * n1,  //
            n1 * n1, n1 * n1, n1 * n1, n1 * n1, n1 * n1, n1 * n1, n1 * n1, n1 * n1, n1 * n1,  //
            n1 * n1, n1 * n1, n1 * n1, n1 * n1, n1 * n1, n1 * n1, n1 * n1, n1 * n1, n1 * n1,  //
            n1 * n1, n1 * n1, n1 * n1, n1 * n1, n1 * n1, n1 * n1, n1 * n1, n1 * n1, n1 * n1,  //
            n1 * n1, n1 * n1, n1 * n1, n1 * n1, n1 * n1, n1 * n1, n1 * n1, n1 * n1, n1 * n1,  //
            n1 * n1, n1 * n1, n1 * n1, n1 * n1, n1 * n1, n1 * n1, n1 * n1, n1 * n1, n1 * n1,  //
            n1 * n1, n1 * n1, n1 * n1, n1 * n1, n1 * n1, n1 * n1, n1 * n1, n1 * n1, n1 * n1,  //
            n1 * n1, n1 * n1, n1 * n1, n1 * n1, n1 * n1, n1 * n1, n1 * n1, n1 * n1, n1 * n1,  //
            };

    // model covariance matrix. Size is <Nstate, Nstate>
    K.Q =
    { m1 * m1, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, //
            0.0, m1 * m1, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, //
            0.0, 0.0, m1 * m1, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, //
            0.0, 0.0, 0.0, m1 * m1, 0.0, 0.0, 0.0, 0.0, 0.0, //
            0.0, 0.0, 0.0, 0.0, m1 * m1, 0.0, 0.0, 0.0, 0.0, //
            0.0, 0.0, 0.0, 0.0, 0.0, m1 * m1, 0.0, 0.0, 0.0, //
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0, m1 * m1, 0.0, 0.0, //
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, m1 * m1, 0.0, //
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, m1 * m1, //
            };

    return true;
}

void NavigationPlugin::reset ()
{
    setup();
}

void NavigationPlugin::cycle ()
{
    const long now = millis();
    const float dt = (now - t) * 0.001;

    update_transforms(K.x(2));

// GRAB MEASUREMENT and WRITE IT INTO 'obs'
    get_sensor_data();

// APPLY KALMAN FILTER
    K.update(obs);

    if (is_enabled())
    {
        // PRINT RESULTS: measures and estimated state
        Serial << obs << ' ' << K.x << " b2w " << body_2_world << "\n";
    }
}

void NavigationPlugin::get_sensor_data ()
{
    const Motor &right_motor = car.get_motor(RIGHT);
    const Motor &left_motor = car.get_motor(LEFT);
    const float right_velocity = right_motor.get_velocity();
    const float left_velocity = left_motor.get_velocity();

    // Counterclockwise
    const float angular_velocity = right_velocity - left_velocity;

    // TODO: Scale from motor velocity to meters per second.
    // TODO: estimate rotational velocity from motor difference.
    // TODO: verify that rotational directions are correct
    BLA::Matrix<2> body_velocity =
    { right_velocity, left_velocity };
    BLA::Matrix<2> world_velocity = body_2_world * body_velocity;
    obs =
    { K.x(0), K.x(1), K.x(2), world_velocity(0), world_velocity(1), angular_velocity, K.x(6), K.x(
            7), K.x(8) };
}

void NavigationPlugin::update_transforms (const float angle)
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
