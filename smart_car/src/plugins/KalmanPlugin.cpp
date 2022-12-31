/*
 * NavigationPlugin.cpp
 *
 *  Created on: Dec 28, 2022
 *      Author: cre
 */

#include "KalmanPlugin.hpp"

#include "Car.hpp"

// This is needed or the matrices won't print
using namespace BLA;

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

KalmanPlugin::KalmanPlugin (Car &car) : Plugin(KALMAN_PLUGIN), car(car)
{
}

// Use this to get dmp values from IMU
// https://mjwhite8119.github.io/Robots/mpu6050

bool KalmanPlugin::setup ()
{
    set_enabled(false);

    // Initial state of the system
    // Px, Py, Pa, Vx, Vy, Va, Ax, Ay, Aa
    state =
    { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };

    // Model matrix to compute the update delta for each dt.
    // Size is <Nstate, Nstate>
    time_update =
    { 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0,       // Px += dt * Vx
            0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, // Py += dt * Vy
            0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, // Pa += dt * Va
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, // Vx += dt * Ax
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, // Vy += dt * Ay
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, // Va += dt * Aa
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, // Ax += 0
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, // Ay += 0
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0  // Aa += 0
            };

    t = millis();
    return true;
}

void KalmanPlugin::reset ()
{
    setup();
}

void KalmanPlugin::cycle ()
{
    const long now = millis();
    const float dt = (now - t) * 0.001;

    const Motor &right_motor = car.get_motor(RIGHT);
    const Motor &left_motor = car.get_motor(LEFT);
    const float right_velocity = right_motor.get_velocity();
    const float left_velocity = left_motor.get_velocity();

    // Rotation from body to world for current angle
    update_transforms(state(2));

    const BLA::Matrix<2> body_velocity =
    { right_velocity, left_velocity };
    const BLA::Matrix<2> world_velocity = body_2_world * body_velocity;
    const float odom_vx = world_velocity(0);
    const float odom_vy = world_velocity(1);
    const float angular_velocity = left_velocity - right_velocity;  // Clockwise
    // Get yaw and acceleration from MPU
    MpuPlugin *mpu_plugin = car.get_mpu_plugin();
    const float mpu_yaw = mpu_plugin->get_yaw();  // clockwise
    const float mpu_Ax = mpu_plugin->get_Ax();
    const float mpu_Ay = mpu_plugin->get_Ay();
    obs =
    { state(0), state(1), mpu_yaw, odom_vx, odom_vy, angular_velocity, mpu_Ax, mpu_Ay, 0.0 };

    state = (state + obs) * 0.5;

    state = state + time_update * state * dt;

    t = now;

    if (is_enabled())
    {
        // PRINT RESULTS: measures and estimated state
        Serial << "Nav State: " << state << " dt: " << dt << " Obs: " << obs
        // << " rm: " << right_velocity << " lm: " << left_velocity << " b2w" << body_2_world
                << "\n";
    }
}

void KalmanPlugin::update_transforms (const float angle)
{
    const float sin_angle = sin(angle);
    const float cos_angle = cos(angle);
    // Clockwise
    body_2_world =
    { cos_angle, sin_angle, sin_angle, -cos_angle };
    // Counterclockwise
    world_2_body =
    { cos_angle, -sin_angle, sin_angle, cos_angle };
}

float KalmanPlugin::get_x ()
{
    return state(0);
}

float KalmanPlugin::get_y ()
{
    return state(1);
}

float KalmanPlugin::get_angle ()
{
    return state(2);
}

float KalmanPlugin::get_dx ()
{
    return state(3);
}

float KalmanPlugin::get_dy ()
{
    return state(4);
}

float KalmanPlugin::get_dangle ()
{
    return state(5);
}

float KalmanPlugin::get_ax ()
{
    return state(6);
}

float KalmanPlugin::get_ay ()
{
    return state(7);
}

float KalmanPlugin::get_aangle ()
{
    return state(8);
}
