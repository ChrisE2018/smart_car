/*
 * NavigationPlugin.cpp
 *
 *  Created on: Dec 28, 2022
 *      Author: cre
 */

#include "KalmanPlugin.hpp"
#include "Robot.hpp"

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

KalmanPlugin::KalmanPlugin () :
        Plugin(PluginId::KALMAN_PLUGIN)
{
}

// Use this to get dmp values from IMU
// https://mjwhite8119.github.io/Robots/mpu6050

bool KalmanPlugin::setup ()
{
    set_state(Plugin::DISABLE);

    // Initial state of the system
    // Px, Py, Pa, Vx, Vy, Va, Ax, Ay, Aa
    state.Fill(0);

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

    odom_gain =
    { 0, 0, 0, 0.5, 0.5, 0.0, 0.0, 0.0, 0.0 };

    imu_gain =
    { 0, 0, 0.5, 0, 0, 0, 0.01, 0.01, 0.01 };

    total_gain = odom_gain + imu_gain;
    BLA::Matrix<Nobs> unit_gain;
    unit_gain.Fill(1);
    state_gain = unit_gain = total_gain;
    t = millis();
    return true;
}

int KalmanPlugin::get_preferred_interval () const
{
    return 10;
}

int KalmanPlugin::get_expected_us () const
{
    return 3800;
}

void KalmanPlugin::cycle ()
{
    const unsigned long current_millis = millis();
    if (deadline < current_millis)
    {
        dt = (current_millis - t) * 0.001;

        // Rotation from body to world for current angle
        update_transforms(state(2));

        const float right_velocity = get_pid_plugin(MotorLocation::RIGHT_FRONT).get_measured_velocity();
        const float left_velocity = get_pid_plugin(MotorLocation::LEFT_FRONT).get_measured_velocity();

        const BLA::Matrix<2> body_velocity =
        { (right_velocity + left_velocity) * 0.5, 0 };
        const BLA::Matrix<2> world_velocity = body_2_world * body_velocity;
        const float odom_vx = world_velocity(0);
        const float odom_vy = world_velocity(1);
        // Convert from mps to radians per second
//        const float angular_velocity = (left_velocity - right_velocity) * angular_meters_2_angular_radians; // Clockwise
        // Get yaw and acceleration from MPU
        const float mpu_yaw = mpu_plugin.get_yaw();  // clockwise
        // Verify that mpu_Ax and mpu_Ay are in the right frame of reference
        const float mpu_Ax = mpu_plugin.get_Ax();
        const float mpu_Ay = mpu_plugin.get_Ay();

//        Serial.print(F("("));
//        Serial.print(left_velocity);
//        Serial.print(F(" - "));
//        Serial.print(right_velocity);
//        Serial.print(F(") * "));
//        Serial.print(angular_meters_2_angular_radians);
//        Serial.print(F(" = "));
//        Serial.println(angular_velocity);

        // Separate obs by sensor (odom, mpu) and define gain as a vector
        odom_obs =
        { 0, 0, 0, odom_vx, odom_vy, 0.0, 0.0, 0.0, 0.0 };
        imu_obs =
        { 0, 0, mpu_yaw, 0, 0, 0, mpu_Ax, mpu_Ay, atan2(mpu_Ay, mpu_Ax) };

        state_update.Fill(0);
        hadamard(odom_obs, odom_gain, state_update);
        hadamard(imu_obs, imu_gain, state_update);
        hadamard(state, state_gain, state_update);

        state = state_update + time_update * state_update * dt;

        t = current_millis;
        deadline = current_millis + interval;
    }
}

void KalmanPlugin::trace ()
{
    // PRINT RESULTS: measures and estimated state
    Serial << "Kalman State: " << state << " dt: " << dt << " Odom: " << odom_obs << " Imu: " << imu_obs
    // << " rm: " << right_velocity << " lm: " << left_velocity << " b2w" << body_2_world
            << "\n";
}

void KalmanPlugin::update_transforms (const float angle)
{
    const float sin_angle = sin(angle);
    const float cos_angle = cos(angle);
    body_2_world =
    // Rotate Clockwise
            { cos_angle, sin_angle, sin_angle, -cos_angle };
    world_2_body =
    // Rotate Counterclockwise
            { cos_angle, -sin_angle, sin_angle, cos_angle };
}

void KalmanPlugin::hadamard (const BLA::Matrix<Nobs> &a, const BLA::Matrix<Nobs> &b, BLA::Matrix<Nobs> &result) const
{
    for (int i = 0; i < Nobs; i++)
    {
        result(i) += a(i) * b(i);
    }
}

float KalmanPlugin::get_x () const
{
    return state(0);
}

float KalmanPlugin::get_y () const
{
    return state(1);
}

float KalmanPlugin::get_angle () const
{
    return state(2);
}

float KalmanPlugin::get_dx () const
{
    return state(3);
}

float KalmanPlugin::get_dy () const
{
    return state(4);
}

float KalmanPlugin::get_dangle () const
{
    return state(5);
}

float KalmanPlugin::get_ax () const
{
    return state(6);
}

float KalmanPlugin::get_ay () const
{
    return state(7);
}

float KalmanPlugin::get_aangle () const
{
    return state(8);
}

const BLA::Matrix<Nstate>& KalmanPlugin::get_state () const
{
    return state;
}
