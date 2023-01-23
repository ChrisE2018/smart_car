/*
 * NavigationPlugin.hpp
 *
 *  Created on: Dec 28, 2022
 *      Author: cre
 */

#pragma once

#include <Arduino.h>
#include <BasicLinearAlgebra.h>
#include "Plugin.hpp"

class Car;

// Dimensions of the matrices
#define Nstate 9 // length of the state vector
#define Nobs 9   // length of the measurement vector

class KalmanPlugin: public Plugin
{
    public:
        KalmanPlugin ();
        bool setup () override;
        virtual int get_preferred_interval () const;
        virtual int get_expected_us () const;
        void cycle () override;
        void trace () override;
        void update_transforms (const float angle);
        float get_x () const;
        float get_y () const;
        float get_angle () const;
        float get_dx () const;
        float get_dy () const;
        float get_dangle () const;
        float get_ax () const;
        float get_ay () const;
        float get_aangle () const;
        const BLA::Matrix<Nstate>& get_state () const;

    private:
        unsigned long t = 0;
        float dt = 0;
        // Px, Py, Pa, Vx, Vy, Va, Ax, Ay, Aa
        BLA::Matrix<Nstate> state; // state vector
        BLA::Matrix<Nobs> odom_obs; // observation vector
        BLA::Matrix<Nobs> odom_gain; // observation vector
        BLA::Matrix<Nobs> imu_obs; // observation vector
        BLA::Matrix<Nobs> imu_gain; // observation vector
        BLA::Matrix<Nobs> total_gain;
        BLA::Matrix<Nobs> state_gain;
        BLA::Matrix<Nobs> state_update;
        BLA::Matrix<Nstate, Nstate> time_update;   // time update
        BLA::Matrix<2, 2> body_2_world;
        BLA::Matrix<2, 2> world_2_body;

        static const float angular_meters_2_angular_radians = M_PI / 0.534;
        const int interval = 100; // ms
        unsigned long deadline = 0;
        void hadamard (const BLA::Matrix<Nobs> &a, const BLA::Matrix<Nobs> &b, BLA::Matrix<Nobs> &result) const;

};

