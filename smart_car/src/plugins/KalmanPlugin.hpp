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

class KalmanPlugin : public Plugin
{
    public:
        KalmanPlugin (Car &car);
        bool setup () override;
        void cycle () override;
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

        Car &car;
        long t = 0;
        // Px, Py, Pa, Vx, Vy, Va, Ax, Ay, Aa
        BLA::Matrix<Nstate> state; // state vector
        BLA::Matrix<Nobs> obs; // observation vector
        BLA::Matrix<Nstate, Nstate> time_update;   // time update
        BLA::Matrix<2, 2> body_2_world;
        BLA::Matrix<2, 2> world_2_body;

        const int interval = 5;
        long deadline = 0;
};

