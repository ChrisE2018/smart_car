/*
 * NavigationPlugin.hpp
 *
 *  Created on: Dec 28, 2022
 *      Author: cre
 */

#pragma once

#include "Plugin.hpp"
#include <Kalman.h>
#ifndef Car
    class Car;
#endif

//------------------------------------
/****       KALMAN PARAMETERS    ****/
//------------------------------------
// Dimensions of the matrices
#define Nstate 9 // length of the state vector
#define Nobs 9   // length of the measurement vector

// Note: I made 'obs' a global variable so memory is allocated before the loop.
//       This might provide slightly better speed efficiency in loop.

class NavigationPlugin : public Plugin
{
    public:
        NavigationPlugin (const PluginId id, Car &car);
        bool setup () override;
        void reset () override;
        void cycle () override;
        void update_transforms ();

    private:

        Car &car;
        KALMAN<Nstate, Nobs> K; // your Kalman filter
        BLA::Matrix<Nobs> obs; // observation vector
        BLA::Matrix<2,2> body_2_world;
        BLA::Matrix<2,2> world_2_body;

        void get_sensor_data ();
};

