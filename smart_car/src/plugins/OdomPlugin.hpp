/*
 * OdomPlugin.hpp
 *
 *  Created on: Dec 31, 2022
 *      Author: cre
 */

#pragma once

#include <Arduino.h>
#include <BasicLinearAlgebra.h>
#include "Plugin.hpp"

class Car;

class OdomPlugin: public Plugin
{
    public:
        OdomPlugin (Car &car);
        bool setup () override;
        virtual int get_preferred_interval () const override;
        virtual int get_expected_us () const;
        void cycle () override;
        void trace () override;

    private:
        Car &car;
        unsigned long t = 0;
        float dt = 0;
        float right_distance = 0;
        float left_distance = 0;
        float right_velocity = 0;
        float left_velocity = 0;
        float angular_velocity = 0;
        float body_velocity = 0;

        // x, y, angle, dx, dy, dangle
        BLA::Matrix<6> state;
};
