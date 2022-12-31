/*
 * OdomPlugin.hpp
 *
 *  Created on: Dec 31, 2022
 *      Author: cre
 */

#pragma once

#include <BasicLinearAlgebra.h>
#include "Plugin.hpp"

class Car;

class OdomPlugin : public Plugin
{
    public:
        OdomPlugin (Car &car);
        bool setup () override;
        void reset () override;
        void cycle () override;
        void update_transforms (const float angle);

    private:
        Car &car;
        long t = 0;
        // x, y, angle, dx, dy, da
        BLA::Matrix<6> state;
        // dx, dy, da
        BLA::Matrix<6> obs;
        BLA::Matrix<6, 6> time_update;   // time update

        BLA::Matrix<2, 2> body_2_world;
        BLA::Matrix<2, 2> world_2_body;
};