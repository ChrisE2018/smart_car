/*
 * WallMode.hpp
 *
 *  Created on: Dec 26, 2022
 *      Author: cre
 */

#pragma once

#include "Car.hpp"
#include "Cyclic.hpp"

class WallMode: public Cyclic
{
    public:
        WallMode (Car &car);

        void set_mode(Mode mode) override;
        void cycle () override;

    private:
        Car &car;
        int speed = 0;
};

