/*
 * WallMode.hpp
 *
 *  Created on: Dec 26, 2022
 *      Author: cre
 */

#pragma once

#include "Plugin.hpp"

class Car;

class WallMode: public Plugin
{
    public:
        WallMode (Car &car);

        void set_mode (Mode mode) override;
        void cycle () override;

    private:
        Car &car;
        bool active = false;
        int speed = 0;
};

