/*
 * WallMode.hpp
 *
 *  Created on: Dec 26, 2022
 *      Author: cre
 */

#pragma once

#include "Plugin.hpp"

class Car;

class WallPlugin: public Plugin
{
    public:
        WallPlugin (Car &car);
        void set_enabled (const bool enable);
        void cycle () override;

    private:
        Car &car;
        int speed = 0;
};

