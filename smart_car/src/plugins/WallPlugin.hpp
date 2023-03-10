/*
 * Copyright (c) 2023 by Christopher Eliot.
 *
 * WallMode.hpp
 *
 *  Created on: Dec 26, 2022
 *      Author: cre
 */

#pragma once

#include "Plugin.hpp"

class Car;

class WallPlugin : public Plugin
{
    public:
        WallPlugin (Car &car);
        virtual void enter_state (const int state) override;
        virtual void cycle () override;

    private:
        Car &car;
        int speed = 0;
};

