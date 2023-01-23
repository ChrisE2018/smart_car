/*
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
        WallPlugin ();
        virtual void enter_state (const int state) override;
        virtual void cycle () override;

    private:
        int speed = 0;
};

