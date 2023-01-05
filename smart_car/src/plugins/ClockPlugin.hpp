/*
 * ClockPlugin.hpp
 *
 *  Created on: Dec 28, 2022
 *      Author: cre
 */

#pragma once

#include "Plugin.hpp"
#include <DS3231.h>

class ClockPlugin: public Plugin
{
    public:
        ClockPlugin ();
        bool setup() override;
        void cycle () override;

    private:
        const bool enable_clock = true;
        DS3231 clock;
        RTCDateTime dt;
};

