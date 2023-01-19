/*
 * ClockPlugin.hpp
 *
 *  Created on: Dec 28, 2022
 *      Author: cre
 */

#pragma once

#include "Plugin.hpp"
#include <DS3231.h>
#include <ctime>
#include "../logging/TimeSource.hpp"

class ClockPlugin : public Plugin, public TimeSource
{
    public:
        ClockPlugin ();
        bool setup () override;
        virtual bool is_cyclic () const override;
        time_t get_unixtime ();

    private:
        const bool enable_clock = true;
        bool is_setup = false;
        const char *format = "Y-m-d H:i:s";
        DS3231 clock;
        RTCDateTime dt;
};

