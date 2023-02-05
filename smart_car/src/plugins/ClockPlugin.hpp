/*
 * Copyright (c) 2023 by Christopher Eliot.
 *
 * ClockPlugin.hpp
 *
 *  Created on: Dec 28, 2022
 *      Author: cre
 */

#pragma once

#include "Plugin.hpp"
#include <DS3231.h>
#include <ctime>
#include "TimeSource.hpp"

class ClockPlugin: public Plugin, public logging::TimeSource
{
    public:
        ClockPlugin ();
        bool setup () override;
        virtual bool is_cyclic () const override;
        virtual time_t unixtime () override;

    private:
        const bool enable_clock = true;
        bool is_setup = false;
        const char *format = "Y-m-d H:i:s";
        DS3231 clock;
        RTCDateTime dt;
};

