/*
 * TimeSource.hpp
 *
 *  Created on: Jan 19, 2023
 *      Author: cre
 */

#pragma once

#include <time.h>

namespace logging
{

class TimeSource
{
    public:
        TimeSource ();
        virtual ~TimeSource () = default;

        virtual time_t get_unixtime();
};

}
