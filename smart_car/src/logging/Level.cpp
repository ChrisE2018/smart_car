/*
 * Level.cpp
 *
 *  Created on: Jan 20, 2023
 *      Author: cre
 */

#include "Level.hpp"

const char* stringify (const Level level)
{
    switch (level)
    {
        case Level::error:
            return "error";
        case Level::warning:
            return "warning";
        case Level::info:
            return "info";
        case Level::debug:
            return "debug";
        case Level::data:
            return "data";
        case Level::none:
            return "none";
        default:
            return "?level?";
    }
}

