/*
 * Serial3Appender.hpp
 *
 *  Created on: Jan 20, 2023
 *      Author: cre
 */

#pragma once

#include "Appender.hpp"

namespace logging
{

class Serial3Appender: public Appender
{
    public:
        Serial3Appender (const Level level, Formatter &formatter);
        virtual ~Serial3Appender () = default;
        virtual void append (const Level level, const char *message) override;
};

}
