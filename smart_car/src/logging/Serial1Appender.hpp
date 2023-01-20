/*
 * Serial1Appender.hpp
 *
 *  Created on: Jan 20, 2023
 *      Author: cre
 */

#pragma once

#include "Appender.hpp"

class Serial1Appender: public Appender
{
    public:
        Serial1Appender (const Level level, Formatter &formatter);
        virtual ~Serial1Appender () = default;
        virtual void append (const Level level, const char *message) override;
};

