/*
 * Serial2Appender.hpp
 *
 *  Created on: Jan 20, 2023
 *      Author: cre
 */

#pragma once

#include "Appender.hpp"

class Serial2Appender: public Appender
{
    public:
        Serial2Appender (const Level level, Formatter &formatter);
        virtual ~Serial2Appender () = default;
        virtual void append (const Level level, const char *message) override;
};

