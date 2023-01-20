/*
 * SerialAppender.hpp
 *
 *  Created on: Jan 19, 2023
 *      Author: cre
 */

#pragma once

#include "Appender.hpp"

class SerialAppender: public Appender
{
    public:
        SerialAppender (const Level level, Formatter &formatter);
        virtual ~SerialAppender () = default;
        virtual void append (const Level level, const char *message) override;
};

