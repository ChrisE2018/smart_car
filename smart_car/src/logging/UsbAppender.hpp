/*
 * UsbAppender.hpp
 *
 *  Created on: Jan 19, 2023
 *      Author: cre
 */

#pragma once

#include "Appender.hpp"

class UsbAppender: public Appender
{
    public:
        UsbAppender (const Level level, Formatter &formatter);
        virtual ~UsbAppender () = default;
        virtual void append (const Level level, const char *message) override;
};

