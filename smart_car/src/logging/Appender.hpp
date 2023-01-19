/*
 * Appender.hpp
 *
 *  Created on: Jan 5, 2023
 *      Author: cre
 */

#pragma once

#include "Logger.hpp"
#include "Formatter.hpp"

class Appender
{
    public:
        Appender ();
        virtual ~Appender () = default;
        void set_formatter (Formatter *const formatter);
        virtual void append (const Logger *logger, const Level level, const int line, const char *message);
        virtual void append (const Level level, const char *message);
    private:
        static const int buffer_size = 256;
        char buffer[buffer_size];
        Formatter *formatter = nullptr;
};

