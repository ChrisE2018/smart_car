/*
 * LogBuffer.hpp
 *
 *  Created on: Jan 5, 2023
 *      Author: cre
 */

#pragma once

#include <streambuf>
#include <iostream>
#include <string>
#include <WString.h>
#include "Level.hpp"

class Logger;

// http://www.angelikalanger.com/Articles/C++Report/IOStreamsDerivation/IOStreamsDerivation.html
// https://gcc.gnu.org/onlinedocs/libstdc++/manual/streambufs.html
std::ostream& operator<< (std::ostream &lhs, const __FlashStringHelper *pstr);

class LogBuffer: private std::streambuf, public std::ostream
{
    public:
        // Share the private buffer to save sram
//        friend void Logger::logging (const Level _level, const int line, const char *format, ...);
//        friend void Logger::logging_p (const Level _level, const int line, const char *format, ...);
        LogBuffer ();
        void reset ();
        void set_logger (Logger *const logger);
        void set_level (const Level level);
        void set_line (const int line);

    protected:
        virtual std::streambuf::int_type overflow (const std::streambuf::int_type c) override;

    private:
        static const int buffer_size = 128;
        static char buffer[buffer_size];
        static int pos;
        Logger *logger;
        Level level;
        int line = 0;

        void flush ();
};
